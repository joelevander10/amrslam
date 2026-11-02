"""Branch selection at a diverter: which of the sensor's tracks to follow.

The order comes from RFID station tags, held in a PLC seal-in latch:

    Branch_right = (SetRight OR Branch_right) AND NOT ClearRight AND NOT ForkPassed AND NOT Branch_left
    Branch_left  = (SetLeft  OR Branch_left ) AND NOT ClearLeft  AND NOT ForkPassed AND NOT Branch_right
    Slow         = (SetSlow  OR Slow        ) AND NOT ClearSlow

Seal-in, explicit reset, mutual interlock. Transcribed rung for rung from the
drawn design, and deliberately kept that shape: the code is meant to be diffed
against the drawing, not merely to compute the same answer.

*** The Slow rung is NEWER THAN THE DRAWING - add it there too. *** It has no
interlock because it has no opposite coil, and it is fed by the same tag pair as
the junction it belongs to: the entry tag of any profile row carrying
slow_speed sets it, that row's exit tag clears it. So the slow zone IS the
junction zone, which is what gives the speed ramp the same lead time the branch
order already gets - the entry tag sits before the diverter for exactly that
reason. What the vehicle does with it is autopilot's business, not this file's.

The seal-in is the whole point. A tag read is a ONE-SCAN pulse - the tag is
past the reader a tick later - so without the seal-in the order would evaporate
before the junction arrived.

*** ForkPassed is why the order does not outlive the fork. *** A branch order is
direction-relative and momentary - "take the left fork" - but held to an exit tag
it becomes a standing rule, "always follow the leftmost tape", for as long as the
vehicle takes to get between the two tags. Run 0023/0024 is what that costs: the
order was still LEFT when the vehicle came out of the U-turn facing the other
way, so the MAIN LINE was now the leftmost tape and select_track() dutifully
jumped onto it - -43 -> +83 -> -57 mm, changing tape and back, every lap.

So the order is consumed by the fork itself: #LCP going back to a single track,
having been at a diverter while the order stood. That is a SENSOR pulse and needs
no tag, which is what lets the Slow rung keep the entry/exit tag pair and hold the
vehicle at auto_slow_rpm across the whole junction bubble while the steering order
ends at the fork where it belongs.

ClearLeft/ClearRight stay wired to the exit tags as a backstop. If the fork is
never detected - a diverter driven so fast that #LCP never resolves, a tape fault -
the order survives to the exit tag exactly as it used to, so the failure mode is
today's behaviour rather than a new one.

Track layout, from the MLS manual (section 8.4.1, table 17):

    "If only one line center point is found, it is output as LCP2. If a further
     line center point is found, it is output as LCP1 or LCP3, depending on its
     direction."  and  "LCP1 < LCP2 < LCP3 always applies."

  #LCP 2  LCP2         plain track
  #LCP 3  LCP1 + LCP2  diverter toward the negative side
  #LCP 6  LCP2 + LCP3  diverter toward the positive side
  #LCP 7  all three    90 deg crossing / double diverter

So the main track keeps LCP2 and the branch appears beside it. Selecting the
extreme position in the wanted direction therefore needs no case analysis on
#LCP, and degrades correctly on its own: ask for a side that is not there and
the extreme in that direction IS LCP2, the straight-through track. That is the
safe answer, and it falls out rather than being special-cased.

*** But LCP2 is only the main track while there IS a main track. *** The indices
are assigned by POSITION - "LCP1 < LCP2 < LCP3 always applies" - not by which
tape is which, so where two tapes converge and cross within the sensor window
the labels swap and LCP2 becomes the other tape. Run 0023 caught this at the
merge after a U-turn: the followed position went -43 -> +86 -> -60 mm in half a
second, switching tape and back. So STRAIGHT no longer trusts the label; it
follows the track nearest the one it was already on. See select_track().

*** A STANDING DEFAULT REPLACES THE TAG PAIR AT THE U-TURN. *** The test track
has no RFID tag at the U-turn junction, so there is no pulse to seal in and
nothing for the ladder to latch. `default` is the intent when no coil is set:
with it at RIGHT the vehicle takes the rightmost tape wherever a diverter
appears, and the U-turn is selected by the junction's geometry rather than by
being told about it in advance.

*** That default is a STANDING RULE, and standing rules are what run 0023/0024
cost us. *** Everything above about ForkPassed is about not letting an order
outlive its fork; a default outlives every fork by definition, because there is
no pulse that could consume it. What makes it tolerable is that it is only ever
consulted where there is a CHOICE to make: on plain tape the extreme track in
any direction is the only track, and at a crossing choose() overrides it to
STRAIGHT. The exposure is a merge - two tapes converging inside the sensor
window - where the rightmost is whichever tape is momentarily further over, and
STRAIGHT's continuity rule is not there to hold the vehicle on the one it was
already following. A track whose merges are not all right-handed will jump tape
at them, and that is a property of the layout rather than of this file.

Deliberately imports neither config nor any clock - the caller supplies
handedness, the standing default, and the compiled tag table. Same rule
plotrun.py follows: a module that cannot read the vehicle profile cannot grow a
hidden dependency on one vehicle.
"""

STRAIGHT, LEFT, RIGHT = "straight", "left", "right"
DIRECTIONS = (LEFT, RIGHT)

# A 90 deg crossing is not a diverter. The manual's own advice is to navigate
# intersections by markers rather than by #LCP, so intent is ignored there and
# the vehicle holds the main track straight through.
CROSSING_NLCP = 7

# #LCP values that mean more than one track is in view - the sensor's picture of
# a diverter. 3 is LCP1+LCP2, 6 is LCP2+LCP3, 7 is all three.
MULTI_NLCP = (3, 6, CROSSING_NLCP)
SINGLE_NLCP = 2


class Ladder:
    """The intent latch. One scan per control tick, inputs frozen by the caller.

    `default` is what intent() answers with no coil set. STRAIGHT is the drawn
    behaviour and the ladder is unchanged by any other value: a default is not
    a coil, cannot be sealed in, cannot be cleared, and does not interlock. It
    is only the answer given when the rungs have nothing to say.
    """

    def __init__(self, default=STRAIGHT):
        if default not in (STRAIGHT,) + DIRECTIONS:
            raise ValueError(f"unknown default intent {default!r}")
        self.default = default
        self.left = False
        self.right = False
        self.slow = False

    def reset(self):
        """Disarm clears intent. A latch that survived a manual intervention
        would take a junction on an order given before whatever made the
        operator stop the vehicle.

        The DEFAULT survives, because it is not intent that somebody gave - it
        is the layout of the track, and disarming does not rebuild the track.
        """
        self.left = False
        self.right = False
        self.slow = False

    def scan(self, set_left=False, clear_left=False,
             set_right=False, clear_right=False,
             set_slow=False, clear_slow=False, fork_passed=False):
        """Solve the rungs in drawn order. Returns the resulting intent.

        `slow` is left on the instance rather than returned, the same way `left`
        and `right` are: the return value is the steering answer, and a caller
        that wants the speed answer asks for it by name.

        Rung order is semantics, not layout. The second rung reads the `right`
        the first rung just wrote - that is PLC coil semantics, and it is what
        resolves a set-left and a set-right landing in the SAME scan: right
        wins. Swap the two lines and left wins instead. The case is rare enough
        never to show up in testing and to then bite once, at a junction where
        two tags sit close together, so it is asserted in the tests rather than
        left to whichever order someone types next.
        """
        self.right = (set_right or self.right) and not clear_right \
            and not fork_passed and not self.left
        self.left = (set_left or self.left) and not clear_left \
            and not fork_passed and not self.right
        # fork_passed is deliberately NOT on this rung. The steering order ends
        # at the fork; the slow zone runs to its exit tag, so the vehicle stays
        # slow across the whole junction bubble and not just up to the fork.
        # That separation is the reason these are three coils and not one.
        self.slow = (set_slow or self.slow) and not clear_slow
        return self.intent()

    def latched(self):
        """Is a coil actually sealed in? Distinct from intent(), which answers
        with the standing default when nothing is."""
        return self.left or self.right

    def intent(self):
        if self.right:
            return RIGHT
        if self.left:
            return LEFT
        return self.default


def compile_table(rows):
    """Profile branch_latch rows -> the four pulse sets the rungs read.

    Several junctions may drive the same direction, so each contact is the OR
    of every tag wired to it - the generalisation of the drawn ladder from one
    junction per coil to many. The slow contacts work the same way, and take
    only the rows that asked for it: a junction without slow_speed contributes
    no tag to either slow set, so it can neither start nor end a slow zone.
    """
    table = {"set_left": set(), "clear_left": set(),
             "set_right": set(), "clear_right": set(),
             "set_slow": set(), "clear_slow": set()}
    for row in rows or ():
        side = row["branch"]
        table[f"set_{side}"].add(row["entry_tag"])
        table[f"clear_{side}"].add(row["exit_tag"])
        if row.get("slow_speed"):
            table["set_slow"].add(row["entry_tag"])
            table["clear_slow"].add(row["exit_tag"])
    return table


class BranchEngine:
    """Ladder plus the compiled tag table. Scanned once per control tick."""

    def __init__(self, rows=(), positive_is_left=True, default=STRAIGHT):
        self.ladder = Ladder(default)
        self.table = compile_table(rows)
        self.positive_is_left = bool(positive_is_left)
        self.set_by = None          # the tag that set the live latch
        self.unmatched = 0          # tags read that no rung wires up
        self.unhonoured = False     # ordered a side that was not there
        self.forks = 0              # orders consumed by a fork, for the UI
        # Edge state for the ForkPassed contact: have we been at a diverter
        # since the order was given? Only then does a single track mean the
        # fork has been TAKEN rather than simply never reached.
        self._at_fork = False

    def reset(self):
        self.ladder.reset()
        self.set_by = None
        self.unhonoured = False
        self._at_fork = False

    def _fork_passed(self, nlcp):
        """The ForkPassed contact: has the vehicle just come out of a diverter
        with an order standing?

        Armed only while an order is live, so a diverter seen with no order -
        the merge on the way back, a crossing - can never produce a pulse and
        can never clear anything.

        *** Armed by a LATCHED coil, not by intent(). *** A standing default is
        not an order and there is nothing to consume: it has no seal-in to
        break, and clearing coils that were never set would leave the intent
        exactly where it was while logging a fork the vehicle did not take. So
        this reads the coils directly - the one place in the file where the
        difference between "sealed in" and "answered with" matters.

        #LCP 0 is "no track at all", which is a dropout rather than the far side
        of a fork, so it holds the armed state instead of resolving it. Only a
        genuine single track counts as having come out the other side.
        """
        if nlcp in MULTI_NLCP:
            if self.ladder.latched():
                self._at_fork = True
            return False
        if nlcp == SINGLE_NLCP and self._at_fork:
            self._at_fork = False
            return True
        return False

    def scan(self, tag=None, nlcp=None):
        """One scan against one tag id (or None for a tick with no read).

        Returns the intent. The caller must pass a genuine one-shot: a tag held
        live for several ticks would re-trigger its rung every tick, which for a
        set contact is harmless and for a clear contact would pin the latch off.

        nlcp is this tick's track count from the sensor, which drives the
        ForkPassed contact. Omitting it leaves the latch on the tags alone,
        which is what it did before.
        """
        was = self.ladder.intent()
        if tag is not None and not any(tag in s for s in self.table.values()):
            self.unmatched += 1
        # Evaluated BEFORE the rungs, so the pulse reflects the order that was
        # standing when the diverter was driven - not one set on this same tick.
        fork = self._fork_passed(nlcp)
        intent = self.ladder.scan(
            set_left=tag in self.table["set_left"],
            clear_left=tag in self.table["clear_left"],
            set_right=tag in self.table["set_right"],
            clear_right=tag in self.table["clear_right"],
            set_slow=tag in self.table["set_slow"],
            clear_slow=tag in self.table["clear_slow"],
            fork_passed=fork)
        # `fork` can only pulse when a coil was sealed in - see _fork_passed -
        # so it already means an order was consumed, with no second test on the
        # intent. That test would read as true forever under a standing
        # default, which is exactly the trap this pair of methods avoids.
        if fork:
            self.forks += 1
        if intent != was:
            self.set_by = tag if self.ladder.latched() else None
        return intent

    @property
    def slow(self):
        """Is a slow zone latched? Read after scan()."""
        return self.ladder.slow

    def choose(self, nlcp, tracks, last_mm=None):
        """Pick the track to follow. Returns (track_or_None, choice).

        Sets .unhonoured when a real diverter was in view and the ordered side
        was not one of the tracks in it - the vehicle carries straight on, which
        is safe, but the order was not carried out and someone should hear so.

        last_mm is the position the follower is currently tracking, and it must
        be the SAME value the follower will pass to select_track() - the two
        re-select independently and are only guaranteed to agree because they
        are handed identical inputs.
        """
        choice = STRAIGHT if nlcp == CROSSING_NLCP else self.ladder.intent()
        track = select_track(tracks, choice, self.positive_is_left, last_mm)
        self.unhonoured = bool(
            choice in DIRECTIONS and track is not None
            and track.get("index") == 2 and nlcp not in (None, 0, 2,
                                                         CROSSING_NLCP))
        return track, choice


def select_track(tracks, choice, positive_is_left=True, last_mm=None):
    """Pick a track. tracks: [{"index", "pos_mm"}, ...]. Returns the dict or None.

    LEFT/RIGHT take the extreme position on the wanted side, which needs no
    knowledge of #LCP and degrades to LCP2 when that side is absent. That is the
    deliberate jump onto a branch and it ignores `last_mm` on purpose.

    STRAIGHT follows the track it was already following - the one nearest
    `last_mm`. *** This is continuity, and it is the point. *** A physical tape
    cannot cross the sensor between two 20 ms ticks, so the nearest track to
    where we were IS the one we were on, whatever the sensor labelled it this
    scan. The old rule took LCP2, which assumes the sensor keeps the main track
    there; but LCP indices are assigned by POSITION (the manual's "LCP1 < LCP2 <
    LCP3 always applies"), so when two tapes converge and cross within the
    window the labels swap and LCP2 becomes the other tape.

    last_mm None means there is no history to be continuous with - the first
    tick of a run, or after a reset - and the old LCP2 rule is the fallback.
    """
    usable = [t for t in tracks or [] if t.get("pos_mm") is not None]
    if not usable:
        return None
    if choice == STRAIGHT:
        if last_mm is not None:
            return min(usable, key=lambda t: abs(t["pos_mm"] - last_mm))
        main = [t for t in usable if t.get("index") == 2]
        return main[0] if main else min(usable, key=lambda t: abs(t["pos_mm"]))
    want_positive = (choice == LEFT) == bool(positive_is_left)
    return (max if want_positive else min)(usable, key=lambda t: t["pos_mm"])
