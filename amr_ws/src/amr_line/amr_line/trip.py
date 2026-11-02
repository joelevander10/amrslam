"""Calls -> trips (site-layout-103 section 3, tracked plan 3 sections 2.4-2.5). Pure.

    CallQueue   machines call from the display. A trip forms when trip_calls calls of ONE
                group are pending (site D3/D4); the other group's calls keep waiting. A
                machine already pending is refused, as is one not in the mission.
    Trip        one group, its machines, and the circuit's stops in CIRCUIT order (not call
                order). Facing machines share a stop: one stop serves both (D6). MRU
                machines are stopped at twice: release westbound, take back eastbound.
    Payload     the guard for both U-turns (D9). No sensor sees the trolley (D10), so
                "released" is the record of a served release stop, and "off at home" is
                the operator's confirmation on the Calls page.

The trip never moves anything. It answers "stop here?", "may it pivot?", "may it leave
the bay?"; the line layer turns a "no" into a hold.
"""

SWAP, RELEASE, TAKE = "swap", "release", "take"


class CallQueue:
    def __init__(self, mission):
        self.m = mission
        self.pending = []        # [(machine, t)] in call order

    def call(self, machine, t):
        """(ok, message)."""
        group = self.m["MACHINES"].get(machine)
        if group is None:
            return False, f"{machine!r} is not a machine of {self.m['MISSION_NAME']}"
        if any(m == machine for m, _ in self.pending):
            return False, f"{machine} is already waiting"
        self.pending.append((machine, float(t)))
        n = sum(1 for m, _ in self.pending if self.m["MACHINES"][m] == group)
        need = self.m["TRIP_CALLS"]
        return True, f"{machine} called ({group}: {n} of {need})"

    def cancel(self, machine):
        before = len(self.pending)
        self.pending = [(m, t) for m, t in self.pending if m != machine]
        return len(self.pending) != before

    def ready_group(self, available=None):
        """The group whose trip is formed: trip_calls pending, earliest first call wins.
        `available` limits it to groups whose circuit is taught."""
        need = self.m["TRIP_CALLS"]
        counts, first = {}, {}
        for m, t in self.pending:
            g = self.m["MACHINES"][m]
            if available is not None and g not in available:
                continue
            counts[g] = counts.get(g, 0) + 1
            first.setdefault(g, t)
        ready = [g for g, n in counts.items() if n >= need]
        return min(ready, key=lambda g: first[g]) if ready else None

    def take(self, available=None):
        """Form the trip and remove its calls from the queue, or None."""
        g = self.ready_group(available)
        if g is None:
            return None
        need = self.m["TRIP_CALLS"]
        mine = [m for m, _ in self.pending if self.m["MACHINES"][m] == g][:need]
        self.pending = [(m, t) for m, t in self.pending if m not in mine]
        return Trip(self.m, g, mine)


class Trip:
    def __init__(self, mission, group, machines):
        self.m = mission
        self.group = group
        self.machines = list(machines)
        g = mission["GROUPS"][group]
        self.circuit = g["circuit"]
        items = mission["CIRCUITS"][self.circuit]["items"]
        wanted = {}
        for mc in self.machines:
            for kind, sid in g["machines"][mc].items():
                wanted.setdefault(sid, []).append((mc, kind))
        # Circuit order. HOME (first and last) and ATTACH are stops of every trip.
        self.plan = []
        for k, it in enumerate(items):
            if it["kind"] != "stop":
                continue
            sid = it["id"]
            if sid == mission["ATTACH"]:
                self.plan.append({"id": sid, "role": "attach", "work": []})
            elif sid == mission["HOME"] and k == len(items) - 1:
                self.plan.append({"id": sid, "role": "home", "work": []})
            elif sid in wanted:
                self.plan.append({"id": sid, "role": "machine", "work": wanted[sid]})
        self.served = set()
        self.uturn_at = next((k for k, it in enumerate(items) if it["kind"] == "uturn"), None)

    def stops(self):
        """Stop ids in order: where this trip stops (ATT, machines, HOME)."""
        return [p["id"] for p in self.plan]

    def stops_at(self, sid):
        return any(p["id"] == sid for p in self.plan)

    def work(self, sid):
        return next((p["work"] for p in self.plan if p["id"] == sid), [])

    def serve(self, sid):
        """The operator pressed Start at this stop: its work is recorded as done."""
        if self.stops_at(sid):
            self.served.add(sid)

    def confirm_released(self):
        """The operator confirms on the Calls page that every trolley of this trip is off
        (the payload HOLD before the U-turn): the release stops count as served."""
        g = self.m["GROUPS"][self.group]["machines"]
        for mc in self.machines:
            if RELEASE in g[mc]:
                self.served.add(g[mc][RELEASE])

    def releases_pending(self):
        """Machines whose release stop has not been served."""
        g = self.m["GROUPS"][self.group]["machines"]
        return [mc for mc in self.machines if RELEASE in g[mc] and g[mc][RELEASE] not in self.served]

    def may_pivot(self):
        """(ok, why) for the U-turn: every release done, so no trolley is attached."""
        left = self.releases_pending()
        if left:
            return False, f"release not recorded for {', '.join(left)}: a trolley may still be attached"
        return True, ""

    def missed(self):
        """Machine stops passed without being served (reported at HOME)."""
        out = []
        for p in self.plan:
            if p["role"] == "machine" and p["id"] not in self.served:
                out += [f"{mc} ({kind} at {p['id']})" for mc, kind in p["work"]]
        return out

    def result(self):
        miss = self.missed()
        done = ", ".join(self.machines)
        return f"{self.group} trip {done}: " + (f"MISSED {'; '.join(miss)}" if miss else "served")


class Payload:
    """Has the last trip's trolley come off at HOME? Survives trips; starts unknown.

    Unknown at boot counts as attached: the operator confirms once at the bay. A trip
    may leave the bay only empty (the HOME U-turn R717 is driven empty, D9/D11)."""

    def __init__(self):
        self.off_at_home = False
        self.why = "not confirmed since start-up"

    def trip_ended(self):
        self.off_at_home = False
        self.why = "trip ended at HOME: confirm the trolleys are off"

    def confirm_off(self):
        self.off_at_home = True
        self.why = ""

    def may_leave(self):
        return (True, "") if self.off_at_home else (False, self.why)
