"""The alarm horn, dio.alarm_on (operator, 2026-10-02): AUTO RUNNING and (protective stop OR
warning 1 OR warning 2), plus the pre-move warning. Running = RunState EXECUTING/BLOCKED.

MCQ4-4 outputs (operator, 2026-10-27): DO00 rotary lamp, DO01 horn (the movement horn),
DO02 alarm horn."""

from amr_base.panel_io import alarm_wanted, motion_outputs

IDLE, EXECUTING, PAUSED, BLOCKED, FAULT, DONE = 0, 2, 3, 4, 5, 6  # RunState
LAMP, HORN = 0, 1
MOTION, MOVEMENT_HORN = [LAMP, HORN], [HORN]


def test_auto_run_with_a_field_occupied_sounds_it():
    for run in (EXECUTING, BLOCKED):
        assert alarm_wanted(True, run, True, True, True), run  # warning 1 or 2
        assert alarm_wanted(True, run, True, False, True), run  # protective stop


def test_no_field_no_alarm():
    assert not alarm_wanted(True, EXECUTING, True, True, False)


def test_not_running_or_not_auto_is_silent():
    for run in (IDLE, PAUSED, FAULT, DONE, None):
        assert not alarm_wanted(True, run, True, False, True), run
    assert not alarm_wanted(False, EXECUTING, True, False, True), "selector MANUAL"


def test_the_pre_move_warning_is_lamp_and_alarm_horn_then_lamp_and_horn():
    """2026-10-08: a Start under AUTO sounds the alarm horn before the vehicle moves - while
    RunState.premove - with no field data at all. 2026-10-27: the lamp turns with it, and
    once the wheels turn the sound goes back to the horn."""
    assert alarm_wanted(True, EXECUTING, False, True, False, run_premove=True)
    assert alarm_wanted(True, None, False, True, False, run_premove=True)
    assert not alarm_wanted(True, EXECUTING, True, True, False, run_premove=False)
    assert not alarm_wanted(False, EXECUTING, True, True, False, run_premove=True), "selector MANUAL"
    # pre-move: nothing turns yet, the alarm sounds -> lamp on, horn off
    assert motion_outputs(False, True, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: False}
    # moving, no field: lamp and horn
    moving = alarm_wanted(True, EXECUTING, True, True, False)
    assert motion_outputs(True, moving, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: True}


def test_manual_moving_is_lamp_and_horn():
    alarm = alarm_wanted(False, None, True, True, True)
    assert not alarm
    assert motion_outputs(True, alarm, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: True}


def test_stale_field_data_does_not_sound_it():
    assert not alarm_wanted(True, EXECUTING, False, False, True)


def test_a_field_switches_the_horn_to_the_alarm_horn():
    """Operator, 2026-10-07: on warning 1, warning 2 or protective the sound changes - the
    horn off while the alarm horn sounds; the lamp stays on."""
    for run, protective_clear in ((EXECUTING, True), (BLOCKED, True), (EXECUTING, False)):
        alarm = alarm_wanted(True, run, True, protective_clear, True)
        assert alarm
        assert motion_outputs(True, alarm, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: False}
    assert motion_outputs(True, False, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: True}, "no field"
    assert motion_outputs(False, False, MOTION, MOVEMENT_HORN) == {LAMP: False, HORN: False}, "stopped"
    assert motion_outputs(False, True, MOTION, MOVEMENT_HORN) == {LAMP: True, HORN: False}, \
        "stopped by protective: lamp and alarm horn"


def test_every_coil_is_low_while_not_normal():
    """Owner, 2026-10-30: no lamp, no horn, no alarm horn in an emergency (DO00/DO01/DO02 and
    the horn coil); outside it the motion/alarm rules are unchanged."""
    from amr_base.panel_io import coil_image

    ALARM, H = 2, 5
    kw = dict(alarm_on=[ALARM], motion_on=MOTION, movement_horn=MOVEMENT_HORN, horn_ch=H)
    assert coil_image(True, True, True, True, **kw) == {ALARM: True, LAMP: True, HORN: False, H: True}
    assert coil_image(True, False, True, False, **kw) == {ALARM: False, LAMP: True, HORN: True, H: False}
    for alarm in (False, True):
        for turning in (False, True):
            outs = coil_image(False, alarm, turning, True, **kw)
            assert outs == {ALARM: False, LAMP: False, HORN: False, H: False}
