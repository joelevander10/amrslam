"""The vehicle profile. Every tunable parameter in the system lives here.

Reads profiles/<name>.json and publishes it as flat module-level constants, so
call sites read `config.TRACK_M` rather than digging through nested dicts.
Everything else imports this; this imports nothing but the standard library,
which is what keeps the dependency graph acyclic.

WHICH VEHICLE
=============
The profile is chosen by the AGV_PROFILE environment variable, falling back to
DEFAULT_PROFILE:

    python3 app.py                      # profiles/agv-01.json
    AGV_PROFILE=agv-02 python3 app.py   # profiles/agv-02.json

and in the systemd unit, Environment=AGV_PROFILE=agv-01. There is no CLI flag
and no fallback to a profile that does not exist - a missing file is fatal and
lists the names that ARE present.

Four rules the loader enforces, in order of how much trouble they save:

  1. Unknown and missing keys are BOTH fatal. A profile with "k_rato" would
     otherwise leave the gain at whatever the code last defaulted to, and the
     vehicle would drive off with a number nobody chose. This is the main thing
     the JSON buys over .py constants and the main way it could bite.
  2. Only primitives are stored. Anything derivable is derived here, so the file
     cannot hold a geometry that disagrees with itself - no MPS_PER_RPM that
     contradicts the wheel diameter it was supposedly computed from.
  3. Validation runs before anything is published. A bad profile stops the
     process at import with the failing check named, rather than surfacing as
     strange behaviour halfway through a run.
  4. profile_name must match the filename. A profile copied for a second
     vehicle and not renamed would report the old identity in the event log and
     in every run CSV header, and nothing would look wrong until two runs were
     compared weeks later.
load() is written as an atomic swap - parse and validate into a fresh namespace,
publish only on success - so nothing observes a half-applied profile. Only
called once today; a reload endpoint would be a caller, not a rewrite.


TUNING NOTES
============
JSON cannot carry comments, so the reasoning behind the settings that are not
self-evident lives here. Read this before editing a profile.

vehicle.invert_left / invert_right
    Both drivers take a POSITIVE 60FFh to travel forward on this AGV. If you swap
    a motor, re-flash a driver or remount a wheel, re-verify on blocks and set
    these rather than editing kinematics - everything above it stays in vehicle terms.

drivers.ramp
    6083h/6084h, written at arm time. One mode, "auto" (2026-10-02: the "manual"
    block went with canworker - drive_node always arms with this one, and the
    manual S-curve lives in cmd_mux). Set as transparent as is safe, on the
    assumption that the motion profile is shaped in software above it. A STOP
    falls through to this decel rate.
    CAUTION: with a 400 W motor on a gearhead the Function Edition warns the motor
    can be damaged by a hard decel while demand and actual velocity differ a lot
    (opman_fun:2951). This vehicle IS that combination (BLMR6400SKM-GFV-B, 1:30):
    the drives run in motion extension (drive_forward.PVCM_MOTION_EXTENSION), and
    the auto decel should not be raised far above its accel.
    CAUTION (auto): 6083h also caps how fast the wheel DIFFERENCE can slew, so
    it is the ceiling on YAW ACCELERATION as well as on forward ramp -
    kinematics.max_yaw_accel() turns it into rad/s^2 and gives 2.59 at the
    present 2400. A controller that commands a yaw rate the drives cannot slew
    to diverges, so derive the limit from this rather than hardcoding one; and
    raising this without re-checking traction is a different kind of mistake.


timing.manual_watchdog_s
    A held button re-POSTs every ~100 ms. Miss six in a row and the setpoint is
    zeroed - covers a closed tab, a dropped Wi-Fi link and a wedged browser.

    0.6 s rather than the original 0.4: three missed POSTs is tight over Wi-Fi,
    and the trips it produced were the link hiccuping rather than the operator
    letting go. It does not change how fast the vehicle stops when they DO let
    go - the browser sends /api/stop on release and that is immediate; this
    deadline only covers the case where nothing arrives at all.

    Expiry zeroes the setpoint and logs a warning; it does NOT latch a fault.
    Recovery is to hold the button again, which is a deliberate act with a hand
    on the control, so there is nothing for an acknowledgment to add.

    *** An autonomous mode would need the opposite. *** Latched motion that
    would otherwise resume on its own has to latch a fault instead, the way the
    retired tape follower did. Whatever navigates must make that choice
    deliberately rather than inherit this one.

panel.di_manual_arm
    The pendant enable / arm input (DI09 on MCQ4-4). The drives are powered
    only while this input is HIGH, in MANUAL and AUTO alike (operator,
    2026-10-27); LOW leaves them in standby (CANopen up, encoders read, power
    stage off), so a jog, a pendant press, a survey move or an AUTO run does
    nothing until it is switched on. Under AUTO power
    also needs the run to ask for it (amr_base/arm_policy.py).

    It replaces manual_auto_arm (the selector in MANUAL was itself the arm
    command, 2026-10-08): the drives are no longer powered while idle.

panel.estop_low_scans / manual_arm_scans
    Input filters for DI08 (E-stop OK, Flexi Soft Q3) and DI09 (pendant
    enable / arm), in DIO scans (dio.scan_period_s each).

    estop_low_scans: DI08 must read LOW this many consecutive scans before
    the PC calls it an emergency (2026-11-02). With 1, a single LOW scan
    tripped it: Q3 was dropping LOW for one scan every 400 ms in bursts (83
    one-scan pulses in the logs, 30 Oct - 2 Nov; Q3 test pulses, which
    io-mapping.md says must be off), and each one powered the drives off
    and wanted a Reset. 2 rejects those at the cost of 50 ms on a real
    press. This filters only the PC's picture: STO is the Flexi Soft's, in
    hardware, and nothing here delays it. An invalid image or a comms gap
    is still an emergency at once; leaving one still needs debounce_scans
    HIGH scans. Capped at 0.2 s.

    manual_arm_scans: DI09 must read the same level this many consecutive
    scans before it is believed, either way (it was debounce_scans). A
    switch, not a pushbutton, so it can afford a little more. Capped at 0.5 s.

panel.di_continue
    PB Continue (DI0D on MCQ4-4, 2026-10-30). A pushbutton with exactly
    Start's rules: rising edge only, no edge from a button held at power-on or
    across a DIO reconnect. It releases a run WAITING at a route wait step (a
    trolley hitch / release point) and resumes a held run as Start does; Start
    also releases a wait (owner, 2026-10-30). Refused, not queued, during an
    emergency: only a fresh press after NORMAL counts.

    *** Power is not motion. *** A powered manual vehicle holds zero; every
    millimetre still needs a jog HELD (manual_watchdog_s). And nothing here can
    power through the safety chain: an FX3 demand leaves the drives in ETO, the
    enable fails, and it is retried on a slow backoff, never latched.

panel.destinations
    The destination pushbuttons (MCQ4-4, owner 2026-11-03: DI0A UBC, DI0B ITG,
    DI0C PISTON), a list of {"name", "di", "route_id"}. Momentary, with Start's
    rules: rising edge only, none from a button held at power-on or across a
    DIO reconnect, and a press counts only while it is the ONLY destination
    button down. In AUTO, with no run under way (IDLE, READY or DONE), a press
    loads the newest saved mission for `route_id` on the active map (highest
    route revision, then label set) and starts it as Start would: the start
    gate at the route's start pose, the prerequisites, then the pre-move
    warning. PB Start is not needed. Ignored in MANUAL, during a run (moving,
    held or WAITING: Continue is the button there), in FAULT and during an
    emergency. An empty route_id is a button with no route yet: a press is
    refused with that reason. Route IDs, not mission IDs, so a new route or
    map revision is picked up once it is saved as a mission.
    "di": null is a destination with no pushbutton (AMR QR, 2026-11-06: POS 1
    .. POS 6): the Run page's destination buttons start it, with the same
    rules (AUTO, no run under way, start gate, pre-move warning). Only on
    platform qr_analog: panel_node reads a DI for every destination.

gls.xy_scale_m_per_count
    What one XMCL/YMCL count is, in metres. 0 means NOT MEASURED, and it is
    the shipped value: the camera then publishes raw counts only (GlsReading
    calibrated = false) and nothing may turn a reading into a pose. Set it -
    with x_sign, y_sign, angle_sign and angle_offset_rad - from the bench tests
    in manuals/gls6-qr/gls6-system.md §7 (slide the label exactly +10 mm along
    each printed arrow, rotate it 90 deg). Codes 18 mm apart differed by ~20
    counts with the as-found template, so it is NOT plain mm.

    *** A wrongly scaled label scales every offset. *** The printed label must
    measure 75.0 mm, with 54.0 mm +-0.2 between the -27 and +27 ticks. On
    2026-10-30 the per-code values disagreed in proportion to each code's
    distance from the centre (~2.3 counts per 18 mm row): check the print
    before trusting a scale measured on it.

gls.swap_xy / x_sign / y_sign / angle_sign / angle_offset_rad
    How the device's raw X/Y/angle become T_LC (gls.to_camera_pose): swap
    first, then the signs. Bench 2026-10-30 (pendant over the HOME label, odom
    logged): vehicle fore-aft moves the device's Y, reversing lowers it, so the
    device's +Y is the vehicle's forward and swap_xy is true. The lateral sign,
    the angle sign and the offset that keeps yaw consistent with the swapped
    axes come from the sideways and spin tests (gls6-system.md §7.6).

gls.fix_frames / fix_spread_xy_m / fix_spread_yaw_rad / stationary_s
    What counts as ONE label observation, for "Record label here" and HOME at
    survey start (gls-wait-coding-plan §C): the last fix_frames readings, all
    the same label, taken after the wheels stood still for stationary_s, whose
    camera-in-label poses lie within the two spreads of their mean. 15 frames
    is ~1 s. The spreads are the plan's starting values (2 mm, 0.5 deg); tune
    them from the bench repeatability (gls6-system.md §7.4) once the scale is in.

gls.mount_x_m / mount_y_m / mount_yaw_rad
    T_BC: where the camera's optical axis meets the floor, in base_footprint.
    The lens is NOT the housing centre (op-ins Fig. 17). Placeholder zeros
    until the calibration spin (gls6-system.md §7.6). The URDF's gls_frame is
    generated from these, so TF shows whatever is here.

timing.loop_period_s / telemetry_period_s
    50 Hz bus loop; 5 Hz statusword + rpm + error register (6 fast reads).

    *** The telemetry poll is the largest spike in the loop. *** Six blocking
    SDO round-trips at ~1.8 ms each land inside a single 20 ms tick, one tick in
    ten. Measured over tape-following runs 0023-0025 the loop held a p50 of
    20.1 ms but a p95 of 28.1 and a max of 37.9. Lowering this period makes the
    spike more frequent, not smaller; moving the statusword and actual velocity
    onto drive TPDOs is what removes it - see manuals/codebase-improvement.md.

timing.driver_timeout_s
    How long a driver may go without answering a telemetry read before it is
    declared silent and the vehicle is stopped. This is HARDWARE health and has
    nothing to do with the browser watchdogs above - those cover an absent
    operator, this covers an absent driver.

    Must sit above telemetry_period_s or the ordinary gap between two polls
    reads as a dead driver; _validate() enforces that. 0.6 s is three normal
    polls, which is deliberately generous: the 5 Hz telemetry poll shares the
    bus with setpoint writes, so an occasional late read is expected. Tighten it
    only after watching the event log across several runs.

can.use_rpdo
    Send the controlword and 60FFh as ONE RPDO1 frame per tick instead of a
    blocking SDO write per node. *** Ships FALSE - turn it on only after the
    bench checklist. ***

    60FFh is currently written by a blocking SDO transfer twice a tick, about
    1.8 ms each, against a 20 ms budget the loop already overruns (20.4 ms
    average, 31 ms peak). An RPDO is a bare CAN frame with nothing to wait for,
    so this returns ~3.6 ms of every tick. It is the largest available win on
    the control loop and it is a prerequisite for the ROS drive node, not an
    optimisation - see manuals/slam-generalized-plan/hardware-reconciliation.md
    D-8 and manuals/potential-ros-migration.md section 9.

    An RPDO is not acknowledged, and that is deliberately not a loss: nothing
    ever checked the acknowledgement on the per-tick path, a failed setpoint is
    corrected 20 ms later by the next tick, and what actually detects a drive
    that has gone quiet is health.py fed by the 1017h heartbeat - untouched by
    this. A late tick, by contrast, IS a steering update the vehicle does not
    get.

    A flag rather than a straight replacement because it changes the motion
    path: if the bench finds the drives unhappy with an async RPDO, backing out
    is a profile edit rather than a revert. Both paths stay tested.

can.heartbeat_ms
    Producer heartbeat time (1017h), written to each drive at bus-up. The
    driver default is 0 = OFF, and with it off a drive that has stopped
    responding looks identical to one that is simply idle - which is exactly
    the hole health.py was working around by inferring liveness from whether an
    SDO happened to answer. 200 ms is the plan's recommendation.

    Must be under half timing.driver_timeout_s or one missed heartbeat reads as
    a dead driver; _validate() enforces that. 0 leaves the drive's own setting
    alone.

monitor.*
    Drive health monitoring - see manuals/can-monitoring-plan.txt.

    *** DIAGNOSTIC ONLY. Nothing here may decide whether it is safe to move. ***
    The safety chain is lidar/encoders -> FX3 -> HWTO1/HWTO2 -> STO, in
    hardware. CAN is the channel beside it that records what happened and warns
    before a trip.

    The analogue objects are polled ONE PER NODE per poll, round-robin, and
    poll_period_s paces it. At tick rate that would be two SDO round-trips out
    of every 20 ms budget - about 4 ms of tick and a fifth of a 125 kbps bus -
    for values that move on a thermal timescale. At 0.1 s it is ~0.8 ms of
    average tick and ~4% of the bus, sweeping the whole table in about a
    second. The plan asks for 1-5 Hz on Tier 3; this sits just under it and
    leaves the control loop alone, which matters more.

    Temperature warns are the plan's: driver trips at 85 C so warn at 70, motor
    trips at 95 so warn at 80. Bus voltage is two-sided - low is battery sag
    with stopping distance already degraded, high is regen the battery is
    refusing to take, and the high warn wants to sit well below the drive's
    own 63 V overvoltage trip.

horn.*
    The only output this software energises. One DO drives the horn AND the
    running lights together - one channel, one meaning - and it goes high
    whenever motion is COMMANDED: manual jog or auto run, no distinction,
    because the hazard is the same 150 kg either way and a horn that means two
    different things means neither.

    *** It follows the setpoint, not the wheels. *** The coil is raised from the
    commanded target, so it sounds at the instant motion is asked for rather
    than once the vehicle is already rolling - which is the whole point of a
    warning device. It also means the horn goes quiet while the drives are
    still decelerating, and that is the right way round: the warning ends when
    the vehicle stops being commanded to move, and what remains is a coast that
    is already ending.

    *** Not a safety function. *** The stop is the OSSD chain into the FX3, in
    hardware. This is an audible and visible warning and nothing gates on it: a
    horn that failed to sound must not be able to prevent a jog, or a broken
    lamp becomes a stranded vehicle.

    Renewal, not level. The command carries HORN_HOLD_S and the CAN tick renews
    it every 20 ms; a tick that stops renewing drops the coil at the DIO scan
    rather than leaving it energised. See the derived value.

blind_run.*
    Encoder-only test moves from the Blind run page (/blindrun), to measure
    the encoders before SLAM relies on them. The page only SETS a plan; PB
    Start runs it, with the selector in MANUAL and the vehicle armed. Clear, a
    selector move to AUTO, an emergency, a fault or lost 6064h feedback aborts
    it, never to resume (an emergency too, like a survey move: owner,
    2026-11-02). Hold the move again to re-run it.

    Profile velocity mode with a software position loop: the faster wheel runs
    a trapezoid on its REMAINING counts at accel_rpm_s, capped at max_rpm; the
    other follows the planned ratio plus sync_kp (1/s) times its progress lag.
    A segment ends within stop_tolerance_mm, then waits settle_s at standstill
    before its counts are recorded. Overrunning by overrun_margin, or taking
    twice the planned time, aborts. max_distance_m caps each wheel per segment.

    Nothing steers on a sensor. The IMU yaw rate is written beside the encoder
    heading in the evidence so the two can be compared afterwards - see imu.* -
    and that is the whole of its involvement.

    max_speed_mps caps the centre-path speed of a blind move (the page offers
    up to this). Above ~0.4 m/s the protective field sizing of the
    nanoScan3/FX3 must be confirmed first; that is a check on the vehicle, not
    something this file can know. Profile position (pp) was removed on
    2026-10-30 and is not part of this (2026-11-02, blind run restored).

timing.auto_start_delay_s
    The pre-move warning (2026-10-08): every move that follows an operator's
    Start under AUTO - a run start, or a resume after an E-stop - first powers
    the drives and sounds the ALARM horn for at least this long, then switches
    to the movement horn and moves (RunState.premove). A Reset, a
    selector move or a lapsed prerequisite during it cancels the start.
    Holds that resume by themselves (protective field, controller abort) keep
    their power and do not repeat the warning: nobody pressed anything.

timing.premove_timeout_s
    How long the pre-move phase waits for the drives to report Operation
    enabled before the start is refused ("drives did not power up"). Usually
    the safety chain: E-stop pressed or not yet reset. Must exceed
    auto_start_delay_s; the enable retry backoff is 2 s, so give it two tries.

imu.*
    The IMU inside the SICK MLS on can.sensor_node. drive_node reads its yaw
    rate (2034h:03, by SDO unless the sensor's yaw-rate TPDO is enabled) and
    publishes it on /imu/data_raw; imu_bias_node removes the bias and /imu/data
    feeds the EKF. Nothing gates motion on it and it is not a health source -
    see drivers/canbus/read_imu.py for why the gyro is a navigation component.

    One SDO read per poll, round-robin over eight objects (gyro xyz, accel xyz,
    yaw, the sensor's own clock). A read costs ~4 ms on the 125 kbps bus, so
    reading the whole set at once would eat a 20 ms tick; spreading it keeps
    every tick short and refreshes the whole picture every 8 x poll_period_s.
    At 0.04 s that is ~3 Hz, which is plenty to eyeball. SLAM wants 100 Hz
    yaw rate and will get it from a TPDO (read_imu.py tpdo gyro), routed
    through TpdoTap - not by turning this poll up.

    retry_period_s is the back-off after a read times out. Without it an
    absent or unpowered sensor would cost a timeout on every poll, and the
    timeout (short as it is) is dead time the control loop pays for.

can.channel / can.adapter_serial
    The SocketCAN interface to prefer, and which CANable2 to accept on the USB
    fallback. Per-vehicle hardware identity, so it belongs in the profile rather
    than in canbus/verify_drivers.py where it used to live. adapter_serial ""
    accepts any matching adapter, which is what a bench with one wants.

platform.*
    Which drive hardware this profile describes, and so which ROS base node
    owns it:
      blvr_canopen : gvievo-01. Two Oriental BLV-R drives in CiA 402 velocity
                     mode + the SICK MLS on can0 (amr_base drive_node), panel
                     and horn on the DIO island (amr_base panel_node).
      qr_analog    : the AMR QR platform. Open-loop BLDC drivers commanded by
                     an analog voltage (CKDA08ETH) and FWD/REV/BRK coils
                     (CK5162E), wheel-axle CANopen absolute encoders (CiA 406,
                     0x6004) and a WitMotion serial IMU. One node owns all of
                     it: amr_base qr_base_node. Requires the qr_base section;
                     every other profile must NOT carry one.
    For qr_analog the CAN, ramp and motor keys of the shared sections are
    re-read in its terms: can.left_node/right_node are the ENCODER nodes,
    drivers.ramp.* is the SOFTWARE acceleration ceiling the mux derives its
    limits from (the analog drivers have no ramp of their own that software
    can see), vehicle.motor_max_rpm is the fastest motor speed the voltage cap
    can hold with PI headroom, and vehicle.invert_left/right say which coil is
    "forward" for that wheel (the QR wiring has the right motor mirrored).
    Also re-read on qr_analog: imu.enabled switches the WitMotion serial IMU
    (qr_base.imu_*), not the MLS gyro; panel.di_manual_arm is unused (the QR
    panel has no Manual Arm input, so MANUAL is powered whenever the panel
    image is trusted - see amr_base.qr_base_node); lidar.host_ip is the
    fallback when nanoscan3.<profile>.yaml says host_ip "auto" and no route
    to the scanner exists at launch; panel.di_estop_ok must be qr_base.di_estop
    (one E-stop input). panel.di_continue and panel.destinations are panel_node
    channels: qr_base_node reads neither (a wait is continued by START or the
    web's Continue). gls.enabled is false while no GLS621 is fitted.

qr_base.*
    AMR QR hardware. Everything here was read off the QR controller
    (amr_controller.py, trackless_module.py, analog_digital.py) and is marked
    in the profile where it is still a placeholder.

    Analog: CKDA08ETH holding register ao_register_base + channel, in
    ao_counts_per_volt (1000 = millivolts), 0..ao_full_scale_v. Both channels
    are written in ONE write_registers call when they are adjacent, so the two
    wheels never receive setpoints from different ticks.

    *** The analog module HOLDS its last value. *** If this process dies the
    voltage stays. The FWD/REV coils are claims that expire after
    coil_hold_s (drivers/dio.DioLink), so a dead control thread drops the
    direction inputs, which stops a BLDC driver whose FWD/REV are its run
    inputs. A dead PROCESS leaves the coils latched too, exactly as the QR
    controller always did. Configure the I/O modules' communication-loss safe
    state if they have one, and keep the hard-wired E-stop chain in front of
    all of this - see the commissioning notes.

    Direction changes pass through zero: both coils low, voltage 0, and the
    readback showing both low for dir_dwell_s, before the other coil is
    raised. FWD and REV are never commanded high together.

    vehicle.track_m on qr_analog is the EFFECTIVE track - what turning in place
    actually takes - not the 0.37652 m between the wheel centres (the URDF keeps
    that). Tyre width and castor scrub make a diff-drive turn as if the wheels
    were further apart: 2026-11-05, 3 turns each way, the encoders read 13.2 %
    more than the gyro, so 0.37652 x 1.132 = 0.4263 m. With it, odometry turns
    as much as the vehicle does and a commanded spin rate is the one driven.

    Straight-line trim: wheel_scale_left / wheel_scale_right are each wheel's
    effective rolling size divided by vehicle.wheel_dia_m (1.0 = nominal). The
    encoder readings are multiplied by them, so odometry measures the ground
    each wheel really covered and the speed loops make both wheels cover the
    same ground - a vehicle that drifts to one side with both encoders reading
    the same distance drives straight. Measure them with a 5 m straight run:
    the gyro's heading change against the encoders' (2026-11-05: left 1.0073,
    right 0.9927, the vehicle drifted 11 deg right over 5 m). Mechanical causes
    (tyre size, pressure, a dragging brake) are better fixed first.

    Wheel loop: volts = ff_offset_v + |w_motor_rpm| / ff_motor_rpm_per_volt
    (feedforward) + PI on the encoder speed error, clamped to v_max_v, integral
    clamped to +/-i_max_v and frozen while saturated. Below zero_rad_s the wheel
    is commanded to rest (both coils low, 0 V, brake per brake_on_stop).
    ff_* come from the bench calibration (qr_calibrate ff; amr-qr-01: the left
    wheel's 970 r/min/V, 0.25 V). The driver holds speed on its own Hall
    sensors, so the PI is only a gentle trim (kp/ki small); a stiff loop on the
    axle encoders fought it and the vehicle surged. A zero command while driving
    first COASTS (coil kept, 0 V, brake off) for up to 0.3 s, so a steering
    correction through zero does not brake and dwell.

    Power (gvievo's arm_policy, read by qr_base_node): MANUAL is always powered
    (no Manual Arm input); AUTO is STANDBY - 0 V, wheels at rest, brakes held -
    until a run asks for power. /drives/disarm is push mode: brakes released.

    Fault latches (cleared by /drives/ack_fault): an encoder older than
    enc_timeout_s; a wheel turning the WRONG way faster than runaway_rad_s for
    runaway_s (a wrong enc_invert_* or invert_* sign shows up exactly like
    this, within a fraction of a second, which is why the first power-up is
    on blocks); a wheel commanded above 4 x zero_rad_s that does not turn for
    stall_s. The drivers report nothing, so a driver stopped from OUTSIDE the
    software (its alarm, an E-stop or scanner OSSD wired to its enable, no
    motor power) is only visible this way - and until the latch trips, the PI
    is winding up against it (integral capped at i_max_v). Wire such a stop
    signal into a spare DI if the vehicle has one.

    Encoders: CiA 406 position 0x6004; enc_range_counts is where it wraps,
    0 = read each encoder's own 6002h at start (the AMR QR's two units differ:
    the right one read 27.6 M, past the 24 bits of the EDS default),
    enc_counts_per_rev per WHEEL turn (on the axle, no gearbox). enc_mode
    auto listens for TPDO1 (event timer set to enc_event_ms at start) and
    polls by SDO while none arrive. The QR controller subtracted raw counts,
    which breaks at the 24-bit wrap (about 1.16 km of travel at 8192/rev);
    every delta here is taken modulo the range.

    IMU: WitMotion 0x55 frames, imu_gyro_sign making a CCW (left) spin positive
    (qr_calibrate imu shows it; the launch file's gyro_sign does not apply
    here). Yaw rate from the 0x52 angular-velocity packet; if the unit only
    sends 0x53 (angles), the yaw rate is differentiated from the angle and the
    diagnostics say so. 9600 baud carries ~40 Hz of 0x52+0x53 at most; the AMR
    QR's unit runs at 115200 (qr_calibrate imu-setup).

    Panel (virtual selector): the QR panel has no AUTO/MANUAL switch. START
    (panel.di_start) pressed in MANUAL switches to AUTO; pressed in AUTO it is
    the Start edge. STOP (panel.di_reset) pressed in AUTO returns to MANUAL; in
    MANUAL it is the Reset edge. With auto_hold_s > 0 only a START held that
    long enters AUTO, and a short press in MANUAL is a MANUAL Start edge (what
    the Commission page's blind run needs). E-stop (di_estop, NC when
    estop_active_low) takes the drives out of operation; it does not move the
    selector.

    route_v_mps: the speed every forward straight of a route runs at (gvievo-01
    keeps the fixed 0.60). It must fit under 95 % of the top speed that
    vehicle.motor_max_rpm gives, so the S-curve is never flattened by the
    wheel limit: 0.80 m/s needs motor_max_rpm >= 2680 (amr-qr-01 2026-11-06: 3000 =
    0.942 m/s top, 3.30 V of feedforward; v_max_v 10 V, the drivers' full scale). A straight in a route may be slower (its v_mps).
    Above the speed the 2026-11-05 feedforward calibration covered (2.0 V,
    1690 r/min) re-run `qr_calibrate ff` up to the new voltage on blocks, and
    size the scanner's protective field for the stopping distance.

    USB keypad (keypad_*, amr_base qr_keypad): the KIM2A / Kayaba wireless
    keypad on the QR PC. W/A/S/D and the arrows are a second pendant (hold to
    drive, MANUAL only, pendant speeds and S-curve; W+A etc. curve); G is the
    START button, ENTER the STOP button, Q switches MANUAL/AUTO, E latches a
    keypad E-stop that ENTER releases (software, not safety rated).
    keypad_name_contains picks the device by part of its name ("" = every USB
    keyboard - then an engineer's keyboard plugged into the PC drives the
    vehicle too); keypad_exclude_contains is a comma-separated list of names
    never used. keypad_grab keeps the keys away from the desktop and terminals.
    keypad_panel_keys false leaves only the direction keys. Needs python3-evdev
    and the service user in group input (amr.service SupplementaryGroups=input).
    `ros2 run amr_base qr_keypad --list` shows which devices would be used.

"""
import json
import math
import os
import re
import textwrap

# Anchored to the REPO ROOT, not to this file. config.py lives in agv_core/
# now, so dirname(__file__) would be agv_core/ and every profile lookup would
# miss - with no error until a vehicle boots and finds no profile at all.
# profiles/ stays at the root deliberately: it is site data, not library code.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILE_DIR = os.path.join(_ROOT, "profiles")
DEFAULT_PROFILE = "agv-01"

# Which vehicle this process is. Deliberately NOT named AGV_ID: the sibling
# KIM2A checkout uses that name, both are worked on from the same shell, and an
# export meant for one vehicle silently retargeting the other is the kind of
# mistake that only shows up as strange behaviour halfway through a run.
PROFILE_ENV_VAR = "AGV_PROFILE"


class ConfigError(Exception):
    """Raised for a malformed or physically nonsensical profile."""


def profile_path(name=None):
    """Resolve a profile name to a path. name > $AGV_PROFILE > DEFAULT_PROFILE.

    There is no fallback to a profile that does not exist: a missing file is
    fatal and says which name it looked for, in the same spirit as the key
    checks below. A silent fallback would let a typo'd AGV_PROFILE boot the
    wrong vehicle's geometry, which is exactly the failure this loader exists
    to prevent.
    """
    name = name or os.environ.get(PROFILE_ENV_VAR) or DEFAULT_PROFILE
    return os.path.join(PROFILE_DIR, f"{name}.json")


# section -> json key -> (exported name, type). The exported names match the
# constants these replaced, so the diff at every call site is one word.
_SCHEMA = {
    "vehicle": {
        "track_m":            ("TRACK_M", float),
        "wheel_dia_m":        ("WHEEL_DIA_M", float),
        "gear_ratio":         ("GEAR_RATIO", float),
        "motor_max_rpm":      ("MOTOR_MAX_RPM", float),
        "invert_left":        ("INVERT_LEFT", bool),
        "invert_right":       ("INVERT_RIGHT", bool),
    },
    "drivers": {
        # "ramp" is a nested dict; handled separately in _read_ramp().
    },
    "dio": {
        "enabled":            ("DIO_ENABLED", bool),
        "ip":                 ("DIO_IP", str),
        "port":               ("DIO_PORT", int),
        # The module answers to any unit id; this is documentation, and the
        # value the requests carry.
        "device_id":          ("DIO_DEVICE_ID", int),
        "scan_period_s":      ("DIO_SCAN_PERIOD_S", float),
        "timeout_s":          ("DIO_TIMEOUT_S", float),
        "reconnect_period_s": ("DIO_RECONNECT_PERIOD_S", float),
        "silent_warn_s":      ("DIO_SILENT_WARN_S", float),
        "di_base":            ("DIO_DI_BASE", int),
        "do_base":            ("DIO_DO_BASE", int),
        "num_di":             ("DIO_NUM_DI", int),
        "num_do":             ("DIO_NUM_DO", int),
        "di_flipped":         ("DIO_DI_FLIPPED", bool),
        # "di_names"/"do_names" are lists sized against num_di/num_do;
        # handled separately in _read_io_names().
    },
    "lidar": {
        # SICK nanoScan3. The live UDP stream is owned by the ROS driver
        # (amr_ws, sick_safetyscanners2); the controller no longer listens.
        # These keys configure the standalone bench tool drivers/lidar_scan.py,
        # which binds the port only while a human runs it. READ-ONLY: nothing
        # here opens a CoLa 2 session or writes to the device.
        "enabled":               ("LIDAR_ENABLED", bool),
        "host_ip":               ("LIDAR_HOST_IP", str),
        "sensor_ip":             ("LIDAR_SENSOR_IP", str),
        "port":                  ("LIDAR_PORT", int),
        "reassembly_timeout_s":  ("LIDAR_REASSEMBLY_TIMEOUT_S", float),
        # Provisional cut-off-path mapping. The status block's layout is not in
        # this repo, so the bytes it reads are configuration rather than code
        # until a human has walked the rings and confirmed them.
        "zone_block":            ("LIDAR_ZONE_BLOCK", int),
        "zone_active_low":       ("LIDAR_ZONE_ACTIVE_LOW", bool),
        "zones_validated":       ("LIDAR_ZONES_VALIDATED", bool),
        # "zone_bytes" is a list of ints; handled separately in _read_zone_bytes().
    },
    "panel": {
        "enabled":        ("PANEL_ENABLED", bool),
        "di_reset":       ("PANEL_DI_RESET", int),
        "di_start":       ("PANEL_DI_START", int),
        "di_auto":        ("PANEL_DI_AUTO", int),
        "auto_when_on":   ("PANEL_AUTO_WHEN_ON", bool),
        "di_manual_arm":  ("PANEL_DI_MANUAL_ARM", int),
        "di_estop_ok":    ("PANEL_DI_ESTOP_OK", int),
        "di_continue":    ("PANEL_DI_CONTINUE", int),
        "estop_reset_timeout_s": ("PANEL_ESTOP_RESET_TIMEOUT_S", float),
        "debounce_scans": ("PANEL_DEBOUNCE_SCANS", int),
        "estop_low_scans": ("PANEL_ESTOP_LOW_SCANS", int),
        "manual_arm_scans": ("PANEL_MANUAL_ARM_SCANS", int),
        "coincidence_hold_s": ("PANEL_COINCIDENCE_HOLD_S", float),
    },
    "horn": {
        "enabled":    ("HORN_ENABLED", bool),
        "do_channel": ("HORN_DO_CHANNEL", int),
    },
    "pendant": {
        # Hard-wired jog pendant on the DIO island: four direction pushbuttons,
        # levels resolved in core/panel.pendant_intent() (an opposing pair
        # cancels); the mux turns them into a body twist.
        "enabled":   ("PENDANT_ENABLED", bool),
        "di_fwd":    ("PENDANT_DI_FWD", int),
        "di_rvs":    ("PENDANT_DI_RVS", int),
        "di_left":   ("PENDANT_DI_LEFT", int),
        "di_right":  ("PENDANT_DI_RIGHT", int),
    },
    "can": {
        "bitrate":        ("CAN_BITRATE", int),
        "channel":        ("CAN_CHANNEL", str),
        "adapter_serial": ("CAN_ADAPTER_SERIAL", str),
        "heartbeat_ms":   ("CAN_HEARTBEAT_MS", int),
        "left_node":      ("LEFT", int),
        "right_node":     ("RIGHT", int),
        "sensor_node":    ("SENSOR_NODE", int),
        "use_rpdo":       ("CAN_USE_RPDO", bool),
    },
    "monitor": {
        "enabled":            ("MONITOR_ENABLED", bool),
        "poll_period_s":      ("MONITOR_PERIOD_S", float),
        "driver_temp_warn_c": ("MON_DRV_WARN_C", float),
        "driver_temp_trip_c": ("MON_DRV_TRIP_C", float),
        "motor_temp_warn_c":  ("MON_MTR_WARN_C", float),
        "motor_temp_trip_c":  ("MON_MTR_TRIP_C", float),
        "bus_v_warn_low":     ("MON_BUS_V_WARN_LOW", float),
        "bus_v_warn_high":    ("MON_BUS_V_WARN_HIGH", float),
    },
    "imu": {
        "enabled":        ("IMU_ENABLED", bool),
        "poll_period_s":  ("IMU_PERIOD_S", float),
        "retry_period_s": ("IMU_RETRY_PERIOD_S", float),
    },
    "blind_run": {
        "max_distance_m":    ("BLIND_MAX_DISTANCE_M", float),
        "max_rpm":           ("BLIND_MAX_RPM", float),
        "accel_rpm_s":       ("BLIND_ACCEL_RPM_S", float),
        "settle_s":          ("BLIND_SETTLE_S", float),
        "stop_tolerance_mm": ("BLIND_STOP_TOLERANCE_MM", float),
        "sync_kp":           ("BLIND_SYNC_KP", float),
        "overrun_margin":    ("BLIND_OVERRUN_MARGIN", float),
        "max_segments":      ("BLIND_MAX_SEGMENTS", int),
        "max_speed_mps":     ("BLIND_MAX_SPEED_MPS", float),
    },
    "timing": {
        "loop_period_s":      ("LOOP_PERIOD_S", float),
        "telemetry_period_s": ("TELEMETRY_PERIOD_S", float),
        "manual_watchdog_s":  ("MANUAL_WATCHDOG_S", float),
        "driver_timeout_s":   ("DRIVER_TIMEOUT_S", float),
        "auto_start_delay_s": ("AUTO_START_DELAY_S", float),
        "premove_timeout_s":  ("PREMOVE_TIMEOUT_S", float),
    },
    # SICK GLS621 floor-label camera (manuals/gls6-qr/gls-wait-coding-plan.md §B.3).
    "gls": {
        "enabled":              ("GLS_ENABLED", bool),
        "host":                 ("GLS_HOST", str),
        "port":                 ("GLS_PORT", int),
        "connect_timeout_s":    ("GLS_CONNECT_TIMEOUT_S", float),
        "keepalive_s":          ("GLS_KEEPALIVE_S", float),
        "verify_timeout_s":     ("GLS_VERIFY_TIMEOUT_S", float),
        "frame_gap_s":          ("GLS_FRAME_GAP_S", float),
        "stale_s":              ("GLS_STALE_S", float),
        "expect_template":      ("GLS_EXPECT_TEMPLATE", str),
        "min_codes":            ("GLS_MIN_CODES", int),
        "outlier_m":            ("GLS_OUTLIER_M", float),
        "xy_scale_m_per_count": ("GLS_XY_SCALE_M_PER_COUNT", float),
        "x_sign":               ("GLS_X_SIGN", int),
        "y_sign":               ("GLS_Y_SIGN", int),
        "angle_sign":           ("GLS_ANGLE_SIGN", int),
        "angle_offset_rad":     ("GLS_ANGLE_OFFSET_RAD", float),
        "swap_xy":              ("GLS_SWAP_XY", bool),
        "mount_x_m":            ("GLS_MOUNT_X_M", float),
        "mount_y_m":            ("GLS_MOUNT_Y_M", float),
        "mount_yaw_rad":        ("GLS_MOUNT_YAW_RAD", float),
        "fix_frames":           ("GLS_FIX_FRAMES", int),
        "fix_spread_xy_m":      ("GLS_FIX_SPREAD_XY_M", float),
        "fix_spread_yaw_rad":   ("GLS_FIX_SPREAD_YAW_RAD", float),
        "stationary_s":         ("GLS_STATIONARY_S", float),
    },
}

_TOP_LEVEL_SCALARS = {"profile_name": ("PROFILE_NAME", str), "platform": ("PLATFORM", str)}

PLATFORM_BLVR = "blvr_canopen"
PLATFORM_QR = "qr_analog"
PLATFORMS = (PLATFORM_BLVR, PLATFORM_QR)

# Present exactly when platform is qr_analog (see the platform.* note). Kept
# out of _SCHEMA so a blvr_canopen profile is not asked for keys that describe
# hardware it does not have.
_QR_SCHEMA = {
    "qr_base": {
        # analog output module (CKDA08ETH)
        "ao_ip":                 ("QR_AO_IP", str),
        "ao_port":               ("QR_AO_PORT", int),
        "ao_device_id":          ("QR_AO_DEVICE_ID", int),
        "ao_timeout_s":          ("QR_AO_TIMEOUT_S", float),
        "ao_register_base":      ("QR_AO_REGISTER_BASE", int),
        "ao_counts_per_volt":    ("QR_AO_COUNTS_PER_VOLT", float),
        "ao_full_scale_v":       ("QR_AO_FULL_SCALE_V", float),
        "ao_ch_left":            ("QR_AO_CH_LEFT", int),
        "ao_ch_right":           ("QR_AO_CH_RIGHT", int),
        # motor coils on the DIO island (CK5162E)
        "do_left_fwd":           ("QR_DO_LEFT_FWD", int),
        "do_left_rev":           ("QR_DO_LEFT_REV", int),
        "do_left_brk":           ("QR_DO_LEFT_BRK", int),
        "do_right_fwd":          ("QR_DO_RIGHT_FWD", int),
        "do_right_rev":          ("QR_DO_RIGHT_REV", int),
        "do_right_brk":          ("QR_DO_RIGHT_BRK", int),
        "brake_on_stop":         ("QR_BRAKE_ON_STOP", bool),
        "coil_hold_s":           ("QR_COIL_HOLD_S", float),
        "dir_dwell_s":           ("QR_DIR_DWELL_S", float),
        # panel
        "di_estop":              ("QR_DI_ESTOP", int),
        "estop_active_low":      ("QR_ESTOP_ACTIVE_LOW", bool),
        "auto_hold_s":           ("QR_AUTO_HOLD_S", float),
        # wheel encoders (CiA 406 on the axle)
        "enc_counts_per_rev":    ("QR_ENC_COUNTS_PER_REV", float),
        "enc_range_counts":      ("QR_ENC_RANGE_COUNTS", int),
        "enc_invert_left":       ("QR_ENC_INVERT_LEFT", bool),
        "enc_invert_right":      ("QR_ENC_INVERT_RIGHT", bool),
        "enc_mode":              ("QR_ENC_MODE", str),
        "enc_event_ms":          ("QR_ENC_EVENT_MS", int),
        "enc_timeout_s":         ("QR_ENC_TIMEOUT_S", float),
        # each wheel's effective rolling size / vehicle.wheel_dia_m (straight-line trim)
        "wheel_scale_left":      ("QR_WHEEL_SCALE_LEFT", float),
        "wheel_scale_right":     ("QR_WHEEL_SCALE_RIGHT", float),
        # WitMotion IMU
        "imu_port":              ("QR_IMU_PORT", str),
        "imu_baud":              ("QR_IMU_BAUD", int),
        "imu_timeout_s":         ("QR_IMU_TIMEOUT_S", float),
        "imu_gyro_sign":         ("QR_IMU_GYRO_SIGN", float),
        # wheel speed loop
        "ff_motor_rpm_per_volt": ("QR_FF_MOTOR_RPM_PER_VOLT", float),
        "ff_offset_v":           ("QR_FF_OFFSET_V", float),
        "v_max_v":               ("QR_V_MAX_V", float),
        "kp_v_per_rad_s":        ("QR_KP_V_PER_RAD_S", float),
        "ki_v_per_rad":          ("QR_KI_V_PER_RAD", float),
        "i_max_v":               ("QR_I_MAX_V", float),
        "zero_rad_s":            ("QR_ZERO_RAD_S", float),
        # fault latches
        "runaway_rad_s":         ("QR_RUNAWAY_RAD_S", float),
        "runaway_s":             ("QR_RUNAWAY_S", float),
        "stall_s":               ("QR_STALL_S", float),
        # route speed on straights (amr_navigation.route.route_speed)
        "route_v_mps":             ("QR_ROUTE_V_MPS", float),
        # USB keypad on the QR PC (amr_base qr_keypad)
        "keypad_enabled":          ("QR_KEYPAD_ENABLED", bool),
        "keypad_name_contains":    ("QR_KEYPAD_NAME_CONTAINS", str),
        "keypad_exclude_contains": ("QR_KEYPAD_EXCLUDE_CONTAINS", str),
        "keypad_grab":             ("QR_KEYPAD_GRAB", bool),
        "keypad_panel_keys":       ("QR_KEYPAD_PANEL_KEYS", bool),
    },
}
QR_ENC_MODES = ("auto", "tpdo", "sdo")
QR_IMU_BAUDS = (4800, 9600, 19200, 38400, 57600, 115200, 230400)


def _coerce(value, want, where):
    """JSON has one number type, so 2000 must be accepted for a float field -
    but a bool must never pass as a number, because in Python it silently would.
    """
    if want is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{where}: expected true/false, got {value!r}")
        return value
    if isinstance(value, bool):
        raise ConfigError(f"{where}: expected a number, got {value!r}")
    if want is int:
        if not isinstance(value, int):
            raise ConfigError(f"{where}: expected a whole number, got {value!r}")
        return value
    if want is float:
        if not isinstance(value, (int, float)):
            raise ConfigError(f"{where}: expected a number, got {value!r}")
        # An int too large for a float raises here; a float that already read
        # as inf (1e999 in the file) is refused below. Either way a positive-only
        # check would otherwise pass infinity - a watchdog that never expires.
        try:
            out = float(value)
        except OverflowError:
            out = math.inf
        if not math.isfinite(out):
            raise ConfigError(f"{where}: expected a finite number, "
                              f"got {value!r:.40}")
        return out
    if want is list:
        if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
            raise ConfigError(f"{where}: expected a list of strings, got {value!r}")
        return list(value)
    if want is str:
        if not isinstance(value, str):
            raise ConfigError(f"{where}: expected a string, got {value!r}")
        return value
    raise ConfigError(f"{where}: unsupported schema type {want!r}")


def _read_io_names(raw, count, where):
    """dio.di_names / do_names -> exactly `count` strings.

    _SCHEMA's flat table can say "a list of strings" but not "as many as
    num_di", and that is the check worth having: a 16-lamp page fed 12 labels
    would silently mislabel channels 12-15 or blank them, which is worse than
    no names at all.
    """
    if not isinstance(raw, list) or not all(isinstance(v, str) for v in raw):
        raise ConfigError(f"{where}: expected a list of strings")
    if len(raw) != count:
        raise ConfigError(f"{where}: has {len(raw)} name(s) but there are "
                          f"{count} channel(s) - they must match")
    return [v.strip() for v in raw]


def _read_do_list(raw, count, where):
    """dio.motion_on / dio.alarm_on -> sorted DO channel numbers, distinct, in 0..num_do-1.

    motion_on  on while the motors turn, any mode. MCQ4-4 (operator, 2026-10-27): DO00
               rotary lamp (also on while the alarm sounds) and DO01 horn.
    alarm_on   on while an AUTO run is active and the protective field is violated or
               warning field 1 or 2 is occupied, and during the pre-move warning.
               DO02 on MCQ4-4.
    movement_horn  the motion_on channels that are a movement horn: off while the alarm
               horn sounds, so a field switches the sound (2026-10-07). DO01 on MCQ4-4.
    """
    if not isinstance(raw, list) or not all(isinstance(v, int) and not isinstance(v, bool) for v in raw):
        raise ConfigError(f"{where}: expected a list of DO channel numbers")
    if len(set(raw)) != len(raw):
        raise ConfigError(f"{where}: a channel is listed twice")
    bad = [v for v in raw if not 0 <= v < count]
    if bad:
        raise ConfigError(f"{where}: channel(s) {bad} outside 0..{count - 1}")
    return sorted(raw)


def _read_zone_bytes(raw, where):
    """lidar.zone_bytes -> one byte offset per cut-off path, in path order.

    A list of ints, which _SCHEMA's flat table cannot express (its `list` means
    a list of strings, for the I/O names). Three entries, because the device's
    verification report defines paths 1-3 as stop, slow and warn - and the
    ORDER is the thing lidar_brief.md sec 2.5 insists must be confirmed rather
    than assumed, which is exactly why it lives in the profile.
    """
    if not isinstance(raw, list) or not all(isinstance(v, int)
                                            and not isinstance(v, bool)
                                            for v in raw):
        raise ConfigError(f"{where}: expected a list of integers")
    if len(raw) != 3:
        raise ConfigError(f"{where}: expected 3 offsets (stop, slow, warn), "
                          f"got {len(raw)}")
    if any(v < 0 for v in raw):
        raise ConfigError(f"{where}: byte offsets must be >= 0, got {raw}")
    return list(raw)


MAX_DESTINATIONS = 8  # PanelState.dest_edge is a 1-based index


def _read_destinations(raw, where):
    """panel.destinations -> a tuple of (name, di, route_id), in button order.

    Channel ranges and clashes with the other panel inputs are checked in
    _validate, beside the rest of the panel channels."""
    if not isinstance(raw, list):
        raise ConfigError(f"{where}: expected a list of destinations")
    if len(raw) > MAX_DESTINATIONS:
        raise ConfigError(f"{where}: at most {MAX_DESTINATIONS} destinations, got {len(raw)}")
    out = []
    for i, d in enumerate(raw):
        at = f"{where}[{i}]"
        if not isinstance(d, dict):
            raise ConfigError(f"{at}: expected an object")
        keys = {"name", "di", "route_id"}
        if set(d) != keys:
            raise ConfigError(f"{at}: needs exactly the keys {sorted(keys)}, got {sorted(d)}")
        name = _coerce(d["name"], str, f"{at}.name").strip()
        if not name:
            raise ConfigError(f"{at}.name: must not be empty")
        di = None if d["di"] is None else _coerce(d["di"], int, f"{at}.di")  # None: no button
        route_id = _coerce(d["route_id"], str, f"{at}.route_id").strip()
        # the map store's own rule for a file name (amr_navigation/store._plain)
        if route_id and (re.search(r"[/\\\0]", route_id) or route_id.startswith(".")):
            raise ConfigError(f"{at}.route_id: {route_id!r} is not a plain route id")
        out.append((name, di, route_id))
    names = [n.casefold() for n, _, _ in out]
    if len(set(names)) != len(names):
        raise ConfigError(f"{where}: two destinations share a name")
    return tuple(out)


def _read_ramp(raw):
    """drivers.ramp -> {"auto": {...}} of ints."""
    ramp = raw.get("ramp")
    if not isinstance(ramp, dict):
        raise ConfigError("drivers.ramp: missing or not an object")
    unknown = set(ramp) - {"auto"}
    if unknown:
        raise ConfigError(f"drivers.ramp: unknown mode(s) {sorted(unknown)}")
    out = {}
    for mode in ("auto",):
        block = ramp.get(mode)
        if not isinstance(block, dict):
            raise ConfigError(f"drivers.ramp.{mode}: missing or not an object")
        unknown = set(block) - {"accel", "decel"}
        if unknown:
            raise ConfigError(f"drivers.ramp.{mode}: unknown key(s) "
                              f"{sorted(unknown)}")
        missing = {"accel", "decel"} - set(block)
        if missing:
            raise ConfigError(f"drivers.ramp.{mode}: missing key(s) "
                              f"{sorted(missing)}")
        out[mode] = {k: _coerce(block[k], int, f"drivers.ramp.{mode}.{k}")
                     for k in ("accel", "decel")}
    return out


def _parse(doc):
    """Raw JSON document -> flat namespace of primitives. Strict both ways."""
    if not isinstance(doc, dict):
        raise ConfigError("profile must be a JSON object")

    # The platform decides whether qr_base belongs, so read it first - and
    # refuse an unknown one before it can make the section check below say
    # something misleading.
    platform = doc.get("platform")
    if "platform" in doc and platform not in PLATFORMS:
        raise ConfigError(f"platform: expected one of {list(PLATFORMS)}, got {platform!r}")
    schema = dict(_SCHEMA)
    if platform == PLATFORM_QR:
        schema.update(_QR_SCHEMA)

    expected_sections = set(schema) | set(_TOP_LEVEL_SCALARS)
    unknown = set(doc) - expected_sections
    if unknown:
        extra = (" (qr_base belongs only to a platform \"qr_analog\" profile)"
                 if "qr_base" in unknown else "")
        raise ConfigError(f"unknown top-level key(s): {sorted(unknown)}{extra}")
    missing = expected_sections - set(doc)
    if missing:
        raise ConfigError(f"missing top-level key(s): {sorted(missing)}")

    ns = {}
    for key, (name, want) in _TOP_LEVEL_SCALARS.items():
        ns[name] = _coerce(doc[key], want, key)

    for section, fields in schema.items():
        block = doc[section]
        if not isinstance(block, dict):
            raise ConfigError(f"{section}: expected an object")
        allowed = set(fields)
        if section == "drivers":
            allowed.add("ramp")
        if section == "dio":
            allowed |= {"di_names", "do_names", "motion_on", "alarm_on", "movement_horn"}
        if section == "lidar":
            allowed |= {"zone_bytes"}
        if section == "panel":
            allowed |= {"destinations"}
        unknown = set(block) - allowed
        if unknown:
            raise ConfigError(f"{section}: unknown key(s) {sorted(unknown)}")
        missing = allowed - set(block)
        if missing:
            raise ConfigError(f"{section}: missing key(s) {sorted(missing)}")
        for key, (name, want) in fields.items():
            ns[name] = _coerce(block[key], want, f"{section}.{key}")

    ns["RAMP"] = _read_ramp(doc["drivers"])
    # Sanity-check the counts BEFORE the name lists are sized against them, or
    # "num_di": 0 gets reported as a problem with di_names, which sends the
    # reader to fix the wrong key.
    for key, name in (("num_di", "DIO_NUM_DI"), ("num_do", "DIO_NUM_DO")):
        if not 1 <= ns[name] <= 256:
            raise ConfigError(f"dio.{key}: expected 1..256, got {ns[name]}")
    ns["DIO_DI_NAMES"] = _read_io_names(
        doc["dio"]["di_names"], ns["DIO_NUM_DI"], "dio.di_names")
    ns["DIO_DO_NAMES"] = _read_io_names(
        doc["dio"]["do_names"], ns["DIO_NUM_DO"], "dio.do_names")
    ns["DIO_MOTION_ON"] = _read_do_list(
        doc["dio"]["motion_on"], ns["DIO_NUM_DO"], "dio.motion_on")
    ns["DIO_ALARM_ON"] = _read_do_list(
        doc["dio"]["alarm_on"], ns["DIO_NUM_DO"], "dio.alarm_on")
    ns["DIO_MOVEMENT_HORN"] = _read_do_list(
        doc["dio"].get("movement_horn", []), ns["DIO_NUM_DO"], "dio.movement_horn")
    ns["LIDAR_ZONE_BYTES"] = _read_zone_bytes(
        doc["lidar"]["zone_bytes"], "lidar.zone_bytes")
    ns["PANEL_DESTINATIONS"] = _read_destinations(
        doc["panel"]["destinations"], "panel.destinations")
    return ns


def _derive(ns):
    """Everything computable from the primitives, so the profile cannot carry a
    value that contradicts the one it was derived from."""
    # Motor r/min -> wheel travel. One wheel turn covers pi*D and takes
    # GEAR_RATIO motor turns, so v = (N / GEAR_RATIO) * pi*D / 60.
    ns["MPS_PER_RPM"] = (math.pi * ns["WHEEL_DIA_M"]
                         / (ns["GEAR_RATIO"] * 60.0))          # 3.14159e-4
    ns["RPM_PER_MPS"] = 1.0 / ns["MPS_PER_RPM"]                # 3183.1
    # Yaw rate from a left/right difference: omega = (v_right - v_left) / TRACK.
    ns["RAD_S_PER_RPM_DIFF"] = ns["MPS_PER_RPM"] / ns["TRACK_M"]   # 6.46418e-4
    ns["MAX_SPEED_MPS"] = ns["MOTOR_MAX_RPM"] * ns["MPS_PER_RPM"]  # 1.2566 m/s

    # 6083h caps both the forward ramp and the rate at which the wheel
    # DIFFERENCE can slew, so it is also the ceiling on yaw acceleration -
    # kinematics.max_yaw_accel() turns it into rad/s^2. Whatever produces
    # (v, omega) next has to respect that, and must DERIVE its limit from this
    # rather than carry a hardcoded one, or a later ramp change silently
    # commands a yaw rate the drives cannot produce.
    ns["ACCEL_RPM_S"] = ns["RAMP"]["auto"]["accel"]
    ns["DECEL_RPM_S"] = ns["RAMP"]["auto"]["decel"]

    # Threshold table for canmon.MonitorPoller.snapshot(). Two-sided on bus
    # voltage: low is battery sag (stopping distance already degraded), high is
    # regen the battery is refusing to take.
    ns["MONITOR_THRESHOLDS"] = {
        "drv_c": {"warn": ns["MON_DRV_WARN_C"], "trip": ns["MON_DRV_TRIP_C"]},
        "mtr_c": {"warn": ns["MON_MTR_WARN_C"], "trip": ns["MON_MTR_TRIP_C"]},
        "bus_v": {"warn_low": ns["MON_BUS_V_WARN_LOW"],
                  "warn": ns["MON_BUS_V_WARN_HIGH"]},
    }

    ns["NODES"] = {ns["LEFT"]: "left", ns["RIGHT"]: "right"}

    # How long a coil command stays valid without being renewed. The horn is
    # commanded from the CAN tick and written by the DIO thread, so the two run
    # at different rates and the command has to survive the gap between them -
    # but only the gap. If the tick stops renewing it, the coil must fall low on
    # its own rather than sound until somebody kills the process, which is the
    # one failure a horn can have that trains operators to ignore it.
    #
    # Five ticks or two scans, whichever is longer, so neither thread's normal
    # jitter can expire a command that is still being renewed.
    ns["HORN_HOLD_S"] = max(5.0 * ns["LOOP_PERIOD_S"],
                            2.0 * ns["DIO_SCAN_PERIOD_S"])

    # How old the last SUCCESSFUL DIO acquisition may be and still count as panel
    # input (selector, Start/Reset, pendant). silent_warn_s (2 s) is the diagnostic
    # "the module has gone quiet" age; a pendant FWD held in a 2 s old image is not a
    # hand on the pendant now (audit R01, 2026-10-02). Four scans, at least 0.2 s.
    ns["PANEL_SOURCE_MAX_AGE_S"] = max(4.0 * ns["DIO_SCAN_PERIOD_S"], 0.2)
    return ns


def _validate(ns):
    def check(cond, msg):
        if not cond:
            raise ConfigError(msg)

    g = ns.__getitem__

    # -- geometry ---------------------------------------------------------
    check(g("TRACK_M") > 0, "vehicle.track_m must be > 0")
    check(g("WHEEL_DIA_M") > 0, "vehicle.wheel_dia_m must be > 0")
    check(g("GEAR_RATIO") > 0, "vehicle.gear_ratio must be > 0")
    check(g("MOTOR_MAX_RPM") > 0, "vehicle.motor_max_rpm must be > 0")


    # -- drivers ----------------------------------------------------------
    for mode, block in g("RAMP").items():
        check(block["accel"] > 0 and block["decel"] > 0,
              f"drivers.ramp.{mode} accel and decel must be > 0")

    # -- bus and timing ---------------------------------------------------
    check(g("CAN_BITRATE") > 0, "can.bitrate must be > 0")
    check(bool(g("CAN_CHANNEL")), "can.channel must be a SocketCAN interface "
                                  "name such as \"can0\"")
    ids = [g("LEFT"), g("RIGHT"), g("SENSOR_NODE")]
    check(all(1 <= n <= 127 for n in ids),
          "CAN node IDs must be in 1..127")
    check(len(set(ids)) == 3,
          f"CAN node IDs must be distinct, got left={ids[0]} right={ids[1]} "
          f"sensor={ids[2]}")
    check(g("LOOP_PERIOD_S") > 0, "timing.loop_period_s must be > 0")
    check(g("TELEMETRY_PERIOD_S") >= g("LOOP_PERIOD_S"),
          "timing.telemetry_period_s must be >= loop_period_s")
    check(g("MANUAL_WATCHDOG_S") > 0, "watchdog deadline must be > 0")
    # A driver is declared silent when its telemetry stops answering. The
    # deadline has to sit ABOVE the poll period or the normal gap between two
    # polls reads as a fault - the same pairing check as loop/telemetry above,
    # and one neither module could make alone.
    check(g("DRIVER_TIMEOUT_S") > g("TELEMETRY_PERIOD_S"),
          f"timing.driver_timeout_s ({g('DRIVER_TIMEOUT_S')}) must exceed "
          f"telemetry_period_s ({g('TELEMETRY_PERIOD_S')}), or the gap between "
          f"two normal polls is reported as a dead driver")

    # -- drive monitoring --------------------------------------------------
    # Heartbeat is what makes a dead driver distinguishable from an idle one,
    # so it has to arrive comfortably inside the window that declares the
    # driver silent, or the health monitor trips on ordinary jitter.
    check(g("CAN_HEARTBEAT_MS") >= 0, "can.heartbeat_ms must be >= 0")
    check(g("CAN_HEARTBEAT_MS") == 0
          or g("CAN_HEARTBEAT_MS") / 1000.0 < g("DRIVER_TIMEOUT_S") / 2.0,
          f"can.heartbeat_ms ({g('CAN_HEARTBEAT_MS')}) must be under half "
          f"timing.driver_timeout_s ({g('DRIVER_TIMEOUT_S')} s), or a single "
          f"missed heartbeat reads as a dead driver")
    # One object per node per poll. At the 20 ms tick this would be two SDO
    # round-trips EVERY tick - about 4 ms of a 20 ms budget and a fifth of the
    # bus - for values that move on a thermal timescale. Pacing it to 0.1 s
    # sweeps the whole table in about a second, which is inside the 1-5 Hz the
    # monitoring plan asks for and leaves the control loop alone.
    check(g("MONITOR_PERIOD_S") >= g("LOOP_PERIOD_S"),
          f"monitor.poll_period_s ({g('MONITOR_PERIOD_S')}) must be >= "
          f"loop_period_s ({g('LOOP_PERIOD_S')})")
    check(g("MON_DRV_WARN_C") < g("MON_DRV_TRIP_C"),
          "monitor.driver_temp_warn_c must be below driver_temp_trip_c")
    check(g("MON_MTR_WARN_C") < g("MON_MTR_TRIP_C"),
          "monitor.motor_temp_warn_c must be below motor_temp_trip_c")
    check(g("MON_BUS_V_WARN_LOW") < g("MON_BUS_V_WARN_HIGH"),
          f"monitor.bus_v_warn_low ({g('MON_BUS_V_WARN_LOW')}) must be below "
          f"bus_v_warn_high ({g('MON_BUS_V_WARN_HIGH')})")

    # -- dio --------------------------------------------------------------
    check(1 <= g("DIO_PORT") <= 65535, "dio.port must be in 1..65535")
    check(0 <= g("DIO_DEVICE_ID") <= 255, "dio.device_id must be in 0..255")
    # num_di/num_do ranges are checked in _parse(), before the name lists are
    # sized against them.
    check(g("DIO_DI_BASE") >= 0, "dio.di_base must be >= 0")
    check(g("DIO_DO_BASE") >= 0, "dio.do_base must be >= 0")
    check(g("DIO_SCAN_PERIOD_S") > 0, "dio.scan_period_s must be > 0")
    check(g("DIO_TIMEOUT_S") > 0, "dio.timeout_s must be > 0")
    check(g("DIO_RECONNECT_PERIOD_S") > 0, "dio.reconnect_period_s must be > 0")
    # A scan cannot outlast its own period, or the loop falls permanently
    # behind and every snapshot is older than it looks.
    check(g("DIO_TIMEOUT_S") < g("DIO_SCAN_PERIOD_S"),
          f"dio.timeout_s ({g('DIO_TIMEOUT_S')}) must be below scan_period_s "
          f"({g('DIO_SCAN_PERIOD_S')}) - two reads must fit inside one scan")
    # Otherwise a single late scan reads as a fault and the module flaps.
    check(g("DIO_SILENT_WARN_S") > g("DIO_SCAN_PERIOD_S"),
          f"dio.silent_warn_s ({g('DIO_SILENT_WARN_S')}) must exceed "
          f"scan_period_s ({g('DIO_SCAN_PERIOD_S')})")
    # The retry wait has to fit inside the health window, or a single transient
    # error is enough to declare the module dead - the reconnect sleep alone
    # would age the last good scan past silent_warn_s and stop auto.
    check(g("DIO_RECONNECT_PERIOD_S") < g("DIO_SILENT_WARN_S"),
          f"dio.reconnect_period_s ({g('DIO_RECONNECT_PERIOD_S')}) must be "
          f"below silent_warn_s ({g('DIO_SILENT_WARN_S')}) - otherwise one "
          f"retry always trips the health timeout")

    # -- lidar ------------------------------------------------------------
    check(1 <= g("LIDAR_PORT") <= 65535, "lidar.port must be in 1..65535")
    check(g("LIDAR_REASSEMBLY_TIMEOUT_S") > 0,
          "lidar.reassembly_timeout_s must be > 0")
    check(0 <= g("LIDAR_ZONE_BLOCK") < 7,
          f"lidar.zone_block ({g('LIDAR_ZONE_BLOCK')}) must be a block slot 0..6")
    check(g("LIDAR_HOST_IP") != g("LIDAR_SENSOR_IP"),
          "lidar.host_ip and lidar.sensor_ip must differ - host_ip is the "
          "address we bind, sensor_ip is the scanner we accept datagrams from")

    # -- panel ------------------------------------------------------------
    chans = {"di_reset": g("PANEL_DI_RESET"), "di_start": g("PANEL_DI_START"),
             "di_auto": g("PANEL_DI_AUTO"), "di_manual_arm": g("PANEL_DI_MANUAL_ARM"),
             "di_estop_ok": g("PANEL_DI_ESTOP_OK"), "di_continue": g("PANEL_DI_CONTINUE")}
    for key, ch in chans.items():
        check(0 <= ch < g("DIO_NUM_DI"),
              f"panel.{key} ({ch}) must be a channel in 0..{g('DIO_NUM_DI') - 1}")
    # Two functions on one channel is a wiring or config error that would
    # otherwise present as phantom presses - a Reset edge every time Start is
    # pushed - which is a miserable thing to debug from the vehicle.
    for name, ch, _route in g("PANEL_DESTINATIONS"):
        if ch is None:  # a web-only destination (no pushbutton): qr_analog only, checked there
            check(g("PLATFORM") == PLATFORM_QR,
                  f"panel.destinations {name}: di null (no pushbutton) is for platform qr_analog only")
            continue
        key = f"destination {name}"
        check(0 <= ch < g("DIO_NUM_DI"),
              f"panel.destinations {name} di ({ch}) must be a channel in 0..{g('DIO_NUM_DI') - 1}")
        check(key not in chans, f"panel.destinations: {name} is listed twice")
        chans[key] = ch
    check(len(set(chans.values())) == len(chans),
          f"panel channels must be distinct, got {chans}")
    if g("PENDANT_ENABLED"):
        jog = {g("PENDANT_DI_FWD"), g("PENDANT_DI_RVS"), g("PENDANT_DI_LEFT"), g("PENDANT_DI_RIGHT")}
        check(g("PANEL_DI_MANUAL_ARM") not in jog,
              f"panel.di_manual_arm ({g('PANEL_DI_MANUAL_ARM')}) is a pendant channel: "
              "every jog press would also power the drives")
        check(g("PANEL_DI_ESTOP_OK") not in jog,
              f"panel.di_estop_ok ({g('PANEL_DI_ESTOP_OK')}) is a pendant channel")
        check(g("PANEL_DI_CONTINUE") not in jog,
              f"panel.di_continue ({g('PANEL_DI_CONTINUE')}) is a pendant channel: "
              "every jog press would also release a waiting run")
        for name, ch, _route in g("PANEL_DESTINATIONS"):
            check(ch is None or ch not in jog,
                  f"panel.destinations {name} di ({ch}) is a pendant channel: "
                  "every jog press would also start a run")
    check(0.5 <= g("PANEL_ESTOP_RESET_TIMEOUT_S") <= 30.0,
          "panel.estop_reset_timeout_s must be in 0.5..30 s")
    check(g("PANEL_DEBOUNCE_SCANS") >= 1, "panel.debounce_scans must be >= 1")
    # The DI08 LOW filter delays the PC's emergency picture (never the Flexi Soft's STO):
    # cap it so a real press is still seen within ~0.2 s.
    check(1 <= g("PANEL_ESTOP_LOW_SCANS")
          and g("PANEL_ESTOP_LOW_SCANS") * g("DIO_SCAN_PERIOD_S") <= 0.2 + 1e-9,
          "panel.estop_low_scans must be >= 1 and at most 0.2 s of DIO scans")
    check(1 <= g("PANEL_MANUAL_ARM_SCANS")
          and g("PANEL_MANUAL_ARM_SCANS") * g("DIO_SCAN_PERIOD_S") <= 0.5 + 1e-9,
          "panel.manual_arm_scans must be >= 1 and at most 0.5 s of DIO scans")
    # A selector change and a pendant press landing in the SAME scan is not a hand: on
    # 2026-09-17 the DIO image showed MANUAL+FWD for 1.02 s mid-route with nobody at the
    # box. Such a coincidence is held back this long before it is believed (0 disables).
    check(0.0 <= g("PANEL_COINCIDENCE_HOLD_S") <= 5.0,
          "panel.coincidence_hold_s must be in 0..5 s")
    # The panel is read out of the DI image; without the scan there is nothing
    # to read, and the buttons would be silently dead.
    check(not g("PANEL_ENABLED") or g("DIO_ENABLED"),
          "panel.enabled is true while dio.enabled is false - the panel is read "
          "from the DI image, so the buttons would never respond")

    # -- motion outputs ----------------------------------------------------
    check(not (g("HORN_ENABLED") and g("HORN_DO_CHANNEL") in g("DIO_MOTION_ON") + g("DIO_ALARM_ON")),
          f"dio.motion_on/alarm_on include the horn's channel {g('HORN_DO_CHANNEL')}: one coil, two owners")
    check(set(g("DIO_MOVEMENT_HORN")) <= set(g("DIO_MOTION_ON")),
          "dio.movement_horn must be among dio.motion_on (it is a motion output that yields to the alarm)")
    check(not set(g("DIO_MOTION_ON")) & set(g("DIO_ALARM_ON")),
          "dio.motion_on and dio.alarm_on share a channel: one coil, two owners")

    # -- horn -------------------------------------------------------------
    check(0 <= g("HORN_DO_CHANNEL") < g("DIO_NUM_DO"),
          f"horn.do_channel ({g('HORN_DO_CHANNEL')}) must be a channel in "
          f"0..{g('DIO_NUM_DO') - 1}")
    # Same reasoning as panel.enabled above: the coil is written by the DIO
    # scan, so without it the horn is silently dead rather than merely off.
    check(not g("HORN_ENABLED") or g("DIO_ENABLED"),
          "horn.enabled is true while dio.enabled is false - the coil is "
          "written by the DIO scan, so the horn would never sound")

    # -- pendant ----------------------------------------------------------
    pchans = {k: g(f"PENDANT_DI_{k.upper()}") for k in ("fwd", "rvs", "left", "right")}
    for key, ch in pchans.items():
        check(0 <= ch < g("DIO_NUM_DI"),
              f"pendant.di_{key} ({ch}) must be a channel in 0..{g('DIO_NUM_DI') - 1}")
    check(len(set(pchans.values())) == 4,
          f"pendant channels must be distinct, got {pchans}")
    # A pendant button on a panel channel would drive the vehicle while also
    # pressing Start/Reset or flipping the selector.
    shared = set(pchans.values()) & set(chans.values())
    check(not shared,
          f"pendant channels {sorted(shared)} collide with panel channels {chans}")
    check(not g("PENDANT_ENABLED") or g("DIO_ENABLED"),
          "pendant.enabled is true while dio.enabled is false - the pendant is "
          "read from the DI image, so its buttons would never respond")

    # -- imu --------------------------------------------------------------
    # One SDO read per poll on the bus thread, so it is paced like the other
    # polls: never faster than the tick it runs inside.
    check(g("IMU_PERIOD_S") >= g("LOOP_PERIOD_S"),
          f"imu.poll_period_s ({g('IMU_PERIOD_S')}) must be >= loop_period_s "
          f"({g('LOOP_PERIOD_S')})")
    # The back-off exists to be longer than a poll; equal or shorter and an
    # absent sensor is retried at poll rate, which is the case it exists for.
    check(g("IMU_RETRY_PERIOD_S") > g("IMU_PERIOD_S"),
          f"imu.retry_period_s ({g('IMU_RETRY_PERIOD_S')}) must exceed "
          f"poll_period_s ({g('IMU_PERIOD_S')})")

    # -- blind run --------------------------------------------------------
    check(0 < g("BLIND_MAX_DISTANCE_M") <= 50,
          "blind_run.max_distance_m must be in (0, 50] m")
    check(0 < g("BLIND_MAX_RPM") <= g("MOTOR_MAX_RPM"),
          "blind_run.max_rpm must be in (0, motor_max_rpm]")
    check(0 < g("BLIND_ACCEL_RPM_S") <= g("RAMP")["auto"]["accel"],
          "blind_run.accel_rpm_s must be > 0 and within the drive ramp "
          "(drivers.ramp.auto.accel), or the drive lags the plan")
    check(g("BLIND_SETTLE_S") >= 0, "blind_run.settle_s must be >= 0")
    check(0 < g("BLIND_STOP_TOLERANCE_MM") <= 50,
          "blind_run.stop_tolerance_mm must be in (0, 50] mm")
    check(g("BLIND_SYNC_KP") >= 0, "blind_run.sync_kp must be >= 0")
    check(0 <= g("BLIND_OVERRUN_MARGIN") <= 0.5,
          "blind_run.overrun_margin must be a fraction in [0, 0.5]")
    check(1 <= g("BLIND_MAX_SEGMENTS") <= 32,
          "blind_run.max_segments must be in 1..32")
    check(0 < g("BLIND_MAX_SPEED_MPS") <= 0.8,
          "blind_run.max_speed_mps must be in (0, 0.8] m/s")
    check(g("BLIND_MAX_SPEED_MPS") * g("RPM_PER_MPS") <= g("BLIND_MAX_RPM") + 1e-6,
          f"blind_run.max_speed_mps ({g('BLIND_MAX_SPEED_MPS')} m/s) needs more than "
          f"blind_run.max_rpm ({g('BLIND_MAX_RPM'):g} r/min)")

    check(g("AUTO_START_DELAY_S") >= 0,
          "timing.auto_start_delay_s must be >= 0")
    check(g("AUTO_START_DELAY_S") <= 10.0,
          f"timing.auto_start_delay_s ({g('AUTO_START_DELAY_S')} s) is a long "
          f"time to stand still after a button press - an operator will press "
          f"it again")
    check(g("AUTO_START_DELAY_S") < g("PREMOVE_TIMEOUT_S") <= 30.0,
          f"timing.premove_timeout_s ({g('PREMOVE_TIMEOUT_S')} s) must exceed "
          f"auto_start_delay_s ({g('AUTO_START_DELAY_S')} s) and be at most 30 s")

    # -- gls --------------------------------------------------------------
    check(bool(g("GLS_HOST").strip()), "gls.host must not be empty")
    check(1 <= g("GLS_PORT") <= 65535, "gls.port must be in 1..65535")
    for key in ("connect_timeout_s", "keepalive_s", "verify_timeout_s", "frame_gap_s", "stale_s"):
        check(g(f"GLS_{key.upper()}") > 0, f"gls.{key} must be > 0")
    # The camera images every ~61 ms (measured 2026-10-30); a gap that long would split
    # nothing, but one far beyond it delays the last frame before every no-read.
    check(g("GLS_FRAME_GAP_S") <= 0.5, "gls.frame_gap_s must be at most 0.5 s")
    check(g("GLS_STALE_S") > g("GLS_FRAME_GAP_S"),
          f"gls.stale_s ({g('GLS_STALE_S')}) must exceed frame_gap_s ({g('GLS_FRAME_GAP_S')}): "
          "a frame is only complete after the gap")
    # Everything downstream assumes the label-CENTRE values: the as-found template output each
    # code's own position, 27 mm apart (gls6-system.md §5).
    check("[$XMCL]" in g("GLS_EXPECT_TEMPLATE") and "[$YMCL]" in g("GLS_EXPECT_TEMPLATE"),
          "gls.expect_template must output [$XMCL] and [$YMCL] (label centre, not each code)")
    check(1 <= g("GLS_MIN_CODES") <= 16, "gls.min_codes must be in 1..16 (a label has 16 codes)")
    check(g("GLS_OUTLIER_M") >= 0, "gls.outlier_m must be >= 0 (0 = off)")
    check(0 <= g("GLS_XY_SCALE_M_PER_COUNT") <= 0.01,
          "gls.xy_scale_m_per_count must be in 0..0.01 (0 = not measured yet)")
    for key in ("x_sign", "y_sign", "angle_sign"):
        check(g(f"GLS_{key.upper()}") in (1, -1), f"gls.{key} must be 1 or -1")
    for key in ("angle_offset_rad", "mount_yaw_rad"):
        check(abs(g(f"GLS_{key.upper()}")) <= math.pi, f"gls.{key} must be within +-pi")
    for key in ("mount_x_m", "mount_y_m"):
        check(abs(g(f"GLS_{key.upper()}")) <= 2.0, f"gls.{key} must be within +-2 m of base_footprint")
    # 64 = GlsLink's frame queue; at ~14.6 frames/s that is 4.4 s of standing still
    check(1 <= g("GLS_FIX_FRAMES") <= 64, "gls.fix_frames must be in 1..64")
    check(0 < g("GLS_FIX_SPREAD_XY_M") <= 0.05, "gls.fix_spread_xy_m must be in (0, 0.05] m")
    check(0 < g("GLS_FIX_SPREAD_YAW_RAD") <= 0.2, "gls.fix_spread_yaw_rad must be in (0, 0.2] rad")
    check(0 < g("GLS_STATIONARY_S") <= 10.0, "gls.stationary_s must be in (0, 10] s")

    if g("PLATFORM") == PLATFORM_QR:
        _validate_qr(ns, check)
    return ns


def _validate_qr(ns, check):
    """qr_base.* (see the platform.* and qr_base.* notes)."""
    g = ns.__getitem__

    # -- things the QR platform does not have --------------------------------
    check(not g("MONITOR_ENABLED"),
          "monitor.enabled must be false on platform qr_analog - it polls BLV-R "
          "drive objects that analog drivers do not have")
    check(g("DIO_ENABLED"),
          "dio.enabled must be true on platform qr_analog - the motor direction "
          "and brake coils are on the DIO island")
    check(g("PANEL_ENABLED"),
          "panel.enabled must be true on platform qr_analog - the panel is the "
          "only thing that grants motion")

    # -- analog output -------------------------------------------------------
    check(1 <= g("QR_AO_PORT") <= 65535, "qr_base.ao_port must be in 1..65535")
    check(0 <= g("QR_AO_DEVICE_ID") <= 255, "qr_base.ao_device_id must be in 0..255")
    check(bool(g("QR_AO_IP")), "qr_base.ao_ip must be set")
    check(g("QR_AO_IP") != g("DIO_IP"),
          "qr_base.ao_ip equals dio.ip - the analog and digital modules are two "
          "devices; one address for both is a copy-paste error")
    check(0 < g("QR_AO_TIMEOUT_S") < g("LOOP_PERIOD_S") * 5,
          f"qr_base.ao_timeout_s must be in (0, {g('LOOP_PERIOD_S') * 5:g}) s - a "
          f"write that can block longer than a few ticks stalls the wheel loop")
    check(g("QR_AO_REGISTER_BASE") >= 0, "qr_base.ao_register_base must be >= 0")
    check(g("QR_AO_COUNTS_PER_VOLT") > 0, "qr_base.ao_counts_per_volt must be > 0")
    check(0 < g("QR_AO_FULL_SCALE_V") <= 10.0,
          "qr_base.ao_full_scale_v must be in (0, 10] V")
    aoch = (g("QR_AO_CH_LEFT"), g("QR_AO_CH_RIGHT"))
    check(all(0 <= c <= 7 for c in aoch) and aoch[0] != aoch[1],
          f"qr_base.ao_ch_left/right must be two distinct channels 0..7, got {aoch}")

    # -- coils ---------------------------------------------------------------
    coils = {k: g(f"QR_DO_{k.upper()}") for k in
             ("left_fwd", "left_rev", "left_brk", "right_fwd", "right_rev", "right_brk")}
    for key, ch in coils.items():
        check(0 <= ch < g("DIO_NUM_DO"),
              f"qr_base.do_{key} ({ch}) must be a channel in 0..{g('DIO_NUM_DO') - 1}")
    check(len(set(coils.values())) == 6, f"qr_base motor coils must be distinct, got {coils}")
    check(not g("HORN_ENABLED") or g("HORN_DO_CHANNEL") not in coils.values(),
          f"horn.do_channel ({g('HORN_DO_CHANNEL')}) is also a motor coil")
    # The coil claim has to survive the gap between two DIO scans, or a live
    # control thread's direction input flickers low every other scan.
    check(2.0 * g("DIO_SCAN_PERIOD_S") <= g("QR_COIL_HOLD_S") <= 0.5,
          f"qr_base.coil_hold_s must be in [2 x dio.scan_period_s, 0.5] s, got "
          f"{g('QR_COIL_HOLD_S')}")
    # The interlock waits for a READBACK showing both coils low, and a readback
    # arrives once per scan.
    check(g("QR_DIR_DWELL_S") >= g("DIO_SCAN_PERIOD_S"),
          f"qr_base.dir_dwell_s ({g('QR_DIR_DWELL_S')}) must be >= dio.scan_period_s "
          f"({g('DIO_SCAN_PERIOD_S')})")
    check(g("QR_DIR_DWELL_S") <= 2.0, "qr_base.dir_dwell_s must be <= 2 s")

    # -- panel ---------------------------------------------------------------
    check(0 <= g("QR_DI_ESTOP") < g("DIO_NUM_DI"),
          f"qr_base.di_estop ({g('QR_DI_ESTOP')}) must be a channel in 0..{g('DIO_NUM_DI') - 1}")
    check(g("QR_DI_ESTOP") not in (g("PANEL_DI_START"), g("PANEL_DI_RESET")),
          "qr_base.di_estop collides with panel.di_start / panel.di_reset")
    # One E-stop input: the emergency picture (PanelState.estop_ok) is read from it.
    check(g("PANEL_DI_ESTOP_OK") == g("QR_DI_ESTOP"),
          f"panel.di_estop_ok ({g('PANEL_DI_ESTOP_OK')}) must be qr_base.di_estop "
          f"({g('QR_DI_ESTOP')}) on platform qr_analog - the AMR QR has one E-stop input")
    if g("PENDANT_ENABLED"):
        pch = [g(f"PENDANT_DI_{k}") for k in ("FWD", "RVS", "LEFT", "RIGHT")]
        check(g("QR_DI_ESTOP") not in pch, "qr_base.di_estop collides with a pendant channel")
    check(0.0 <= g("QR_AUTO_HOLD_S") <= 5.0, "qr_base.auto_hold_s must be in 0..5 s")
    top = 0.95 * g("MAX_SPEED_MPS")
    check(0.1 <= g("QR_ROUTE_V_MPS") <= top + 0.005,
          f"qr_base.route_v_mps ({g('QR_ROUTE_V_MPS'):g}) must be in 0.1..{top:.2f} m/s - 95 % of the "
          f"top speed vehicle.motor_max_rpm gives; raise motor_max_rpm (and calibrate) to go faster")

    # -- encoders ------------------------------------------------------------
    check(g("QR_ENC_COUNTS_PER_REV") > 0, "qr_base.enc_counts_per_rev must be > 0")
    # Unwrapping takes a delta modulo the range and trusts it only below half a
    # range; a range under two wheel turns would make that ambiguous at speed.
    check(g("QR_ENC_RANGE_COUNTS") == 0
          or g("QR_ENC_RANGE_COUNTS") >= 2 * g("QR_ENC_COUNTS_PER_REV"),
          "qr_base.enc_range_counts must be 0 (read 6002h from each encoder) or at least "
          "2 x enc_counts_per_rev")
    check(g("QR_ENC_MODE") in QR_ENC_MODES,
          f"qr_base.enc_mode must be one of {list(QR_ENC_MODES)}, got {g('QR_ENC_MODE')!r}")
    check(5 <= g("QR_ENC_EVENT_MS") <= 200, "qr_base.enc_event_ms must be in 5..200 ms")
    check(g("QR_ENC_TIMEOUT_S") >= 3 * g("QR_ENC_EVENT_MS") / 1000.0,
          f"qr_base.enc_timeout_s ({g('QR_ENC_TIMEOUT_S')}) must be >= 3 event periods "
          f"({3 * g('QR_ENC_EVENT_MS') / 1000.0:g} s), or one late frame is a fault")
    check(g("QR_ENC_TIMEOUT_S") <= 1.0, "qr_base.enc_timeout_s must be <= 1 s")
    for side in ("left", "right"):
        check(0.95 <= g(f"QR_WHEEL_SCALE_{side.upper()}") <= 1.05,
              f"qr_base.wheel_scale_{side} must be in 0.95..1.05 (a wheel more than 5 % "
              f"off the nominal size is a wrong wheel_dia_m or a flat tyre, not a trim)")

    # -- IMU -----------------------------------------------------------------
    check(bool(g("QR_IMU_PORT")), "qr_base.imu_port must be set")
    check(g("QR_IMU_BAUD") in QR_IMU_BAUDS,
          f"qr_base.imu_baud must be one of {list(QR_IMU_BAUDS)}")
    check(0 < g("QR_IMU_TIMEOUT_S") <= 2.0, "qr_base.imu_timeout_s must be in (0, 2] s")
    check(g("QR_IMU_GYRO_SIGN") in (1.0, -1.0),
          "qr_base.imu_gyro_sign must be 1 or -1 (a CCW spin must read positive)")

    # -- wheel loop ----------------------------------------------------------
    check(g("QR_FF_MOTOR_RPM_PER_VOLT") > 0, "qr_base.ff_motor_rpm_per_volt must be > 0")
    check(0 < g("QR_V_MAX_V") <= g("QR_AO_FULL_SCALE_V"),
          "qr_base.v_max_v must be in (0, ao_full_scale_v]")
    check(0 <= g("QR_FF_OFFSET_V") < g("QR_V_MAX_V"),
          "qr_base.ff_offset_v must be in [0, v_max_v)")
    # The mux never asks for more than motor_max_rpm; the feedforward for it
    # must leave the PI room under the voltage cap, or the fastest command the
    # stack can issue is one the wheel loop can only reach saturated.
    need = g("QR_FF_OFFSET_V") + g("MOTOR_MAX_RPM") / g("QR_FF_MOTOR_RPM_PER_VOLT")
    check(need <= 0.9 * g("QR_V_MAX_V") + 1e-9,
          f"vehicle.motor_max_rpm ({g('MOTOR_MAX_RPM'):g}) needs {need:.2f} V of "
          f"feedforward, more than 90 % of qr_base.v_max_v ({g('QR_V_MAX_V'):g} V) - "
          f"lower motor_max_rpm or raise v_max_v after calibration")
    check(g("QR_KP_V_PER_RAD_S") >= 0, "qr_base.kp_v_per_rad_s must be >= 0")
    check(g("QR_KI_V_PER_RAD") >= 0, "qr_base.ki_v_per_rad must be >= 0")
    check(0 <= g("QR_I_MAX_V") <= g("QR_V_MAX_V"), "qr_base.i_max_v must be in [0, v_max_v]")
    check(g("QR_ZERO_RAD_S") > 0, "qr_base.zero_rad_s must be > 0")

    # -- fault latches -------------------------------------------------------
    check(g("QR_RUNAWAY_RAD_S") > g("QR_ZERO_RAD_S"),
          "qr_base.runaway_rad_s must exceed zero_rad_s")
    check(0 < g("QR_RUNAWAY_S") <= 2.0, "qr_base.runaway_s must be in (0, 2] s")
    check(0.5 <= g("QR_STALL_S") <= 10.0, "qr_base.stall_s must be in [0.5, 10] s")


def _json_constant(name):
    """json.load's hook for the non-standard NaN / Infinity / -Infinity tokens.

    Python accepts them silently by default. They are passed through as the
    float they spell rather than refused here, because only _coerce knows which
    field it is reading: every value goes through it, and it refuses nonfinite
    numbers with the field named - "timing.manual_watchdog_s: expected a finite
    number" sends the reader to the line, "Infinity is invalid" does not."""
    return float(name)


def _check_finite(ns):
    """Every published float, primitive or derived, must be finite. Finite
    inputs can still overflow in _derive (a huge wheel diameter), and a NaN
    compares False against every bound, so this runs before _validate."""
    for name, value in ns.items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ConfigError(f"{name} is not finite ({value!r}) - check the "
                              "profile values it is derived from")
    return ns


def load(path=None):
    """Parse, derive, validate, then publish. Nothing is published on failure."""
    path = path or profile_path()
    try:
        with open(path) as fh:
            doc = json.load(fh, parse_constant=_json_constant)
    except FileNotFoundError as e:
        raise ConfigError(
            f"no vehicle profile at {path}. Set {PROFILE_ENV_VAR} to one of "
            f"{_available() or ['(none found)']}, or add the file.") from e
    except json.JSONDecodeError as e:
        raise ConfigError(f"{os.path.basename(path)} is not valid JSON: {e}") from e

    # The name inside the file has to agree with the file it came from. A
    # profile copied for a second vehicle and not renamed would otherwise
    # report the old identity in the event log and in every run CSV header -
    # silent, and only noticed when comparing runs weeks later.
    stem = os.path.splitext(os.path.basename(path))[0]
    ns = _parse(doc)
    if ns["PROFILE_NAME"] != stem:
        raise ConfigError(f"profile_name is {ns['PROFILE_NAME']!r} but the file "
                          f"is {stem}.json - rename one to match the other")

    ns = _validate(_check_finite(_derive(ns)))
    ns["PROFILE_PATH_LOADED"] = path
    globals().update(ns)
    return ns


def _available():
    """Profile names on disk, for the error message. Never raises."""
    try:
        return sorted(f[:-5] for f in os.listdir(PROFILE_DIR)
                      if f.endswith(".json"))
    except OSError:
        return []


# ---------------------------------------------------------------------------
# introspection, for the /params page
# ---------------------------------------------------------------------------
# The page is generated FROM _SCHEMA rather than from a list of its own. A
# hand-written parameters page is a second copy of the schema, and a second copy
# is one that silently stops matching - which on THIS page means a value the
# vehicle is running on that nobody can see. Add a key to a profile and it
# appears; there is nothing to remember.

# Units are read off the exported name's suffix for the same reason: the names
# already carry them, so there is nothing extra to keep in step. Longest first,
# because _MS and _MM would otherwise be eaten by _S and _M.
_UNIT_SUFFIX = (
    ("_RPM_S2", "r/min/s\u00b2"), ("_RPM_S", "r/min/s"), ("_RPM", "r/min"),
    ("_MM", "mm"), ("_MS", "ms"), ("_HZ", "Hz"),
    ("_S", "s"), ("_M", "m"), ("_C", "\u00b0C"),
)
# The ones a suffix cannot know: a bare voltage limit, a bitrate, and the
# derived conversions whose names END in something that reads as a unit but is
# not one (MPS_PER_RPM is metres per second per r/min, not r/min).
_UNIT_EXACT = {
    "CAN_BITRATE": "bit/s",
    "MON_BUS_V_WARN_LOW": "V", "MON_BUS_V_WARN_HIGH": "V",
    "MPS_PER_RPM": "m/s per r/min", "RPM_PER_MPS": "r/min per m/s",
    "RAD_S_PER_RPM_DIFF": "rad/s per r/min diff", "MAX_SPEED_MPS": "m/s",
}

# Everything _derive() computes, with the relation that produced it - which is
# the whole reason these are shown apart from the profile: they cannot be
# edited, and a page that listed them alongside the tunables would invite
# somebody to try. See _derive() for the arithmetic itself.
_DERIVED = (
    ("MPS_PER_RPM", "\u03c0\u00b7wheel_dia_m / (gear_ratio \u00b7 60)"),
    ("RPM_PER_MPS", "1 / MPS_PER_RPM"),
    ("RAD_S_PER_RPM_DIFF", "MPS_PER_RPM / track_m"),
    ("MAX_SPEED_MPS", "motor_max_rpm \u00b7 MPS_PER_RPM"),
    ("ACCEL_RPM_S", "drivers.ramp.auto.accel"),
    ("DECEL_RPM_S", "drivers.ramp.auto.decel"),
    ("HORN_HOLD_S", "max(5 \u00b7 loop_period_s, 2 \u00b7 dio.scan_period_s) - "
                    "how long a coil claim outlives its last renewal"),
)

# A tuning note heads a paragraph and starts at column 0 as section.key.
_NOTE_HEAD = re.compile(r"^[a-z_]+\.[a-z_0-9*]")


def _unit(const):
    if const in _UNIT_EXACT:
        return _UNIT_EXACT[const]
    for suffix, unit in _UNIT_SUFFIX:
        if const.endswith(suffix):
            return unit
    return ""


def _fmt(value):
    """A value as it should read on screen. Never returns an empty string - a
    blank cell reads as "not set" when the setting is genuinely an empty
    string or list, and those two mean different things."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.10g}"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, (list, tuple)):
        return ", ".join(_fmt(v) for v in value) if value else "(empty list)"
    return value if value else "(empty)"


def tuning_notes():
    """section.key -> the paragraph about it in this module's docstring.

    PARSED, not restated. The reasoning behind every setting that is not
    self-evident is already written down at the top of this file, and it is
    written there because JSON cannot carry comments. Copying it into a
    template would produce two versions of the same explanation, one of which
    would go stale - and the stale one would be the one on the screen.

    A heading is a column-0 `section.key`, optionally naming several keys
    separated by "/" and optionally `section.*` for a whole section. Anything
    that does not parse simply yields no note, so the page degrades to a plain
    table rather than failing.
    """
    doc = __doc__ or ""
    at = doc.find("TUNING NOTES")
    if at < 0:
        return {}

    out = {}
    heading = None
    body = []

    def flush():
        if not heading:
            return
        text = textwrap.dedent("\n".join(body)).strip()
        if not text:
            return
        section = heading[0].split(".")[0]
        for token in heading:
            out[token if "." in token else f"{section}.{token}"] = text

    for line in doc[at:].splitlines():
        if line and not line[0].isspace():
            if not _NOTE_HEAD.match(line):
                continue                      # prose between the notes
            flush()
            heading = [t.strip() for t in line.split("(")[0].split("/")]
            body = []
        elif heading is not None:
            body.append(line)
    flush()
    return out


# How fast the vehicle goes, gathered from the three sections that happen to
# hold it, and shown FIRST. Speed is the thing looked up most often and the
# thing most often changed, and it was previously spread across `manual`,
# `autopilot` and `drivers` - three scrolls apart on the page for one question.
#
# Display only. The JSON keeps its existing shape, so these rows carry their
# full dotted path rather than a bare key: this page's other audience is
# somebody about to edit the profile, and "auto_slow_rpm" alone would not say
# which section to put it in.
#
# (section, key) pairs, in the order they should read.
_SPEED_ROWS = [
    ("drivers", "ramp.auto.accel"),
    ("drivers", "ramp.auto.decel"),
]

# Everything in _SPEED_ROWS is suppressed where it would otherwise appear, so no
# value is ever printed twice - a page showing one parameter in two places is a
# page where the two can be read as two parameters.
_MOVED = set(_SPEED_ROWS)

# Section order on the page. Speed first, then the geometry it runs on;
# everything after is in schema order. Named here rather
# than by reordering _SCHEMA, because _SCHEMA's order is the profile's order and
# those are two different jobs.
_SECTION_ORDER = ["vehicle"]



def _row(section, key, const, value, notes, text=None, unit=None):
    """One displayed parameter. `key` is what to edit in the JSON, `const` is
    what the code calls it - both, because the two audiences for this page are
    somebody editing a profile and somebody reading a traceback."""
    # A note may be attached to the nested key it heads (drivers.ramp covers
    # ramp.auto.accel), so fall back to the first segment before giving up.
    note = (notes.get(f"{section}.{key}")
            or notes.get(f"{section}.{key.split('.')[0]}")
            or notes.get(f"{section}.*"))
    return {"key": key, "const": const,
            "value": _fmt(value) if text is None else text,
            "unit": _unit(const) if unit is None else unit, "note": note}


def describe():
    """The loaded profile as ordered sections of displayable rows.

    Read-only by construction: this returns text, and there is no counterpart
    that writes. A profile is changed by editing the JSON and restarting the
    service, which is what makes the value on the screen the value the bus
    thread is actually using.
    """
    g = globals()
    notes = tuning_notes()
    out = []

    def ramp_row(section, key):
        """A drivers.ramp.<mode>.<accel|decel> row. Nested, so _SCHEMA's flat
        table cannot express it and both call sites build it here."""
        _, mode, k = key.split(".")
        return _row(section, key, f'RAMP["{mode}"]["{k}"]',
                    g["RAMP"][mode][k], notes, unit="r/min/s")

    def build(section, key):
        if key.startswith("ramp."):
            return ramp_row(section, key)
        const, _want = _SCHEMA[section][key]
        return _row(section, key, const, g.get(const), notes)

    # Speed first. Gathered from three sections, so each row is labelled with
    # the path it actually lives at - see _SPEED_ROWS.
    speed = []
    for section, key in _SPEED_ROWS:
        row = build(section, key)
        row["key"] = f"{section}.{key}"
        speed.append(row)
    out.append({
        "name": "speed", "rows": speed,
        "note": "How fast the vehicle goes, in one place. These keys live in "
                "the manual and drivers sections of the JSON - each row is "
                "labelled with the path to edit. The manual pair is the jog "
                "pad; the ramp pairs are 6083h/6084h, written to the drives at "
                "arm time, and the auto pair also sets the ceiling on yaw "
                "acceleration - see kinematics.max_yaw_accel()."})

    # Then the schema's own sections: the named ones in the order above, then
    # whatever is left in the order the profile writes it.
    out.append({"name": "platform", "note": notes.get("platform.*"), "rows": [
        _row("platform", "platform", "PLATFORM", g.get("PLATFORM"), notes, unit="")]})
    schema = dict(_SCHEMA)
    if g.get("PLATFORM") == PLATFORM_QR:
        schema.update(_QR_SCHEMA)
    ordered = _SECTION_ORDER + [s for s in schema if s not in _SECTION_ORDER]
    for section in ordered:
        fields = schema[section]
        rows = [_row(section, key, const, g.get(const), notes)
                for key, (const, _want) in fields.items()
                if (section, key) not in _MOVED]

        # The three things _SCHEMA's flat table cannot express, in the same
        # order the profile writes them.
        if section == "drivers":
            for mode in ("auto",):
                for k in ("accel", "decel"):
                    key = f"ramp.{mode}.{k}"
                    if (section, key) not in _MOVED:
                        rows.append(ramp_row(section, key))
        if section == "dio":
            for key, const in (("di_names", "DIO_DI_NAMES"),
                               ("do_names", "DIO_DO_NAMES")):
                names = g[const]
                named = [f"{i:02d} {n}" for i, n in enumerate(names) if n]
                rows.append(_row(
                    section, key, const, names, notes,
                    text=(f"{len(named)} of {len(names)} named \u00b7 "
                          + " \u00b7 ".join(named)) if named
                         else f"none of {len(names)} named",
                    unit=""))
            on = g["DIO_MOTION_ON"]
            rows.append(_row(section, "motion_on", "DIO_MOTION_ON", on, notes,
                             text=(" \u00b7 ".join(f"DO{c:02d}" for c in on) + " on while the motors turn") if on else "none",
                             unit=""))
            al = g["DIO_ALARM_ON"]
            rows.append(_row(section, "alarm_on", "DIO_ALARM_ON", al, notes,
                             text=(" \u00b7 ".join(f"DO{c:02d}" for c in al) + " on in an AUTO run with a field occupied") if al else "none",
                             unit=""))
            mh = g["DIO_MOVEMENT_HORN"]
            rows.append(_row(section, "movement_horn", "DIO_MOVEMENT_HORN", mh, notes,
                             text=(" \u00b7 ".join(f"DO{c:02d}" for c in mh) + " off while the alarm horn sounds") if mh else "none",
                             unit=""))
        if section == "panel":
            dests = g["PANEL_DESTINATIONS"]
            rows.append(_row(section, "destinations", "PANEL_DESTINATIONS", dests, notes,
                             text=" · ".join(f"{'web' if ch is None else f'DI{ch:02d}'} {n} → {r or '(no route)'}"
                                                  for n, ch, r in dests) or "none",
                             unit=""))
        if section == "lidar":
            rows.append(_row(section, "zone_bytes", "LIDAR_ZONE_BYTES",
                             g["LIDAR_ZONE_BYTES"], notes,
                             text="stop {}, slow {}, warn {}".format(
                                 *g["LIDAR_ZONE_BYTES"]),
                             unit="byte offset"))
        # A section whose every key was gathered into `speed` above has nothing
        # left to show - `manual` is exactly that - and a bare heading over
        # nothing reads as a section that failed to load.
        if rows:
            out.append({"name": section, "note": notes.get(f"{section}.*"),
                        "rows": rows})

    # `from` rather than a note: the relation is the point of the row, so it is
    # always on screen, and there is no JSON key to edit because there is no
    # JSON key at all.
    out.append({"name": "derived", "note": None, "rows": [
        {"key": const, "const": "", "from": why,
         "value": _fmt(g[const]),
         "unit": _unit(const), "note": None}
        for const, why in _DERIVED]})
    return out


load()
