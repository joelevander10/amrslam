"""AMR manual, English. Blocks are rendered by build.py."""

META = {
    "lang": "en",
    "title": "AMR New Gen (AMR QR-01)",
    "subtitle": "Operation, Maintenance and Troubleshooting Manual",
    "edition": "Edition 1.0 · November 2026 · software: AMR SLAM New Gen (profile amr-qr-01)",
    "fill_title": "Fill in for this vehicle",
    "fill": [("Web page address", "http://______________________:5001"), ("Wi-Fi network", ""),
             ("Engineer / contact", ""), ("Phone", ""), ("Map in use (map id, revision)", "")],
    "contents": "Contents",
    "part": "Part",
    "page": "page",
    "footer": "AMR New Gen - Operation, Maintenance and Troubleshooting",
    "chapter": "Chapter",
    "who_title": "Who reads what",
    "who": [("Part 1 - Operating", "Everyone who works with the AMR"), ("Part 2 - Maintenance and troubleshooting", "Technician, shift leader"),
            ("Part 3 - Engineer tasks", "Engineer (needs the engineer PIN)")],
}

B = []
A = B.append

# ---------------------------------------------------------------- intro
A(("h1", "Safety first", False))
A(("p", "The AMR (autonomous mobile robot) drives by itself along routes taught on a map. It sees its surroundings with a "
        "safety laser scanner at the front and slows down or stops for people and objects. It is still a heavy machine. "
        "Read this page before you work with it."))
A(("warn", "<b>The red E-stop button is the real stop.</b> Know where it is before the AMR moves. Pressing it cuts the motors "
           "and holds the brakes at once."))
A(("bul", [
    "Stay out of the path of a moving AMR, and never ride on it.",
    "The scanner only looks <b>forward</b>. When the AMR reverses (short reverse steps in some routes) it does not see behind it.",
    "At 0.8 m/s the AMR needs about <b>0.7 m to stop</b>. Do not step in front of it at the last moment.",
    "A stopped AMR may <b>start again by itself</b>: after something leaves the scanner field, after it found its position again, "
    "or after a 2-second warning when someone presses START. The AMR has no horn - watch it.",
    "Before you put your hands near the wheels, press the E-stop.",
    "<b>Push mode</b> (brakes released) only on a level floor: the AMR rolls freely.",
    "Do not cut the main power while the AMR is driving (see Chapter 8, End of shift).",
    "Only trained people change maps, routes or settings (Part 3).",
]))
A(("h2", "Words used in this book"))
A(("table", ["Word", "Meaning"], [
    ["Map", "The AMR's drawing of the floor (walls, machines, racks), made by driving it around once. Each change makes a new <b>revision</b> (rev 1, rev 2, ...)."],
    ["Route", "A path drawn on the map: straights, curves, turns, wait points."],
    ["Mission", "A route ready to run on one map revision. The operator loads and runs missions."],
    ["Homepost", "The start point of a route, marked on the floor. The AMR starts and ends there."],
    ["POS 1-6", "The six stop points (stations). Each POS route goes from the homepost to its stop point and back."],
    ["Localisation", "The AMR knowing where it is on the map. It must read <b>READY</b> before a mission can run."],
    ["MANUAL / AUTO", "MANUAL: you drive it by hand. AUTO: the mission drives it."],
    ["E-stop", "Emergency stop button (red)."],
], [0.22, 0.78]))

# ---------------------------------------------------------------- PART 1
A(("part", "1", "Operating", "For everyone who works with the AMR"))

A(("h1", "Get to know the AMR", True))
A(("fig", "overview", "The main parts (schematic top view; positions of buttons and casters are only illustrative).", 0.95))
A(("table", ["", ""], [
    ["Size", "1.18 m long × 0.675 m wide"],
    ["Drive", "Two driven wheels (left and right) with encoders; casters carry the rest"],
    ["Speed in a mission", "0.80 m/s on straights, 0.30 m/s in curves (each curve can have its own speed), slower in slow zones"],
    ["Speed by hand", "0.10-0.40 m/s from the web page, 0.50 m/s from the USB keypad"],
    ["Sensors", "SICK nanoScan3 safety laser scanner (front), wheel encoders, gyroscope (IMU)"],
    ["Control", "Industrial PC; web page on a tablet, phone or laptop over Wi-Fi; START / STOP buttons; E-stop; USB keypad"],
    ["Navigation", "Laser map (SLAM). No tape or QR codes on the floor are needed"],
], [0.28, 0.72], False))

A(("h1", "Buttons, keypad and the web page", True))
A(("fig", "panel", "Panel buttons and the USB keypad.", 1.0))
A(("h2", "The panel buttons"))
A(("table", ["Button", "In MANUAL", "In AUTO"], [
    ["<b>START</b> (green)", "Switches to AUTO. Nothing moves. The last mission loads by itself.", "Starts the loaded mission, or continues a stopped one."],
    ["<b>STOP</b>", "Acknowledges (resets) a mission fault.", "Back to MANUAL. A running mission is aborted."],
    ["<b>E-stop</b> (red)", "Motors off, brakes on.", "Motors off, brakes on. The mission is held (BLOCKED). Release the E-stop, then press START to continue."],
], [0.18, 0.37, 0.45]))
A(("note", "<b>After an E-stop there is no reset button on this AMR.</b> Releasing (twisting) the E-stop is enough. Some screen texts "
           "say \"press the blue Reset button on the cabinet\" - that text belongs to another vehicle; ignore it. In a mission, press "
           "START to continue."))
A(("bul", [
    "After switching on, the AMR is always in <b>MANUAL</b>.",
    "A button already held down at power-up does nothing until you release it and press it again.",
    "The web page never starts a mission by itself. Movement needs START on the panel/keypad, a POS button, or a held jog key.",
]))
A(("h2", "The USB keypad"))
A(("p", "The wireless keypad works like the panel. Its dongle is plugged into the AMR's PC and is found by itself."))
A(("table", ["Key", "Does"], [
    ["W / ↑, S / ↓", "Forward / reverse while held (MANUAL only)"],
    ["A / ←, D / →", "Turn on the spot left / right while held (MANUAL only). W+A, W+D ... drive a curve"],
    ["G", "= START button"],
    ["ENTER", "= STOP button"],
    ["Q", "Switch MANUAL ⇄ AUTO"],
    ["E", "Keypad stop (latched). ENTER releases it. <b>This is a software stop, not the safety E-stop.</b>"],
    ["F1 ... F6", "Go to POS 1 ... POS 6 (in AUTO, AMR on the homepost, no mission running)"],
], [0.2, 0.8]))
A(("h2", "The web page"))
A(("p", "Open the address written on the cover (port 5001) in the browser of a tablet, phone or laptop on the AMR's Wi-Fi. "
        "The operator view has four pages:"))
A(("table", ["Page", "What it is for"], [
    ["<b>Home</b>", "One line telling you the state of the AMR, the speed, and one sentence of what to do. Pause, Abort, Return to idle, Recover."],
    ["<b>Run</b>", "Choose the map, set the AMR's position, load missions, the POS buttons, Continue, Pause, Abort, Ack fault. The map follows the AMR."],
    ["<b>Manual</b>", "Drive by hand with the jog pad; push mode (brakes off / on)."],
    ["<b>Alarms</b>", "What stops the vehicle now (\"Standing now\"), what happened, localisation losses, Save report."],
], [0.18, 0.82]))
A(("fig", "webpage", "Layout of the Run page.", 1.0))
A(("h2", "The status tiles (left side of every page)"))
A(("table", ["Tile", "Normal", "Meaning / if not normal"], [
    ["Mode", "IDLE (no map), NAVIGATION (map in use), MAPPING", "FAULT or a red banner: see the Alarms page"],
    ["Selector", "MANUAL or AUTO", "STALE / INVALID: the panel I/O module is not answering (Chapter 10)"],
    ["Drives", "ARMED in MANUAL and while a mission drives; OFF in AUTO without a mission (standby, brakes held)",
     "OFF \"push mode\": brakes released - Manual page, Brakes ON. Red: E-stop or a drive fault"],
    ["Command", "none, or who drives (manual, follow ...)", ""],
    ["Scanner", "CLEAR", "WARNING (half speed) or PROTECT (stopped): something in front"],
    ["Localis.", "READY", "CHECKING (waiting for Confirm), LOST (position lost)"],
    ["Run", "READY, EXECUTING, DONE", "BLOCKED, WAITING, FAULT: see Chapter 7"],
    ["Wheels", "Commanded speed left / right", ""],
], [0.14, 0.4, 0.46]))
A(("fig", "modes", "Vehicle mode (left) and mission states (right).", 1.0))

A(("h1", "Start of shift", True))
A(("fig", "startflow", "From switching on to the first mission.", 1.0))
A(("steps", [
    "Look around the AMR: nothing on top that can fall, nothing under the wheels, the scanner window clean, the area clear.",
    "Release the E-stop. Switch the AMR on. Wait about one minute for the software to start.",
    "Open the web page. <b>Home</b> should show: Mode <b>IDLE</b> or NAVIGATION, no red banner, Selector <b>MANUAL</b>, Drives <b>ARMED</b>.",
    "Do not touch or push the AMR for a few seconds after it starts: the gyroscope calibrates itself while it stands still.",
    "If the Home page says <b>\"The vehicle lost power last time\"</b> (UNCLEAN_SHUTDOWN): switch the drive power off and on once "
    "(ask the technician where the switch is) before the first mission.",
    "Continue with Chapter 5 (position) and Chapter 6 (run a mission).",
]))
A(("tip", "Daily 1-minute check: E-stop stops the motors; START switches to AUTO and STOP back to MANUAL; the scanner LED is green with "
          "nothing in front. Write problems in the logbook."))

A(("h1", "Driving by hand", True))
A(("bul", [
    "The AMR must be in <b>MANUAL</b> (press STOP if the Selector tile shows AUTO).",
    "<b>Web page</b> (Manual, Run or Maps page): hold a jog pad button or the W/A/S/D / arrow keys. Releasing stops the AMR. "
    "Choose 0.10-0.40 m/s. Switching browser tab also stops it.",
    "<b>USB keypad</b>: hold W/A/S/D or the arrows; 0.50 m/s. Release every key to stop.",
    "The scanner fields do not slow a hand drive, but the scanner's safety output still stops the motors. If you keep holding a key "
    "while it is stopped, the wheels report \"stalled\" after 2 s - release the key and call the technician to clear it.",
]))
A(("h2", "Push mode (move it by hand with the brakes off)"))
A(("steps", [
    "MANUAL, no mission running. On the <b>Manual</b> page press <b>Brakes OFF (push)</b>. The Drives tile shows OFF · push mode.",
    "Push the AMR slowly. Level floor only - it rolls freely.",
    "Press <b>Brakes ON (drive)</b> when done. Nothing drives until you do.",
]))

A(("h1", "Telling the AMR where it is (localisation)", True))
A(("p", "After the AMR is switched on, or after its position was lost, it must be told where it is on the map. A mission only starts "
        "when Localisation reads <b>READY</b>."))
A(("steps", [
    "<b>Run</b> page, tab <b>Map & pose</b>. If the Active map tile does not show the map you use: pick it under <b>View / activate</b> and press "
    "<b>Use this map on the vehicle</b> (only with the AMR standing still). Mode becomes NAVIGATION.",
    "Easiest: park the AMR exactly on the homepost floor mark, facing the same way as the mark, and press <b>Initial pose = route start</b> "
    "(for the mission picked on the Mission tab) or <b>Initial pose = survey mark</b>.",
    "Otherwise: press <b>Set initial pose on map</b>, press on the map where the AMR really is, drag towards where it faces, release.",
    "Localisation shows CHECKING. Look at the map: the red scanner dots must lie on the black walls (picture below). If they do not, set the pose again.",
    "When <b>Can confirm</b> shows YES and the dots lie on the walls, press <b>Confirm: scans align</b>. Localisation becomes <b>READY</b>.",
]))
A(("fig", "localise", "Check before you confirm: the scanner dots must lie on the walls of the map.", 0.95))
A(("tip", "If Can confirm stays NO, drive the AMR about half a metre forward and back and turn it a little in MANUAL - this helps it settle."))

A(("h1", "Running a mission", True))
A(("h2", "The quick way: POS buttons"))
A(("steps", [
    "Localisation READY, AMR on the homepost (within 20 cm and 10°), area clear.",
    "Press START once (or Q on the keypad): Selector shows AUTO.",
    "Press <b>Go to POS n</b> on the Run page (Mission tab) or F1-F6 on the keypad. After a 2-second warning the AMR drives to that POS.",
    "At the POS the mission <b>waits</b> (Run state WAITING, with the wait's name). Do the job (load / unload), step clear, then press "
    "<b>Continue</b> (Run page) or <b>START</b>. The AMR drives back to the homepost.",
]))
A(("h2", "Any mission"))
A(("steps", [
    "Localisation READY. Put the AMR on the route's homepost (within <b>0.20 m and 10°</b>).",
    "Only the first time, or to change the mission: on the Mission tab pick the mission and press <b>Load (READY)</b>. After that the last "
    "loaded mission loads by itself every time the AMR is in AUTO with nothing loaded.",
    "Clear the area and stay within reach of the E-stop.",
    "Press <b>START</b>: Selector AUTO; the Run state shows READY (\"loaded by itself ... press START to run it\").",
    "Press <b>START</b> again: Run state EXECUTING. For 2 s nothing moves (warning; the drives power up), then the AMR drives.",
    "At the end: Run state <b>DONE</b>. For another run press START again (still AUTO).",
]))
A(("fig", "fields", "While it drives, the scanner slows and stops the AMR for anything in front.", 0.95))
A(("h2", "While the mission runs"))
A(("table", ["You want to ...", "Do"], [
    ["Stop now (end the mission)", "Press STOP (the AMR goes to MANUAL) or Abort on the web page"],
    ["Stop now in an emergency", "Press the E-stop. To continue: release it, press START"],
    ["Hold it and continue later", "Pause (web). To continue: Prepare resume, then START"],
    ["Continue after a wait point", "Continue (Run page) or START"],
    ["Continue a mission that was aborted or faulted", "Run page, Mission tab: <b>Continue route from here</b>, then START (Chapter 7)"],
], [0.4, 0.6]))
A(("note", "The Run page map keeps the AMR in the middle (FOLLOW is lit). Drag the map to look elsewhere; press FOLLOW to come back."))

A(("h1", "When the AMR stops", True))
A(("p", "Read the <b>Run state</b> and the line under it on the Run page. The Home page also says in one sentence what to do."))
A(("table", ["You see", "Why", "Do"], [
    ["BLOCKED - E-stop", "The E-stop was pressed", "Release it, press START"],
    ["BLOCKED - lidar stop", "Something in the scanner's red field", "Clear the area. It continues by itself"],
    ["BLOCKED - path blocked", "The software sees an object in the path", "Clear it. It tries again by itself, as long as needed"],
    ["BLOCKED - relocalising", "It lost its position and is finding it again (up to 3 times per mission)", "Keep clear. It continues by itself in a few seconds"],
    ["WAITING", "A wait point in the route", "Do the job, step clear, Continue or START"],
    ["Continue refused", "Not in AUTO, not READY, the AMR was moved off the wait point, or E-stop", "Read the reason, fix it, press Continue again"],
    ["FAULT", "A problem it cannot solve alone (see the reason)", "Fix the cause, press <b>Ack fault</b> (or STOP in MANUAL), then <b>Continue route from here</b> + START, or start the mission again from the homepost"],
    ["FAULT - could not find the position again", "Relocalisation failed (often where the floor changed)", "Set the position by hand (Chapter 5), Confirm, Ack fault, Continue route from here"],
    ["FAULT - turn centre drifted", "During a turn on the spot it slid sideways (slippery floor, something under a wheel, a hitched trolley)", "Check the wheels and floor; Ack fault; Continue"],
    ["Localisation LOST", "Position lost while not in a mission", "Set the position again (Chapter 5)"],
    ["Drives not ARMED", "E-stop, push mode left on, or a drive / I/O problem", "Chapter 10"],
], [0.24, 0.36, 0.4]))
A(("h2", "Continue a mission after Abort or FAULT"))
A(("steps", [
    "After a FAULT: fix the cause and press <b>Ack fault</b>. After a restart: activate the same map and set the position (Chapter 5).",
    "Mission tab: the <b>Stopped route</b> box shows where it stopped. Press <b>Continue route from here</b>. Nothing moves.",
    "Make sure the AMR is near the line where it stopped (at most 0.30 m beside it, heading within 20°). If not, drive it there in MANUAL.",
    "START (AUTO), START again: after the 2 s warning it continues from that step.",
]))

A(("h1", "End of shift", True))
A(("steps", [
    "Let the mission finish (DONE) or Abort it. Press <b>STOP</b> so the AMR is in MANUAL.",
    "Run page: <b>Return to idle</b>.",
    "Park the AMR in its parking place, on a level floor.",
    "Shut the PC down properly <b>before</b> cutting the main power: a short press of the PC's power button (do not hold it), and "
    "wait until the PC is off - or ask the engineer. (Check once with the engineer that a short press does this on your vehicle.)",
    "Switch off the main power. Charge or swap the battery as your site does it.",
]))
A(("warn", "Cutting the power while the software runs can leave the motor drivers with their last command and makes the next start "
           "slower (\"The vehicle lost power last time\" - switch the drives off and on)."))

# ---------------------------------------------------------------- PART 2
A(("part", "2", "Maintenance and troubleshooting", "For the technician and the shift leader"))

A(("h1", "Maintenance", True))
A(("p", "The schedule below is a starting point for this vehicle. Follow the manufacturers' instructions for the battery, the motor "
        "drivers and the safety scanner where they say more. Write every check and repair in the logbook."))
A(("table", ["When", "Check", "How / OK when"], [
    ["Every shift", "E-stop", "Press during a slow hand drive: the AMR stops at once. Release: Drives ARMED again"],
    ["Every shift", "Scanner window", "Clean with a soft cloth (no solvent). The scanner must not show a contamination warning"],
    ["Every shift", "Wheels and casters", "Nothing wound around them, no cuts or flat spots, casters turn freely"],
    ["Every shift", "Buttons and keypad", "START → AUTO, STOP → MANUAL; keypad drives in MANUAL"],
    ["Weekly", "Scanner stop test", "In AUTO at low speed, walk into the path: half speed (yellow) then stop (red). Must stop before you"],
    ["Weekly", "Cables and connectors", "Encoder, CAN, scanner and I/O cables tight, not rubbing; no loose plugs on the PC"],
    ["Weekly", "Encoder couplings", "The couplings between encoder and wheel are tight, no play (wobble at the encoder = wrong distance)"],
    ["Weekly", "Floor marks", "Homepost and POS marks still visible and in place"],
    ["Weekly", "Alarms page", "Look at Localisation losses: the same place again and again → the map needs updating (Part 3)"],
    ["Monthly", "Brakes", "In MANUAL, standing still, try to push the AMR: the brakes hold"],
    ["Monthly", "Drive straight test", "Drive 10 m straight by hand: it does not pull to one side (Wheels tile R − L near 0)"],
    ["Monthly", "Map backup", "Engineer: push the maps to GitHub (Chapter 17)"],
    ["Monthly", "Disk space", "No DISK_LOW alarm. Engineer removes old reports / trash if needed"],
    ["After any collision", "Scanner and mounting", "Scanner not moved or loose; do a scanner stop test before running AUTO again"],
    ["After a power cut", "Drives", "Switch the drives off and on; check Home shows no UNCLEAN_SHUTDOWN"],
    ["When the floor layout changes", "Map", "Engineer updates the map (dynamic areas or Re-map area, Part 3)"],
], [0.17, 0.22, 0.61]))

A(("h1", "Troubleshooting", True))
A(("fig", "trouble", None, 1.0))
A(("h2", "By symptom"))
A(("table", ["Symptom", "Likely cause", "What to do"], [
    ["Web page does not open", "Tablet not on the AMR's Wi-Fi; AMR off or still starting; wrong address", "Check Wi-Fi and the address on the cover; wait 1 minute after power-on; reload (↻ button top right)"],
    ["Page shows old values / greyed tiles (STALE)", "Wi-Fi weak or dropped", "Move closer, reload. The AMR itself keeps working"],
    ["Selector STALE / INVALID", "Panel I/O module (192.168.1.40) not answering", "Check its power and network cable. The AMR stays in MANUAL with brakes on until it is back"],
    ["Drives OFF in MANUAL", "E-stop pressed, push mode on, I/O module off, CAN down", "Release E-stop; Manual page Brakes ON; check the Alarms page"],
    ["Drives FAULT \"stalled\"", "Wheel commanded but did not turn for 2 s: blocked wheel, scanner stop while holding a key, driver off", "Remove the cause; technician clears it (Chapter 18)"],
    ["Drives FAULT \"AGAINST the command\"", "Wheel turned the wrong way (wiring or setting)", "<b>Do not drive.</b> Call the engineer"],
    ["Drives FAULT \"encoder feedback lost\" / CAN_DOWN", "Encoder cable or USB-CAN adapter unplugged", "Check the USB-CAN adapter and encoder cables; then the fault can be cleared"],
    ["Scanner SILENT / STALE", "Scanner cable or power; scanner network (192.168.3.2)", "The AMR can still be driven by hand; no missions until fixed"],
    ["Stops with nothing in front", "Dirty scanner window, a reflection, something low in the field", "Clean the window; check the floor in front; call the engineer if it repeats at one place"],
    ["Position lost often at one place", "The floor changed there (trolleys, pallets, a new machine)", "Look at Alarms → Localisation losses → Show on map; engineer marks a dynamic area or re-maps (Part 3)"],
    ["Wobbles left-right on straights", "Few fixed things in view (dynamic areas); worn caster", "Report the place; engineer checks map and casters"],
    ["Start refused \"... from the route start\"", "The AMR is not on the homepost (more than 0.20 m / 10° off)", "STOP (MANUAL), drive onto the mark, START twice"],
    ["\"localisation not READY\"", "Position not set or not confirmed", "Chapter 5"],
    ["Keypad does nothing", "Dongle unplugged; keypad battery; keypad E pressed", "Plug the dongle in; batteries; press ENTER to release the keypad stop"],
    ["Home: \"The vehicle lost power last time\"", "Power was cut while the software ran", "Switch the drives off and on"],
    ["Home: a \"Needs service\" message", "Part of the software stopped", "Press <b>Recover</b> or <b>Restart</b> on the Home page as it says. Wait 1 minute"],
], [0.26, 0.37, 0.37]))

A(("h1", "Alarm messages", True))
A(("p", "Every alarm has a code (for example <b>LOC_JUMP</b>). The Alarms page shows the code, a title and what to do. The most common ones:"))
A(("table", ["Code", "Means", "Do"], [
    ["ESTOP", "Emergency stop pressed", "Make safe, release the E-stop. (No Reset button on this AMR)"],
    ["ESTOP_HOLD", "Mission held by the E-stop", "Area clear, then START or Continue"],
    ["FIELD_PROTECTIVE / FIELD_BLOCKED", "Something in the red scanner field", "Clear it; continues by itself"],
    ["FIELD_WARNING", "Something in the yellow field: half speed", "Nothing; full speed returns when clear"],
    ["PATH_BLOCKED", "Path blocked ahead", "Clear the route; it retries by itself"],
    ["RUN_WAITING", "Waiting at a wait point", "Do the job, Continue or START"],
    ["CONTINUE_REFUSED", "Continue did not start", "Fix what the detail says, Continue again"],
    ["LOC_CORRECTED", "Position corrected, kept going", "Nothing. Same place often → tell the engineer"],
    ["LOC_RELOCALIZING / LOC_RELOCALIZED", "Lost its position in a mission / found it again", "Keep clear; it continues by itself"],
    ["LOC_RELOC_FAILED", "Could not find its position again", "Set position by hand, Confirm, Ack fault, Continue route from here"],
    ["LOC_JUMP / LOC_LOST / LOC_SCAN_MISMATCH / LOC_COV_GREW", "Position lost or uncertain", "Set the position again (Chapter 5)"],
    ["LOC_NOT_SET / LOC_NOT_CONFIRMED", "Position not given / not confirmed yet", "Chapter 5"],
    ["EXEC_OFF_PATH / EXEC_ENDPOINT_MISSED / EXEC_TURN_FAILED / EXEC_OVERSHOOT", "The AMR did not follow the route within its limits", "Ack fault; check for an obstacle or slippery floor; Continue route from here"],
    ["EXEC_PREREQ_LOST", "A sensor or the panel dropped out in a mission", "Ack fault, look at the Alarms page"],
    ["DRIVES_NOT_READY", "Motors not powered", "Release E-stop, Brakes ON; in AUTO press START"],
    ["UNCLEAN_SHUTDOWN", "Power was cut last time", "Switch the drives off and on"],
    ["AUTO_LOAD_REFUSED", "The last mission could not load itself (other map active)", "Load a mission on the Run page"],
    ["DEST_REFUSED", "POS button did not start", "Read the detail: MANUAL, a mission running, not on the homepost ..."],
    ["SCANNER_SILENT / SCANNER_STALE", "No data from the safety scanner", "Drive by hand only; call the engineer"],
    ["PANEL_STALE", "Panel I/O not answering", "Check the I/O module; call the engineer"],
    ["CAN_DOWN", "Motor encoders cannot be reached", "Check the USB-CAN adapter; call the engineer"],
    ["DISK_LOW / DISK_FULL", "Storage nearly full", "Call the engineer"],
    ["Needs service: ... (BASE_EXITED, SUPERVISOR_DOWN, WEB_DOWN ...)", "A software part stopped", "Recover / Restart on the Home page; if it repeats, Save report and call the engineer"],
], [0.33, 0.33, 0.34]))
A(("note", "Messages that mention a \"blue Reset button on the cabinet\", \"the pendant enable switch\" or \"the safety reset\" come from "
           "another vehicle type. On this AMR: release the E-stop; in MANUAL, STOP acknowledges a fault."))

A(("h1", "Position problems and map changes", True))
A(("p", "Every time the AMR loses or corrects its position, it writes a report. <b>Alarms → Localisation losses</b> lists them, "
        "newest first, with the likely cause. <b>Show on map</b> draws the moment: green dots = scanner on mapped walls, "
        "red = on things that are not on the map, orange = through mapped walls (something mapped is gone)."))
A(("table", ["The report says", "Means", "Who / what"], [
    ["Many % \"on things not on the map\"", "Trolleys, pallets, new objects", "Engineer: mark the area dynamic, or re-map it if they stay"],
    ["Many % \"through mapped walls\"", "Something mapped has gone", "Engineer: re-map that area"],
    ["Correction along the path, scan fitted well", "Odometry drift in a long aisle - normal if small", "Nothing, unless very frequent"],
    ["A sensor went quiet", "Cable or network of that sensor", "Technician: check that sensor's cable"],
    ["Speed in the report far above 0.8 m/s", "Wheel measurement problem, not real speed", "Engineer: check encoders (Chapter 18)"],
], [0.32, 0.34, 0.34]))
A(("tip", "Write down the time and place when the AMR stops or wobbles. The engineer can find it in the report."))

A(("h1", "Collecting evidence for the engineer", True))
A(("steps", [
    "As soon as possible after the problem: <b>Alarms</b> page → <b>Save report</b>. Read the report's name to the engineer (or take a photo).",
    "Write down: time, place on the floor, what the AMR did, what the screen said.",
    "Photos of the screen and of the floor at the place help a lot.",
    "Do not switch the AMR off before saving the report if you can avoid it.",
]))

# ---------------------------------------------------------------- PART 3
A(("part", "3", "Engineer tasks", "Maps, routes, updates and backups - needs the engineer PIN"))

A(("h1", "The engineer view", True))
A(("p", "Press <b>Op</b> in the page header and type the engineer PIN. Extra pages appear: <b>Maps</b> and <b>Routes</b> (under Run), "
        "<b>Monitor</b>, <b>I/O</b>, <b>Params</b>, <b>Blind run</b>. Press <b>Eng</b> to go back to the operator view. The PIN is stored on the "
        "robot in <b>~/.amr/web.json</b>; give it only to trained people. The detailed engineer guide is <b>OPERATE_AMR_QR.md</b> and "
        "<b>RUNBOOK.md</b> in the software folder."))

A(("h1", "Maps", True))
A(("h2", "Make a new map"))
A(("steps", [
    "Tape an arrow on the floor (the survey mark). Park the AMR on it, facing the arrow, still.",
    "Maps page: Map id, mark description, <b>New map (start survey)</b>. Mode becomes MAPPING.",
    "Drive slowly through every aisle the routes will use (web jog, keypad, or push mode). Avoid fast turns; loop back over known areas.",
    "Drive back onto the same mark, same direction, <b>Returned to start</b>. The closure should be a few cm and 1-2°.",
    "<b>Save map revision</b>.",
]))
A(("h2", "Keep the map up to date"))
A(("table", ["Change on the floor", "Tool (Maps → Edit areas)"], [
    ["Things that come and go (trolleys, pallets)", "<b>Dynamic area</b> rectangle - localisation ignores them. Keep fixed walls/machines outside it"],
    ["Noise dots", "<b>Remove noise</b>"],
    ["A permanent new obstacle", "<b>Static obstacle</b>"],
    ["A stretch really changed (machine moved, rack gone)", "<b>Re-map area</b>: draw a Re-map box, Continue mapping through the stretch, then Preview and Save on the new revision"],
    ["Map is tilted", "<b>Rotate map</b> with a reference line along a long wall"],
    ["Add a new area", "<b>Continue mapping</b> from the revision in use"],
], [0.4, 0.6]))
A(("p", "Edit areas also shows <b>Changes seen during runs</b>: red = mapped but gone, yellow = there now but not on the map, with advice "
        "per area. After every new revision: in Routes open each route on it, <b>Validate</b>, <b>Save revision</b>, <b>Create mission</b>, then use "
        "the new revision on the Run page."))

A(("h1", "Routes, missions and POS", True))
A(("bul", [
    "<b>Routes</b> page: pick the map revision, type a route id, set the start pose (homepost: survey mark, or <b>Start = AMR here</b>).",
    "Add steps: straight, turn (45/90/180/270), arc (radius ≥ 1 m), short reverse, <b>Wait</b> (named stop), <b>Go home</b> (last step).",
    "Speeds: straights 0.80 m/s; curves 0.30 m/s unless the curve has its own speed box; slow zones for doors, crossings and walkways.",
    "<b>Validate</b> (red = the AMR would touch a wall), <b>Save revision</b>, <b>Create mission from saved revision</b>.",
    "POS routes: draw the main route once; for each POS use <b>Branch → Stop point on map</b>; route ids pos1 ... pos6 (used by the POS buttons and F1-F6).",
    "Remove maps, routes or missions with the Remove buttons; nothing is erased - it goes to <b>~/amr_maps/.trash</b>.",
]))
A(("warn", "Before a first run at 0.80 m/s check the scanner's red field is long enough (about 1.0 m, yellow about 1.8 m) and test "
           "the stopping distance against a box."))

A(("h1", "Software updates, restarts and backups", True))
A(("h2", "Update the software"))
A(("code", "sudo systemctl stop amr.service\ncd ~/agv_can && git pull\ncd amr_ws && source /opt/ros/jazzy/setup.bash\n"
           "colcon build --packages-select <changed packages>\nsudo systemctl start amr.service"))
A(("p", "Then reload the web page with the ↻ button (or Ctrl+Shift+R)."))
A(("h2", "Back up the maps (GitHub, private repository amr-maps)"))
A(("code", "cd ~/amr_maps && git add -A && git commit -m \"maps $(date +%F)\" && git push"))
A(("p", "Do it after every new map revision, route or mission. To restore on a new PC: stop the service, "
        "<b>git clone</b> the repository to ~/amr_maps."))
A(("h2", "Restart and logs"))
A(("table", ["Task", "Command / place"], [
    ["Service state / live log", "systemctl status amr.service · journalctl -u amr.service -f"],
    ["Restart the software", "Home page Restart, or sudo systemctl restart amr.service"],
    ["Logs", "~/.amr/logs/base.log (drives, panel), layer.log (map, navigation), events.jsonl (what happened)"],
    ["Reports (Save report)", "~/.amr/reports"],
    ["Localisation loss reports", "~/.amr/loc_losses"],
    ["Settings of this vehicle", "~/agv_can/profiles/amr-qr-01.json, ~/agv_can/amr_ws/deploy/amr.env"],
], [0.32, 0.68]))

A(("h1", "Useful tools and fixes", True))
A(("table", ["Problem", "Tool / command"], [
    ["Clear a latched drive fault (stalled, encoder lost)", "ros2 service call /drives/ack_fault std_srvs/srv/Trigger"],
    ["Record a drive for analysis (send the folder to the engineer)", "~/agv_can/tools/run_log.sh, drive, Ctrl-C"],
    ["Does it pull to one side?", "python3 ~/agv_can/tools/straight_run.py"],
    ["Is the map the right length?", "python3 ~/agv_can/tools/map_scale.py"],
    ["Wheels vs lidar check", "python3 ~/agv_can/tools/scan_odom_sync.py"],
    ["Keypad not found", "ros2 run amr_base qr_keypad --list (service stopped)"],
    ["I/O, encoders, IMU bench tests", "ros2 run amr_base qr_calibrate io | enc | imu (service stopped)"],
], [0.45, 0.55]))
A(("p", "Before running a tool by hand: <b>source ~/agv_can/amr_ws/install/setup.bash</b> and <b>source ~/agv_can/amr_ws/env/vehicle.sh</b>. "
        "Tools that drive the motors need the service stopped and the AMR on blocks or in a clear area."))

A(("h1", "Quick reference card", True))
A(("table", ["", ""], [
    ["Start a mission", "Position READY · AMR on homepost · START (AUTO) · START (go)"],
    ["Go to a POS", "AUTO · Go to POS n (web) or F1-F6 · at the POS: Continue / START"],
    ["Stop now", "STOP (aborts, MANUAL) · E-stop (holds; release + START continues)"],
    ["Stopped by something in front", "Clear it - continues by itself"],
    ["WAITING", "Do the job, step clear, Continue / START"],
    ["FAULT", "Read reason · fix · Ack fault · Continue route from here · START"],
    ["Position lost", "Run page: set initial pose · check dots on walls · Confirm"],
    ["Drives OFF", "E-stop released? push mode off (Brakes ON)?"],
    ["Strange problem", "Alarms → Save report · note time and place · call the engineer"],
    ["End of shift", "STOP · Return to idle · park · PC shut down · power off"],
], [0.3, 0.7], False))
