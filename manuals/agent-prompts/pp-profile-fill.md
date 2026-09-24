# Agent prompt: record the MEXE02 PP settings in the vehicle profile

Use this after a person has set the PP safety parameters in **both** drives with MEXE02, saved them to non-volatile memory and power-cycled the drives, and after [pp-blind-run-deploy.md](pp-blind-run-deploy.md) has passed. There is no vendor confirmation; the decision reference below records the internal decision to run PP.

Fill in the INPUTS block, then paste everything below the line into the coding agent on the AGV PC.

---

## INPUTS (filled in by the operator)

```text
Values set in MEXE02 on BOTH drives (write the number actually entered):
  max_torque_permille     (6072h, Max torque, p5?)                       = 10000    # default, unchanged (no cap)
  following_error_counts  (6065h, Position deviation alarm, p6)          = 36000    # CHANGED from 108000 (1 motor turn, ~19 mm wheel)
  position_window_counts  (6067h, IN-POS positioning completion range, p7) = 1000   # CHANGED from 18 (~0.5 mm wheel)
  halt_option             (605Dh, Halt option, p5?)                      = 1        # default, unchanged
  fault_reaction          (605Eh, Stopping method at alarm generation, p6) = 2      # default, unchanged (follow quick stop)
  quick_stop_decel        (6085h, Quick stop rate, p7)                   = 1600     # CHANGED from 1000 (r/min)/s

Decision reference (goes into pp.vendor_ref):
  "Internal engineering decision, 2026-09-18, no vendor consultation: PP on BLMR6400SKM-GFV-B
   (400 W, 1:30) without motion extension. Bounds: ramps 800 r/min/s, following error 36000,
   IN-POS 1000, quick stop 1600 r/min/s, PP speed cap 0.30 m/s, bench on blocks before floor."
Unlock PP now? (yes / no)                                              = yes
PP speed cap for the first sessions, m/s (<= 0.8)                      = 0.30
```

---

You are on the vehicle PC of a 150 kg differential-drive AMR (two Oriental Motor BLV-R drives on `can0`, nodes 1 and 2; motor BLMR6400SKM-GFV-B, 400 W, 1:30 gearhead). A person has just set the drives' profile-position (pp) safety parameters with MEXE02. Your job:
- **read those parameters back from both drives (read-only)**;
- **check them against the operator's INPUTS above**;
- **record them in `profiles/agv-01.json`** under `pp`.

The drive owner (`amr_base/drive_node`) compares these profile values with the drives before every pp move and refuses on any difference. So the profile must hold exactly what is in the drives.

## Hard rules

- **Read only.** Never write a drive object: no SDO download, no `cansend`, no `drive_forward.py --go`, no `1010h` store. If the drives disagree with the INPUTS, stop and tell the operator to correct them in MEXE02. Do not "fix" them from software.
- **Do not move the vehicle.** Do not arm, jog, plan or press anything.
- `profiles/agv-01.json` is the only file you edit. Do not touch `guard.py`, `pp.py`, `config.py`'s validation or the tests to make something pass.
- Set `pp.enabled: true` **only** if the operator wrote `yes` AND the decision reference is filled in. Otherwise leave `enabled: false` and just record the values. The profile loader refuses `enabled: true` with an empty `vendor_ref` or any null value; that is intended.
- Ask the operator before stopping or restarting `amr.service`, and wait for a yes: someone at the vehicle, E-stop in reach, selector MANUAL.

## Steps

1. **Check the inputs.** Every value must be a whole number:
   - 6072h: 0–10000
   - 6065h: 0–10,000,000
   - 6067h: 0–65,535
   - 605Dh: 1
   - 605Eh: 0, 1 or 2
   - 6085h: ≥ 1

   Stop and ask about anything blank or out of range. Warn, but do not block, if:
   - 6067h < 200, because "target reached" may never switch on;
   - 6065h > 108000, because that is looser than the default;
   - 605Eh = 0, because that is an immediate stop, which the manual warns against with a gearhead;
   - 6085h < 1600, because that stops more gently than the normal decel, so stops are longer.
2. **Free the bus (operator's yes first).**
   ```bash
   systemctl is-active agv_controller amr        # agv_controller must be inactive
   sudo systemctl stop amr.service                # the ordered shutdown de-energises the drives
   ```
3. **Read both drives with a throwaway script.** Put it in `/tmp`, never in the repo. It must take the CAN owner lock, so it fails safely if anything else owns `can0`:
   ```python
   # /tmp/read_pp_objects.py
   import sys; sys.path[:0] = ["/home/gvipc-evo-01/agv_can/drivers/canbus", "/home/gvipc-evo-01/agv_can/core"]
   from verify_drivers import claim_bus, open_bus, sdo_read, u32
   OBJ = {"max_torque_permille": 0x6072, "following_error_counts": 0x6065, "position_window_counts": 0x6067,
          "halt_option": 0x605D, "fault_reaction": 0x605E, "quick_stop_decel": 0x6085,
          "modes_of_operation(stored)": 0x6060, "modes_display": 0x6061}
   lock = claim_bus("pp-profile-readback")
   if lock is None:
       sys.exit(2)
   bus, how = open_bus()
   try:
       for name, idx in OBJ.items():
           row = []
           for node in (1, 2):
               st, payload, note, _ = sdo_read(bus, node, idx, 0)
               row.append(u32(payload) if st else f"ERR({note})")
           print(f"{name:28s} {idx:04X}h  left={row[0]}  right={row[1]}")
   finally:
       bus.shutdown()
   ```
   Run it with `python3 /tmp/read_pp_objects.py` and paste the output in your report.
   - 605Dh and 605Eh are INT16, so a value above 32767 means a negative number was stored: report it.
   - 6060h should read 3 (pv). Report it if not, but do not change it; the software writes it at every arm.
4. **Compare.** For each of the six objects, left must equal right, and both must equal the operator's INPUT. On any difference, stop, show the table, and ask the operator to fix MEXE02 and power-cycle the drives. Then re-run step 3.
5. **Edit `profiles/agv-01.json`**, `pp` section only, using the **values read back**:
   ```json
   "pp": {
     "enabled": false,
     "vendor_ref": "<the decision reference from INPUTS, verbatim>",
     "max_speed_mps": 0.30,
     "expect": {
       "max_torque_permille": <6072h>,
       "following_error_counts": <6065h>,
       "position_window_counts": <6067h>,
       "halt_option": <605Dh>,
       "fault_reaction": <605Eh>,
       "quick_stop_decel": <6085h>
     }
   }
   ```
   Set `"enabled": true` only under the rule above. Keep the file's existing formatting (2-space indent) and change nothing else.
6. **Validate offline.**
   ```bash
   cd ~/agv_can
   python3 -c "import sys; sys.path[:0]=['.','core','drivers','drivers/canbus']; import config; print(config.PP_ENABLED, config.PP_EXPECT, config.PP_MAX_SPEED_MPS)"
   python3 tests/run_all.py                      # 869 checks, all passed
   cd amr_ws && env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_base/test src/amr_web/test
   git diff profiles/agv-01.json
   ```
7. **Restart and check (operator's yes first).**
   ```bash
   sudo systemctl start amr.service
   source ~/agv_can/amr_ws/env/vehicle.sh
   ros2 topic echo --once /drives/pp_status
   curl -s localhost:5001/api/commissioning/capabilities | python3 -m json.tool | grep -E 'pp_available|pp_reason|pp_max_speed'
   ```
   Expected results:
   - `enabled: false`: `available: false`, reason "pp locked in the profile (pp.enabled false)".
   - `enabled: true`: `available: true` once the drives are armed. Availability does not read the drives; the full readback happens at the start of every pp move, and a mismatch then refuses the move with the object named.

## Report back

1. The step-3 readback table.
2. The comparison result.
3. The profile diff.
4. The test results.
5. The `/drives/pp_status` output.
6. Whether PP is now locked or unlocked, and why.

Do not commit or push unless the operator asks. The first pp moves are a **bench test on blocks** (RUNBOOK.md §3), done by the operator, not by you.
