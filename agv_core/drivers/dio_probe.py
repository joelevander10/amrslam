"""Bench tool: hold one DIO coil high for a fixed time, for probing the output.

    python3 -m agv_core.drivers.dio_probe              # DO0 high for 180 s
    python3 -m agv_core.drivers.dio_probe -c 3 -t 30   # DO3 high for 30 s

Drives the coil through DioLink, so it gets the same safety the runtime does:
the claim is renewed every RENEW_S with a short hold, so if this process dies
the coil falls low at the next scan instead of latching. Ctrl-C (or the timer
running out) goes through DioLink.stop(), which writes the coil low and waits
for the readback to confirm it.

Takes the "dio" owner lock, so it refuses to run while amr.service owns the
module.
"""
import argparse
import sys
import time

from agv_core import config, ownerlock
from agv_core.drivers.dio import DioLink

RENEW_S = 0.1       # how often the claim is pushed out
HOLD_S = 0.5        # deadline per claim: the coil drops this long after a hang
REPORT_S = 10.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-c", "--channel", type=int, default=0)
    ap.add_argument("-t", "--seconds", type=float, default=180.0)
    args = ap.parse_args(argv)

    if not config.DIO_ENABLED:
        sys.exit("dio.enabled is false in the profile")
    if not 0 <= args.channel < config.DIO_NUM_DO:
        sys.exit(f"channel must be 0..{config.DIO_NUM_DO - 1}")
    try:
        lock = ownerlock.acquire("dio", "dio_probe")  # noqa: F841 - held until exit
    except ownerlock.OwnerBusy as e:
        sys.exit(f"DIO is in use ({e}). Stop amr.service first.")

    link = DioLink()
    link.start()
    ch = args.channel
    print(f"DIO {config.DIO_IP}:{config.DIO_PORT}  DO{ch} HIGH for {args.seconds:.0f} s"
          "  (Ctrl-C to drop early)")
    end = time.monotonic() + args.seconds
    next_report = 0.0
    try:
        while (now := time.monotonic()) < end:
            link.set_coil(ch, True, HOLD_S)
            if now >= next_report:
                s = link.snapshot()
                state = "HIGH" if s["do"][ch] else "low"
                print(f"  {end - now:6.1f} s left  readback DO{ch}={state}"
                      f"  comms_ok={s['comms_ok']}  {s['detail']}", flush=True)
                next_report = now + REPORT_S
            time.sleep(RENEW_S)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        link.stop(quiesce_s=2.0)
        print(f"DO{ch} driven low, link closed")


if __name__ == "__main__":
    main()
