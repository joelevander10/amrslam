#!/usr/bin/env python3
"""Read the Modbus TCP digital I/O module. Read-only - nothing is written.

    modbus_io.py            one scan, printed as a table
    modbus_io.py watch      re-scan until Ctrl-C, printing only on CHANGE

Deliberately read-only, the same split the other bench tools here take: on a
150 kg vehicle a script that can energise an output is a different kind of tool
and belongs behind its own --go flag, if it is ever wanted at all.

`watch` prints on change rather than on a timer because that is what makes it
useful for buzzing out a harness: short a channel and exactly one line appears,
naming it. A scrolling table of unchanged bits tells you nothing.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import config  # noqa: E402

OK, BAD, WARN = "\033[32mOK\033[0m", "\033[31mFAIL\033[0m", "\033[33mWARN\033[0m"


def read_all(client):
    """-> (di, do). Raises on any Modbus error."""
    out = []
    for fn, base, count, what in (
            (client.read_discrete_inputs, config.DIO_DI_BASE,
             config.DIO_NUM_DI, "discrete inputs"),
            (client.read_coils, config.DIO_DO_BASE,
             config.DIO_NUM_DO, "coils")):
        r = fn(base, count=count, device_id=config.DIO_DEVICE_ID)
        if r.isError():
            raise IOError(f"{what} at {base}: {r}")
        out.append([bool(b) for b in r.bits[:count]])
    if config.DIO_DI_FLIPPED:
        out[0] = [not b for b in out[0]]
    return out[0], out[1]


def bar(bits):
    return "".join("1" if b else "0" for b in bits)


def table(di, do):
    names_i, names_o = config.DIO_DI_NAMES, config.DIO_DO_NAMES
    width = max([len(n) for n in names_i + names_o] + [4])
    rows = max(len(di), len(do))
    print(f"    {'':<6}{'in':<4} {'name':<{width}}   {'':<6}{'out':<4} name")
    print("    " + "-" * (2 * (12 + width) + 3))
    for i in range(rows):
        left = right = ""
        if i < len(di):
            left = (f"DI{i:02d}   {'ON ' if di[i] else '.  '} "
                    f"{names_i[i]:<{width}}")
        if i < len(do):
            right = (f"DO{i:02d}   {'ON ' if do[i] else '.  '} "
                     f"{names_o[i]:<{width}}")
        print(f"    {left}   {right}".rstrip())


def watch(client):
    # Line-buffer explicitly. stdout is block-buffered whenever this is piped
    # or teed, which for a live tool means the change you are watching for does
    # not appear until 4 KB have accumulated - i.e. never.
    say = lambda s: print(s, flush=True)                     # noqa: E731

    say("watching for CHANGES only - short a channel to identify it. "
        "Ctrl-C to stop.\n")
    prev_di = prev_do = None
    while True:
        di, do = read_all(client)
        if prev_di is None:
            say(f"    baseline  DI {bar(di)}  DO {bar(do)}")
        else:
            for tag, now, was, names in (("DI", di, prev_di, config.DIO_DI_NAMES),
                                         ("DO", do, prev_do, config.DIO_DO_NAMES)):
                for i, (a, b) in enumerate(zip(now, was)):
                    if a != b:
                        name = f"  {names[i]}" if names[i] else ""
                        say(f"    {time.strftime('%H:%M:%S')}  {tag}{i:02d} "
                            f"{'OFF -> ON' if a else 'ON -> OFF'}{name}")
        prev_di, prev_do = di, do
        time.sleep(config.DIO_SCAN_PERIOD_S)


def main():
    ap = argparse.ArgumentParser(
        description="Read the Modbus TCP digital I/O module (read-only).")
    ap.add_argument("mode", nargs="?", default="scan", choices=("scan", "watch"))
    args = ap.parse_args()

    if not config.DIO_ENABLED:
        print(f"{WARN} dio.enabled is false in profile {config.PROFILE_NAME} - "
              f"reading anyway.")

    from pymodbus.client import ModbusTcpClient
    client = ModbusTcpClient(config.DIO_IP, port=config.DIO_PORT,
                             timeout=max(config.DIO_TIMEOUT_S, 1.0))
    if not client.connect():
        print(f"{BAD}: no answer at {config.DIO_IP}:{config.DIO_PORT}")
        return 2
    print(f"connected to {config.DIO_IP}:{config.DIO_PORT} "
          f"(unit {config.DIO_DEVICE_ID})\n")

    try:
        if args.mode == "watch":
            return watch(client) or 0
        di, do = read_all(client)
        print(f"    DI {bar(di)}      DO {bar(do)}\n")
        table(di, do)
        on = [i for i, b in enumerate(di) if b]
        print(f"\n    {OK}: {len(di)} inputs, {len(do)} outputs. "
              f"inputs high: {on if on else 'none'}")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except Exception as e:                  # noqa: BLE001 - report, don't trace
        print(f"    {BAD}: {e}")
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
