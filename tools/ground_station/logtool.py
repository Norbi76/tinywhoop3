#!/usr/bin/env python3
"""Read a flight log from the command line.

    python3 logtool.py flight_20260827_181500.ndjson          # the report
    python3 logtool.py flight_*.ndjson                        # several, one after another
    python3 logtool.py flight.ndjson --csv flight.csv         # status stream as a spreadsheet
    python3 logtool.py flight.ndjson --slice 41.0 48.5        # only what happened in that window
    python3 logtool.py flight.ndjson --events                 # just the timeline

The report is the same one the Replay tab shows, so it is also the thing to paste into a message
when asking somebody what went wrong - it is deliberately narrow enough to quote.

DEPENDENCIES: none beyond the standard library. That is on purpose. The ground station needs
PyQt6, numpy and pyqtgraph, and none of them should be needed to read a log on a machine that
does not have them - including the machine you happen to be sitting at when something breaks.
"""

import argparse
import csv
import sys

from flightlog import FlightLog
from flightreport import build_report

# Columns of the --csv export. Status only: it is the one stream that is uniform, at a fixed
# 50 Hz, and it is what a spreadsheet is good at. The control and pid streams have different
# shapes and are better read as JSON.
CSV_COLUMNS = [
    "t", "armed", "flight_mode", "flags",
    "roll", "pitch", "yaw", "altitude", "climb_rate",
    "velocity_x", "velocity_y", "battery_voltage", "loop_hz",
]


def slice_log(log, start, end):
    """Restricts every record list to [start, end] in place, so the report covers only that."""
    def keep(records):
        return [r for r in records if start <= float(r.get("t", 0.0)) <= end]

    log.status = keep(log.status)
    log.control = keep(log.control)
    log.pid = keep(log.pid)
    log.link = keep(log.link)
    log.events = keep(log.events)
    # So every rate in the report is divided by the window, not by the whole flight.
    log.window = (start, end)
    return log


def write_csv(log, path):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_COLUMNS + ["motor_fl", "motor_fr", "motor_rl", "motor_rr"])
        for row in log.status:
            motors = row.get("motor", [0, 0, 0, 0])
            writer.writerow([row.get(column, "") for column in CSV_COLUMNS] + list(motors))
    return len(log.status)


def print_events(log):
    for event in log.events:
        detail = " ".join(f"{k}={v}" for k, v in event.items()
                          if k not in ("rec", "t", "kind"))
        print(f"t={float(event.get('t', 0)):8.2f}  {event.get('kind', '?'):<16} {detail}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Summarise a tinywhoop flight log.",
        epilog="A log with no footer was not stopped cleanly - the report says so at the top.")
    parser.add_argument("paths", nargs="+", help=".ndjson or .ndjson.gz flight log(s)")
    parser.add_argument("--csv", metavar="OUT",
                        help="also write the status stream to a CSV (single input only)")
    parser.add_argument("--slice", nargs=2, type=float, metavar=("START", "END"),
                        help="report only on this time window, in seconds")
    parser.add_argument("--events", action="store_true",
                        help="print only the event timeline")
    parser.add_argument("--width", type=int, default=78, help="report width (default 78)")
    arguments = parser.parse_args(argv)

    if arguments.csv and len(arguments.paths) > 1:
        parser.error("--csv takes a single input log")

    for index, path in enumerate(arguments.paths):
        try:
            log = FlightLog(path)
        except OSError as error:
            print(f"{path}: {error}", file=sys.stderr)
            continue

        if arguments.slice:
            log = slice_log(log, arguments.slice[0], arguments.slice[1])

        if index:
            print()

        if arguments.events:
            print(f"# {path}")
            print_events(log)
        else:
            print(build_report(log, width=arguments.width))

        if arguments.csv:
            count = write_csv(log, arguments.csv)
            print(f"\nwrote {count} status rows to {arguments.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
