"""Turns a flight log into a text report you can read, or paste to somebody, in one screen.

WHAT THIS IS TRYING TO ANSWER
    "It didn't fly right - what happened?" A 15 MB log cannot answer that by being looked at. This
    reduces it to the handful of numbers that actually discriminate between the usual causes, and
    says which ones look wrong rather than leaving that to the reader.

    Every section exists because it separates one cause from another:

    LINK          A control link that dropped explains everything downstream of it, so it is
                  checked first. Nothing else in the report means much across a gap.
    ARM SESSIONS  Each flight attempt separately, with how it ENDED. "Disarmed by the pilot" and
                  "the link went away and the watchdog dropped it" look identical from the air.
    COMMAND vs    The one pair of traces that distinguishes "the drone drifted" from "the pilot
    ACTUAL        was commanding it there". Without the control echo this section is unavailable,
                  which is the whole reason for the firmware change in uart_link.c.
    MOTORS        A drone that drifts one way with a level, zero command is usually a hardware
                  asymmetry, and the mixer output is where that shows: at a steady hover the four
                  motors should sit close together, and the one working hardest is on the side
                  the drone is falling towards.
    SENSORS       An outer loop that never engaged because its validity flag was never set is a
                  very common "it ignored my command" - and it is invisible unless you look.
    PID           Saturation and integrator clamping, per loop. A loop pinned at its limit cannot
                  be fixed by changing its gains, and that is worth knowing before a tuning session.

WHY PLAIN TEXT
    It reads in a terminal, it pastes into a message, and it survives being quoted. The Replay tab
    shows the same report next to the plots.

WHAT IT DOES NOT DO
    It does not guess at causes it cannot see. Where a number is missing, it says the number is
    missing and why, rather than substituting a default that reads like a measurement.
"""

import math

# Status flag bits, mirrored from protocol.py so this module can be used without a link open.
STATUS_FLAG_LINK_OK = 1 << 0
STATUS_FLAG_ATTITUDE_INIT = 1 << 1
STATUS_FLAG_ALT_VALID = 1 << 2
STATUS_FLAG_VEL_VALID = 1 << 3
STATUS_FLAG_KILLED = 1 << 4

CTRL_FLAG_KILL = 1 << 0
CTRL_FLAG_HOLD = 1 << 1

MODES = ["ANGLE", "ALT HOLD", "POS HOLD"]

# telemetry_status_payload_t.motor[] is "in mixer order". flight_control_mix() writes it as
# front-left, front-right, rear-left, rear-right - see the mix[] block in flight_control.c.
MOTOR_NAMES = ["front-left", "front-right", "rear-left", "rear-right"]

# A status gap longer than this is treated as a hole in the record rather than as data. The status
# stream is 50 Hz, so a quarter second is twelve missed frames - well past jitter.
STATUS_GAP_S = 0.25

# Motors are compared only where the drone was actually flying: below this the mixer is at its
# idle floor and the four outputs carry no information about balance.
HOVER_THROTTLE_FLOOR = 0.15

# Flag a motor spread wider than this fraction of the mean as an asymmetry worth chasing.
MOTOR_SPREAD_WARN = 0.15

# How old a command frame may be and still be treated as the command in force.
#
# WITHOUT THIS BOUND the command-versus-actual section quietly lies. Both streams are 50 Hz, but
# they fail independently: control echoes are dropped when the module's echo queue backs up, and
# a Wi-Fi glitch can take out one without the other. Pairing each status frame with "the most
# recent command, however old" then compares the drone's attitude at t=60 s against a demand from
# t=10 s, and the "hands off" test concludes the pilot was asking for level when in truth nobody
# knows what they were asking for. 0.2 s is ten command frames - generous for jitter, far short of
# a gap. Unpaired frames are dropped and counted, and the count is printed.
COMMAND_MAX_AGE_S = 0.2


def _f(value, digits=2, unit="", missing="--"):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return missing
    return f"{value:.{digits}f}{unit}"


def _num(record, key, default=0.0):
    """One numeric field, coerced safely.

    Everything in this module goes through this rather than record[key]. The report's whole point
    is being pointed at a file - possibly one recorded by an older version, possibly one somebody
    else sent you, possibly one truncated mid-line - and a report that raises KeyError on a single
    malformed record is a report you cannot use precisely when you need it most. A missing or
    unparseable field becomes the default and the rest of the log still gets read.
    """
    value = record.get(key, default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _motors(record, count=4):
    """The motor vector, always `count` floats however malformed the record is."""
    values = record.get("motor")
    if not isinstance(values, (list, tuple)):
        return [0.0] * count
    out = [_num({"m": v}, "m") for v in values[:count]]
    return out + [0.0] * (count - len(out))


def _series(records, key):
    return [_num(r, key) for r in records]


def _span(values):
    if not values:
        return None, None
    return min(values), max(values)


def _mean(values):
    return sum(values) / len(values) if values else None


class ArmSession:
    """One armed stretch, from the first armed status frame to the last."""

    def __init__(self, start_t):
        self.start_t = start_t
        self.end_t = start_t
        self.rows = []
        self.ended_by = "still armed at end of log"

    @property
    def duration(self):
        return self.end_t - self.start_t


def find_arm_sessions(status_records):
    """Splits the status stream into armed stretches.

    Driven off the STATUS frames rather than off arm events, because status is what the flight
    controller actually did - the pilot's arm request and the motors being live are different
    things, and the gap between them (a refused arming gate) is exactly the kind of thing worth
    seeing in a report.
    """
    sessions = []
    current = None

    for row in status_records:
        armed = bool(row.get("armed"))
        t = _num(row, "t")

        if armed and current is None:
            current = ArmSession(t)
            sessions.append(current)
        if current is not None:
            if armed:
                current.end_t = t
                current.rows.append(row)
            else:
                killed = bool(int(_num(row, "flags")) & STATUS_FLAG_KILLED)
                current.ended_by = "KILLED" if killed else "disarmed"
                current = None

    return sessions


def _gaps(times, threshold=STATUS_GAP_S):
    """[(start, end, length)] for every jump longer than `threshold`."""
    out = []
    for previous, current in zip(times, times[1:]):
        delta = current - previous
        if delta > threshold:
            out.append((previous, current, delta))
    return out


def _describe_session_end(session, events, log_duration, control=None):
    """Refines how a session ended using nearby events, which status alone cannot distinguish.

    Three different things all look like "not armed any more" in the status stream, and telling
    them apart is most of the value of reading a log at all:
      - the pilot let go,
      - somebody hit KILL (and from where),
      - a watchdog dropped it because the pilot's browser or the link went away.
    """
    if session.ended_by == "still armed at end of log":
        return session.ended_by

    window = [e for e in events
              if session.end_t - 0.5 <= _num(e, "t") <= session.end_t + 1.5]
    kinds = {e.get("kind") for e in window}

    # The control echo distinguishes the dashboard's KILL from the ground station's: the dashboard
    # sets the flag in the frame itself, whereas the ground station's KILL leaves an event here.
    dashboard_kill = False
    if control:
        dashboard_kill = any(
            int(_num(row, "flags")) & CTRL_FLAG_KILL
            for row in control
            if session.end_t - 0.5 <= _num(row, "t") <= session.end_t + 1.5)

    if session.ended_by == "KILLED":
        if "kill" in kinds:
            return "KILLED from the ground station"
        if dashboard_kill:
            return "KILLED from the dashboard"
        return "KILLED (latch set, source not in this log)"

    if "disarm_request" in kinds:
        return "disarmed by the pilot"
    if "link_down" in kinds:
        return "disarmed after the link dropped (watchdog)"
    if session.end_t >= log_duration - 1.0:
        return "disarmed at the end of the recording"
    return "disarmed (no matching ground-station action - the flight controller dropped it)"


def build_report(log, width=78):
    """Returns the report as one string. `log` is a flightlog.FlightLog."""
    lines = []
    add = lines.append

    def rule(title=""):
        if title:
            add("")
            add(f"--- {title} " + "-" * max(0, width - len(title) - 5))
        else:
            add("-" * width)

    status = log.status
    control = log.control
    events = log.events
    duration = log.duration_s

    # --- Overview ----------------------------------------------------------
    add("=" * width)
    add("FLIGHT LOG REPORT")
    add("=" * width)
    add(f"file           {log.path}")
    add(f"recorded       {log.header.get('started_local', '?')}")
    if getattr(log, "window", None) is not None:
        add(f"window         t={log.window[0]:.1f} .. {log.window[1]:.1f} s  "
            f"(a SLICE of the flight - everything below covers only this)")
    add(f"duration       {_f(duration, 1, ' s')}")
    add(f"records        {len(status)} status, {len(control)} control, "
        f"{len(log.pid)} pid, {len(events)} events")

    if not log.complete:
        add("")
        add("!! NO FOOTER - this recording was not stopped cleanly. The ground station was")
        add("   killed, crashed or lost power while recording. The flight itself may have been")
        add("   fine; the log just stops where it stops.")
    if log.bad_lines:
        add(f"!! {log.bad_lines} unparseable line(s) skipped (normal for the last line of a "
            f"truncated log)")
    if log.footer and log.footer.get("dropped"):
        add(f"!! {log.footer['dropped']} record(s) dropped - the disk could not keep up. "
            f"The log has gaps.")

    if not status:
        add("")
        add("No status frames at all. Either the recording never saw the drone, or the")
        add("telemetry module was not reachable. Nothing below can be computed.")
        return "\n".join(lines)

    status_t = _series(status, "t")

    # --- Link --------------------------------------------------------------
    rule("LINK")
    gaps = _gaps(status_t)
    expected = duration * 50.0
    add(f"status frames  {len(status)} over {_f(duration, 1, ' s')} "
        f"= {_f(len(status) / duration if duration else None, 1, ' Hz')} "
        f"(expected ~50 Hz, {int(expected)} frames)")

    if gaps:
        worst = max(gaps, key=lambda g: g[2])
        add(f"gaps           {len(gaps)} gap(s) over {STATUS_GAP_S}s; "
            f"worst {_f(worst[2], 2, ' s')} at t={_f(worst[0], 1, ' s')}")
        add("               A gap means the ground station stopped hearing the drone. Anything")
        add("               odd that starts at a gap is probably the gap, not the drone.")
        for start, _end, length in gaps[:5]:
            add(f"                 t={start:8.2f} s   {length:5.2f} s")
        if len(gaps) > 5:
            add(f"                 ... and {len(gaps) - 5} more")
    else:
        add("gaps           none over "
            f"{STATUS_GAP_S}s - the downlink held up for the whole recording")

    offline = [row for row in log.link if not row.get("online")]
    if offline:
        add(f"link offline   reported offline in {len(offline)} of {len(log.link)} one-second "
            f"samples")

    link_ok_fraction = sum(1 for r in status
                           if int(_num(r, "flags")) & STATUS_FLAG_LINK_OK) / len(status)
    if link_ok_fraction < 0.999:
        add(f"!! CONTROL LINK  the flight controller reported its control link UNHEALTHY for "
            f"{(1 - link_ok_fraction) * 100:.1f}% of the log.")
        add("               That is the UART/dashboard link, not this one. The flight controller")
        add("               disarms after 300 ms of it - suspect this before anything else.")

    gap_events = [e for e in events if e.get("kind") == "control_gap"]
    if gap_events:
        lost = sum(int(_num(e, "frames")) for e in gap_events)
        add(f"command gaps  {lost} command frame(s) lost in transit across {len(gap_events)} gap(s)")
        add("               Detected from the echo's own sequence counter. The command-versus-")
        add("               actual section below excludes the frames it cannot pair.")

    if not control:
        add("")
        add("no control echo  This log has no record of what the pilot commanded, so the")
        add("                 command-vs-actual section below is unavailable. Either the")
        add("                 telemetry module predates the control echo (see uart_link.c) or")
        add("                 no control frames arrived.")

    # --- Arm sessions ------------------------------------------------------
    rule("ARM SESSIONS")
    sessions = find_arm_sessions(status)
    if not sessions:
        add("The motors were never armed in this recording.")
        arm_requests = [e for e in events if e.get("kind") == "arm_request"]
        if arm_requests:
            add(f"{len(arm_requests)} arm request(s) were sent and none of them took. The flight")
            add("controller's arming gate refuses on: no link, attitude not initialised, throttle")
            add("not at zero, or the kill latch set. Check the flags line below.")
    else:
        for index, session in enumerate(sessions, 1):
            rolls = [abs(_num(r, "roll")) for r in session.rows]
            pitches = [abs(_num(r, "pitch")) for r in session.rows]
            alts = [_num(r, "altitude") for r in session.rows]
            ended = _describe_session_end(session, events, duration, control)
            add(f"#{index}  t={session.start_t:7.1f} -> {session.end_t:7.1f} s   "
                f"({_f(session.duration, 1, ' s')})   ended: {ended}")
            add(f"     max tilt   roll {_f(max(rolls) if rolls else None, 1, ' deg')}   "
                f"pitch {_f(max(pitches) if pitches else None, 1, ' deg')}")
            add(f"     altitude   {_f(min(alts) if alts else None, 2)} .. "
                f"{_f(max(alts) if alts else None, 2, ' m')}")

    armed_rows = [r for r in status if r.get("armed")]

    # --- Command vs actual -------------------------------------------------
    if control and armed_rows:
        rule("COMMAND vs ACTUAL   (armed only)")
        add("Does the drone go where it is told? Pairs each status frame with the command in")
        add("force at that moment. A large error with a near-zero command is drift; a large")
        add("error that tracks the command is a tuning problem.")
        add("")

        paired, unpaired = _pair(armed_rows, control)
        if not paired:
            add("No command frames line up with the armed time - nothing to compare.")
        else:
            if unpaired:
                add(f"NOTE  {unpaired} of {len(armed_rows)} armed frames "
                    f"({unpaired / len(armed_rows) * 100:.0f}%) had no command frame within "
                    f"{COMMAND_MAX_AGE_S:.1f} s and are excluded below - the command stream has")
                add("      gaps the status stream does not. Everything here covers the rest.")
                add("")
            for label, actual_key, command_key, unit in (
                ("roll ", "roll", "roll_sp", "deg"),
                ("pitch", "pitch", "pitch_sp", "deg"),
            ):
                errors = [_num(row, actual_key) - _num(command, command_key)
                          for row, command in paired]
                commands = [abs(_num(command, command_key)) for _row, command in paired]
                rms = math.sqrt(sum(e * e for e in errors) / len(errors))
                peak = max(abs(e) for e in errors)
                add(f"{label}  command |max| {max(commands):5.1f} {unit}   "
                    f"mean |command| {_mean(commands):4.1f}   "
                    f"tracking RMS {rms:5.2f}   peak {peak:5.2f} {unit}")

            # The drift test: what was the drone doing while nothing was being asked of it?
            idle = [(row, command) for row, command in paired
                    if abs(_num(command, "roll_sp")) < 1.0
                    and abs(_num(command, "pitch_sp")) < 1.0]
            if len(idle) > 20:
                idle_roll = _mean([_num(row, "roll") for row, _c in idle])
                idle_pitch = _mean([_num(row, "pitch") for row, _c in idle])
                add("")
                add(f"hands off  {len(idle)} frames with roll and pitch commanded within 1 deg "
                    f"of level:")
                add(f"           mean attitude   roll {_f(idle_roll, 2, ' deg')}   "
                    f"pitch {_f(idle_pitch, 2, ' deg')}")
                if abs(idle_roll) > 2.0 or abs(idle_pitch) > 2.0:
                    add("           !! The drone held a TILT while being asked to stay level.")
                    add("              That is a trim or balance problem, not a piloting one -")
                    add("              check the motor balance below and the IMU level offset.")

                idle_vx = [_num(r, "velocity_x") for r, _c in idle
                           if int(_num(r, "flags")) & STATUS_FLAG_VEL_VALID]
                idle_vy = [_num(r, "velocity_y") for r, _c in idle
                           if int(_num(r, "flags")) & STATUS_FLAG_VEL_VALID]
                if idle_vx:
                    add(f"           mean velocity   fwd {_f(_mean(idle_vx), 2, ' m/s')}   "
                        f"right {_f(_mean(idle_vy), 2, ' m/s')}  (optical flow)")

    # --- Motors ------------------------------------------------------------
    rule("MOTORS")
    motor_rows = [(r, _motors(r)) for r in armed_rows]
    flying = [(r, m) for r, m in motor_rows if max(m) > HOVER_THROTTLE_FLOOR]
    if not flying:
        add(f"No armed frames with any motor above {HOVER_THROTTLE_FLOOR:.2f} - the mixer never")
        add("left its idle floor, so there is nothing to compare.")
    else:
        columns = list(zip(*[m for _r, m in flying]))
        means = [_mean(column) for column in columns]
        peaks = [max(column) for column in columns]
        overall = _mean(means)

        add(f"{len(flying)} frames above the idle floor. Mean and peak of each motor:")
        add("")
        for index, name in enumerate(MOTOR_NAMES[:len(means)]):
            bar = "#" * int(round(means[index] * 40))
            add(f"  {name:<12} mean {means[index]:5.3f}  peak {peaks[index]:5.3f}  |{bar}")

        spread = (max(means) - min(means)) / overall if overall else 0.0
        add("")
        add(f"spread         {spread * 100:.1f}% of the mean "
            f"(hardest: {MOTOR_NAMES[means.index(max(means))]}, "
            f"easiest: {MOTOR_NAMES[means.index(min(means))]})")
        if spread > MOTOR_SPREAD_WARN:
            add("!! ASYMMETRIC.  At a steady hover these four should sit close together. One")
            add("               motor working consistently harder means the attitude loops are")
            add("               holding a correction all flight, and they only do that against a")
            add("               real force: a weak motor or prop, a bent arm or a damaged blade,")
            add("               an off-centre battery, or an IMU whose idea of level is off.")
            add("               READ IT AS: the corner working hardest is the corner being held")
            add("               UP, so the airframe is falling towards the opposite one. Compare")
            add("               that direction against the drift in the attitude section above -")
            add("               if they agree, this is the cause and no amount of PID tuning")
            add("               fixes it.")
        else:
            add("               Within tolerance - no obvious hardware asymmetry.")

        saturated = sum(1 for _r, m in flying if max(m) >= 0.995)
        if saturated:
            add(f"!! saturation  {saturated} frame(s) with a motor at full. The mixer had no")
            add(f"               authority left ({saturated / len(flying) * 100:.1f}% of flight);")
            add("               no gain change helps while this is happening.")

    # --- Sensors and modes -------------------------------------------------
    rule("SENSORS AND MODES")
    reference = armed_rows or status
    total = len(reference)

    for label, bit, consequence in (
        ("attitude init", STATUS_FLAG_ATTITUDE_INIT, "arming is refused while this is clear"),
        ("altitude valid", STATUS_FLAG_ALT_VALID, "ALT HOLD cannot engage without it"),
        ("velocity valid", STATUS_FLAG_VEL_VALID, "POS HOLD cannot engage without it"),
    ):
        count = sum(1 for r in reference if int(_num(r, "flags")) & bit)
        percent = count / total * 100.0 if total else 0.0
        note = "" if percent > 99.0 else f"   <- {consequence}"
        add(f"{label:<15} {percent:5.1f}% of {'armed' if armed_rows else 'logged'} time{note}")

    modes = {}
    for row in reference:
        mode = int(_num(row, "flight_mode"))
        modes[mode] = modes.get(mode, 0) + 1
    add("modes in effect " + ", ".join(
        f"{MODES[m] if m < len(MODES) else m} {c / total * 100:.0f}%"
        for m, c in sorted(modes.items())))

    if control:
        held = sum(1 for row in control if int(_num(row, "flags")) & CTRL_FLAG_HOLD)
        if held:
            add(f"hold requested  {held / len(control) * 100:.0f}% of frames "
                f"(the dashboard's HOLD toggle)")

        requested = {}
        for row in control:
            mode = int(_num(row, "mode_request"))
            requested[mode] = requested.get(mode, 0) + 1
        add("modes requested " + ", ".join(
            f"{MODES[m] if m < len(MODES) else m} {c / len(control) * 100:.0f}%"
            for m, c in sorted(requested.items())))
        add("               A mode requested but not in effect means the flight controller")
        add("               refused it - its sensor validity flag above was clear.")

    loop_hz = [_num(r, "loop_hz") for r in armed_rows] or [_num(r, "loop_hz") for r in status]
    low, high = _span(loop_hz)
    add(f"loop rate      {_f(low, 0)} .. {_f(high, 0, ' Hz')}  (the inner rate loop; 1000 Hz "
        f"nominal)")

    # --- PID ---------------------------------------------------------------
    if log.pid:
        rule("PID LOOPS   (only the loop whose tab was open is streamed)")
        for name in log.pid_loops():
            samples = log.pid_for(name)
            errors = [_num(s, "sp") - _num(s, "meas") for s in samples]
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            peak = max(abs(e) for e in errors)
            flags = [int(_num(s, "flags")) for s in samples]
            saturated = sum(1 for f in flags if f & 2) / len(flags) * 100.0
            clamped = sum(1 for f in flags if f & 1) / len(flags) * 100.0
            add(f"{str(name):<11} {len(samples):6d} samples   RMS err {rms:8.3f}   peak {peak:8.3f}   "
                f"out sat {saturated:5.1f}%   I clamped {clamped:5.1f}%")
        add("")
        add("out sat above a few percent means that loop is asking for more than it can have.")

    # --- Events ------------------------------------------------------------
    rule("EVENTS")
    if not events:
        add("none recorded")
    else:
        for event in events:
            kind = event.get("kind", "?")
            detail = " ".join(f"{k}={v}" for k, v in event.items()
                              if k not in ("rec", "t", "kind"))
            marker = ">>" if kind == "mark" else "  "
            add(f"{marker} t={_num(event, 't'):8.2f} s  {str(kind):<16} {detail}")

    marks = [e for e in events if e.get("kind") == "mark"]
    if marks:
        rule("MARKERS")
        add("Times you flagged in flight. Look at these first.")
        for mark in marks:
            add(f"  t={_num(mark, 't'):8.2f} s  {mark.get('text', '')}")

    add("")
    add("=" * width)
    return "\n".join(lines)


def _pair(status_rows, control_rows, max_age_s=COMMAND_MAX_AGE_S):
    """Pairs each status frame with the command in force at that moment.

    A merge walk rather than a search per row: both lists are already in time order, so this is
    one pass instead of N log N, and on a ten-minute log that is the difference between instant
    and noticeable.

    A status frame with no command within `max_age_s` before it is DROPPED rather than paired with
    a stale one - see COMMAND_MAX_AGE_S.

    @return (pairs, dropped) - dropped is how many status frames had no fresh command.
    """
    if not control_rows:
        return [], len(status_rows)

    paired = []
    dropped = 0
    index = 0
    latest = None

    for row in status_rows:
        t = _num(row, "t")
        while index < len(control_rows) and _num(control_rows[index], "t") <= t:
            latest = control_rows[index]
            index += 1
        if latest is not None and (t - _num(latest, "t")) <= max_age_s:
            paired.append((row, latest))
        else:
            dropped += 1

    return paired, dropped
