"""Wire protocol shared with the firmware.

This module is the Python half of a contract. Every struct format here mirrors a packed C struct
in ``shared_components/telemetry_uart/include/telemetry_uart.h``, and the C side carries exact
``_Static_assert``s on all four sizes for the same reason the asserts below exist: a padding byte
introduced on either side would still build, still parse, and produce plots that look like noise
rather than like a bug.

If you change a payload, change it in both files in the same commit.
"""

import struct
from collections import namedtuple

# UDP port the telemetry module listens on. We send here; it replies to whatever ephemeral port
# we bound, which it latches from our first packet.
UDP_PORT = 14550
MODULE_ADDRESS = "192.168.4.1"

# --- Message types (telemetry_msg_type_t) ----------------------------------
MSG_IMU = 1
MSG_BATTERY = 2
MSG_STATUS = 3
MSG_CONTROL = 10
MSG_ARMING = 11
MSG_MODE = 12
MSG_GAINS = 13
MSG_PID_DEBUG = 20
MSG_PID_GAINS = 21
MSG_PID_SELECT = 22
MSG_PID_INJECT = 23

# --- Payload formats -------------------------------------------------------
S_DEBUG = struct.Struct("<IBB6f")   # telemetry_pid_debug_payload_t,   30 B
S_GAINS = struct.Struct("<B6f")     # telemetry_pid_gains_payload_t,   25 B
S_SELECT = struct.Struct("<BBH")    # telemetry_pid_select_payload_t,   4 B
S_INJECT = struct.Struct("<BBff")   # telemetry_pid_inject_payload_t,  10 B

# Pre-existing payloads we only read, for the status bar.
S_STATUS = struct.Struct("<12fHBBB")  # telemetry_status_payload_t,    53 B
S_BATTERY = struct.Struct("<2fB")     # telemetry_battery_payload_t,    9 B
S_CONTROL = struct.Struct("<4f3B")    # telemetry_control_payload_t,   19 B

# UDP record header: msg_type(1) | payload_len(1) | seq(2, little-endian).
# No start byte, no CRC, no end byte - UDP already provides framing and a checksum, and
# re-wrapping the UART frame would spend about a fifth of the link on redundant bytes.
S_REC = struct.Struct("<BBH")

assert S_DEBUG.size == 30, f"S_DEBUG is {S_DEBUG.size} B, firmware says 30"
assert S_GAINS.size == 25, f"S_GAINS is {S_GAINS.size} B, firmware says 25"
assert S_SELECT.size == 4, f"S_SELECT is {S_SELECT.size} B, firmware says 4"
assert S_INJECT.size == 10, f"S_INJECT is {S_INJECT.size} B, firmware says 10"
assert S_STATUS.size == 53, f"S_STATUS is {S_STATUS.size} B, firmware says 53"
assert S_REC.size == 4, f"S_REC is {S_REC.size} B, firmware says 4"

# --- Loop ids (pid_loop_id_t) ----------------------------------------------
LOOP_ALL = 0xFF

# Debug flags (PID_FLAG_*).
FLAG_I_CLAMPED = 1 << 0
FLAG_OUT_SAT = 1 << 1
FLAG_MEAS_STALE = 1 << 2   # defined by the protocol, but nothing on this airframe sets it

# --- Control flags (TELEMETRY_CTRL_FLAG_*) ---------------------------------
CTRL_FLAG_KILL = 1 << 0
CTRL_FLAG_HOLD = 1 << 1

# --- Status flags (TELEMETRY_STATUS_FLAG_*) --------------------------------
STATUS_FLAG_LINK_OK = 1 << 0
STATUS_FLAG_ATTITUDE_INIT = 1 << 1
STATUS_FLAG_ALT_VALID = 1 << 2
STATUS_FLAG_VEL_VALID = 1 << 3
STATUS_FLAG_KILLED = 1 << 4

# Saturation / health bits, added 2026-09-01. Sticky over one 20 ms status period on the flight
# controller: set if the condition occurred on any inner-loop tick since the previous frame.
# The payload struct did not change - these are bits 5-7 of the flags byte, which were free - so
# S_STATUS below is untouched and old logs simply have them clear.
STATUS_FLAG_MIX_BOOST_CAP = 1 << 5   # mixer wanted more headroom than MAX_MIXER_THROTTLE_BOOST
STATUS_FLAG_MIX_SCALED = 1 << 6      # attitude command shrunk to fit the motor band
STATUS_FLAG_ACCEL_REJECTED = 1 << 7  # accelerometer gated out of the fusion (vibration)

STATUS_FLAG_NAMES = (
    (STATUS_FLAG_LINK_OK, "LINK_OK"),
    (STATUS_FLAG_ATTITUDE_INIT, "ATTITUDE_INIT"),
    (STATUS_FLAG_ALT_VALID, "ALT_VALID"),
    (STATUS_FLAG_VEL_VALID, "VEL_VALID"),
    (STATUS_FLAG_KILLED, "KILLED"),
    (STATUS_FLAG_MIX_BOOST_CAP, "MIX_BOOST_CAP"),
    (STATUS_FLAG_MIX_SCALED, "MIX_SCALED"),
    (STATUS_FLAG_ACCEL_REJECTED, "ACCEL_REJECTED"),
)


def status_flag_names(flags):
    """Decode a status flags byte into a list of set bit names, for logs and reports."""
    return [name for bit, name in STATUS_FLAG_NAMES if flags & bit]


Loop = namedtuple("Loop", "id name block sp_unit out_unit rate divider")

# The eight loops, in pid_loop_id_t order. `divider` is what gets sent in a PID_SELECT when this
# loop's tab is opened: 1 for everything slow enough to stream whole, 2 for the three 1 kHz rate
# loops so they arrive at 500 Hz. See the bandwidth section of the README for why.
LOOPS = [
    Loop(0, "alt",        "Altitude PID", "m/s",   "throttle", 50,   1),
    Loop(1, "vel_x",      "Velocity PIDs", "m/s",  "deg",      50,   1),
    Loop(2, "vel_y",      "Velocity PIDs", "m/s",  "deg",      50,   1),
    Loop(3, "ang_roll",   "Angle PIDs",   "deg",   "deg/s",    250,  1),
    Loop(4, "ang_pitch",  "Angle PIDs",   "deg",   "deg/s",    250,  1),
    Loop(5, "rate_roll",  "Rate PIDs",    "deg/s", "cmd",      1000, 2),
    Loop(6, "rate_pitch", "Rate PIDs",    "deg/s", "cmd",      1000, 2),
    Loop(7, "rate_yaw",   "Rate PIDs",    "deg/s", "cmd",      1000, 2),
]

LOOP_COUNT = len(LOOPS)
assert LOOP_COUNT == 8, "pid_loop_id_t has 8 entries terminated by PID_LOOP_COUNT"

# Divider used by the overview tab. The loops tick at different rates, so ONE divider cannot give
# a uniform per-loop sample rate - see the README. 20 gives 50 Hz on the rate loops, 12.5 Hz on
# the angle loops and 2.5 Hz on altitude and velocity.
OVERVIEW_DIVIDER = 20

INJECT_MODES = [("off", 0), ("step", 1), ("doublet", 2), ("square", 3)]


def loop_by_id(loop_id):
    """Returns the Loop tuple for a pid_loop_id_t value, or None if out of range."""
    if 0 <= loop_id < LOOP_COUNT:
        return LOOPS[loop_id]
    return None


def encode_record(msg_type, payload, seq=0):
    """Wraps one payload in a UDP record header. Datagrams are these concatenated."""
    return S_REC.pack(msg_type, len(payload), seq) + payload


def split_records(data):
    """Splits one datagram into (msg_type, seq, payload) tuples.

    Stops at the first truncated record rather than guessing where the next one starts - a short
    datagram means the sender was cut off, not that the remainder is parseable.
    """
    records = []
    offset = 0
    length = len(data)

    while offset + S_REC.size <= length:
        msg_type, payload_len, seq = S_REC.unpack_from(data, offset)
        offset += S_REC.size
        if offset + payload_len > length:
            break
        records.append((msg_type, seq, data[offset:offset + payload_len]))
        offset += payload_len

    return records


def pack_select(loop_id, divider):
    """PID_SELECT payload: subscribe the debug stream to one loop (or LOOP_ALL)."""
    return S_SELECT.pack(loop_id & 0xFF, divider & 0xFF, 0)


def pack_inject(loop_id, mode, amplitude, period_s):
    """PID_INJECT payload: arm or cancel a test signal on one loop."""
    return S_INJECT.pack(loop_id & 0xFF, mode & 0xFF, float(amplitude), float(period_s))


def pack_gains(loop_id, kp, ki, kd, i_limit, out_limit, d_cutoff_hz):
    """PID_GAINS payload used as a SET."""
    return S_GAINS.pack(loop_id & 0xFF, float(kp), float(ki), float(kd),
                        float(i_limit), float(out_limit), float(d_cutoff_hz))


def pack_gains_read_request(loop_id):
    """PID_GAINS payload used as a READ.

    All six floats exactly zero means "report what you have" rather than "set everything to
    zero". An all-zero gain set would disable the loop entirely and is never something anyone
    means, so the overload is unambiguous and saves a message type.
    """
    return S_GAINS.pack(loop_id & 0xFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def unpack_gains(payload):
    """Returns a dict of one loop's gains, or None if the payload is the wrong size."""
    if len(payload) < S_GAINS.size:
        return None
    loop_id, kp, ki, kd, i_limit, out_limit, d_cutoff_hz = S_GAINS.unpack_from(payload)
    return {
        "loop_id": loop_id,
        "kp": kp, "ki": ki, "kd": kd,
        "i_limit": i_limit, "out_limit": out_limit, "d_cutoff_hz": d_cutoff_hz,
    }


def unpack_status(payload):
    """Returns a dict of the flight controller's status frame, or None on a short payload."""
    if len(payload) < S_STATUS.size:
        return None
    values = S_STATUS.unpack_from(payload)
    return {
        "roll": values[0], "pitch": values[1], "yaw": values[2],
        "altitude": values[3], "climb_rate": values[4],
        "velocity_x": values[5], "velocity_y": values[6],
        "battery_voltage": values[7],
        "motor": list(values[8:12]),
        "loop_hz": values[12],
        "armed": values[13], "flight_mode": values[14], "flags": values[15],
    }


def unpack_control(payload):
    """Returns a dict of a control frame, or None on a short payload.

    THIS IS A DOWNLINK USE OF AN UPLINK MESSAGE, and that is deliberate rather than sloppy.
    TELEMETRY_MSG_CONTROL is what the telemetry module sends the flight controller at 50 Hz. The
    module now also echoes every one of those frames to the ground station (see uart_link.c), so
    a flight log can show what the PILOT ASKED FOR next to what the drone actually did - the
    single most useful pair of traces there is when working out why a flight went wrong.

    Direction disambiguates the reuse completely: a CONTROL record arriving here is an echo, and
    a CONTROL record we send is a command. Nothing has to look at anything but which way it went.

    Older module firmware simply never sends these, in which case the flight log has no control
    trace and everything else still works.
    """
    if len(payload) < S_CONTROL.size:
        return None
    roll, pitch, yaw, throttle, armed, flight_mode, flags = S_CONTROL.unpack_from(payload)
    return {
        "roll_sp": roll,          # commanded roll angle, degrees
        "pitch_sp": pitch,        # commanded pitch angle, degrees
        "yaw_rate_sp": yaw,       # commanded yaw RATE, deg/s - yaw has no angle loop
        "throttle": throttle,     # 0.0 - 1.0 trim
        "armed_request": armed,   # what the pilot asked for, NOT whether the motors are live
        "mode_request": flight_mode,
        "flags": flags,
    }


def unpack_battery(payload):
    """Returns a dict of a battery frame, or None on a short payload."""
    if len(payload) < S_BATTERY.size:
        return None
    voltage, current, percent = S_BATTERY.unpack_from(payload)
    return {"battery_voltage": voltage, "battery_current": current, "battery_percent": percent}


def pack_control(roll, pitch, yaw, throttle, armed, flight_mode, flags):
    """TELEMETRY_MSG_CONTROL payload.

    Read the arming caveat in the README before using this for anything but KILL: the telemetry
    module sends its own control frame at 50 Hz and the flight controller acts on whichever
    arrived last, so an arm request from here is overwritten within 20 ms. KILL is the exception
    because the flight controller LATCHES it.
    """
    return S_CONTROL.pack(float(roll), float(pitch), float(yaw), float(throttle),
                          armed & 0xFF, flight_mode & 0xFF, flags & 0xFF)
