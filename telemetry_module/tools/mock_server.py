#!/usr/bin/env python3
"""Mock telemetry module for dashboard development.

Serves the real components/http_server/www/index.html straight off disk and implements every
endpoint the firmware exposes, so the dashboard can be developed and tested without flashing
an ESP32 or having a flight controller attached.

It is not just a stub: it reimplements the parts of the firmware whose behaviour the dashboard
actually reacts to.

  * control_state.c   - the ramp-on-hold / decay-on-release setpoint integration, the throttle
                        trim with no decay, the 500 ms input watchdog and the 3 s disarm
                        backstop. Held buttons therefore feel the same here as on the drone.
  * flight_control.c  - the arming gate (attitude must have settled) and the latching kill,
                        which only clears on an explicit disarm.
  * the flight controller itself - a crude but plausible rigid-body sim, so roll/pitch/yaw,
                        altitude, motors and battery move in response to what you press.

Deliberately stdlib-only: no venv, no pip, no requirements file to keep in sync.

    python3 telemetry_module/tools/mock_server.py
    -> http://localhost:8080

Runtime fault injection, for testing the dashboard's unhappy paths without a restart. Open
these in a second tab, or curl them:

    /mock            - what the simulator is doing right now, as JSON
    /mock/link       - toggle the flight controller link (drives the NO LINK banner)
    /mock/camera     - toggle the camera (capture returns an error, preview stops)
    /mock/sd         - toggle the SD card (captures start failing)
    /mock/reset      - back to a clean boot

The viewfinder is served as a synthetic PNG horizon that tilts with the simulated attitude,
under the /preview.jpg path the firmware uses. The extension is a lie but the Content-Type is
honest, which is all the browser cares about - and it means the preview visibly responds to
the controls instead of being a still image.

LAYOUT OF THIS FILE
    constants mirrored from telemetry_uart.h and control_state.c   (marked with comments)
    ramp_towards / update_axis  - line-for-line copies of the firmware helpers
    DroneSim                    - setpoint integration, watchdogs, arming, the rigid-body sim
    png_encode / render_preview - the synthetic viewfinder, stdlib zlib only
    MockHandler                 - every /api/* endpoint plus the /mock/* fault injection
    sim_thread / main           - a 50 Hz sim tick, matching the firmware's UART TX cadence

THIS FILE IS A MIRROR, AND MIRRORS ROT.
    The constants and the integration math above are duplicated from the firmware, not shared
    with it. If you change a ramp rate, an axis limit, a watchdog timeout, the arming gate or
    the shape of an /api/* response in components/, change it HERE IN THE SAME COMMIT.
    Otherwise the dashboard behaves differently against the mock than against the real board,
    and you will spend an afternoon debugging the difference instead of the bug.
"""

import argparse
import json
import math
import os
import random
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --- Mirrored from shared_components/telemetry_uart/include/telemetry_uart.h ----------------
MODE_ANGLE = 0
MODE_ALT_HOLD = 1
MODE_POS_HOLD = 2
LOOP_COUNT = 8

LOOP_NAMES = [
    "rate_roll", "rate_pitch", "rate_yaw",
    "angle_roll", "angle_pitch",
    "vel_x", "vel_y", "altitude",
]

# --- Mirrored from components/control_state/control_state.c --------------------------------
MAX_ROLL_PITCH_DEG = 15.0
MAX_YAW_RATE_DPS = 90.0
ROLL_PITCH_RAMP_DPS = 30.0
ROLL_PITCH_DECAY_DPS = 60.0
YAW_RAMP_DPS2 = 180.0
YAW_DECAY_DPS2 = 360.0
THROTTLE_TRIM_RATE_PER_S = 0.25
THROTTLE_MAX = 0.85
INPUT_TIMEOUT_S = 0.5
DISARM_TIMEOUT_S = 3.0

# --- Simulator constants --------------------------------------------------------------------
SIM_HZ = 50.0                 # matches the firmware's UART TX task rate
ATTITUDE_SETTLE_S = 2.5       # how long the complementary filter takes to declare itself ready
HOVER_THROTTLE = 0.42         # trim above which the sim drone climbs
TOF_MAX_RANGE_M = 3.6         # VL53L1X long-range ceiling; above this alt_valid drops
FLOW_MIN_HEIGHT_M = 0.06      # optical flow needs some height before it means anything
BATTERY_FULL_V = 4.12
BATTERY_EMPTY_V = 3.30

BUTTON_KEYS = (
    "pitch_forward", "pitch_back",
    "roll_left", "roll_right",
    "yaw_left", "yaw_right",
    "throttle_up", "throttle_down",
)


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def ramp_towards(value, target, rate, dt):
    """Move `value` towards `target` at `rate` units/second without overshooting."""
    step = rate * dt
    if value < target:
        value = min(value + step, target)
    elif value > target:
        value = max(value - step, target)
    return value


def update_axis(current, positive_held, negative_held, limit, ramp_rate, decay_rate, dt):
    # Both held (or neither) means no commanded direction: decay to neutral.
    if positive_held == negative_held:
        return ramp_towards(current, 0.0, decay_rate, dt)
    target = limit if positive_held else -limit
    return ramp_towards(current, target, ramp_rate, dt)


class DroneSim:
    """Pilot-side control state plus a toy flight model, stepped at SIM_HZ on its own thread."""

    def __init__(self, camera_ok=True, sd_ok=True):
        self.lock = threading.RLock()
        self.boot_camera_ok = camera_ok
        self.boot_sd_ok = sd_ok
        self.reset()

    def reset(self):
        with self.lock:
            self.t0 = time.monotonic()

            # Pilot request state (control_state.c)
            self.buttons = {key: False for key in BUTTON_KEYS}
            self.roll_sp = 0.0
            self.pitch_sp = 0.0
            self.yaw_rate_sp = 0.0
            self.throttle_trim = 0.0
            self.arm_request = False
            self.kill_request = False
            self.hold_request = False
            self.flight_mode = MODE_ANGLE
            self.last_input_t = 0.0
            self.inputs_released = True

            # Flight controller state
            self.armed = False
            self.killed = False
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.altitude = 0.0
            self.climb_rate = 0.0
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.motors = [0.0, 0.0, 0.0, 0.0]
            self.battery = BATTERY_FULL_V
            self.loop_hz = 1000
            self.gains = {i: {"kp": 0.0, "ki": 0.0, "kd": 0.0} for i in range(LOOP_COUNT)}

            # Peripherals / link
            self.link_up = True
            self.camera_ok = self.boot_camera_ok
            self.sd_ok = self.boot_sd_ok
            self.preview_on = False
            self.preview_frames = 0
            self.capture_flash_until = 0.0
            self.shots_requested = 0
            self.shots_written = 0
            self.shots_failed = 0
            self.rx_frames = 0
            self.tx_frames = 0
            self.clients = 1
            self.last_status_t = time.monotonic()

    # --- Endpoint-facing setters (one per firmware control_state_set_* function) -----------

    def set_buttons(self, buttons):
        with self.lock:
            for key in BUTTON_KEYS:
                self.buttons[key] = bool(buttons.get(key, False))
            self.last_input_t = time.monotonic()
            self.inputs_released = False

    def set_arm(self, armed):
        with self.lock:
            self.arm_request = bool(armed)
            # An explicit disarm is what releases the kill latch, matching the flight
            # controller: kill must be low AND arm must be low before it will let go.
            if not armed:
                self.kill_request = False
                self.killed = False
            if armed:
                self.throttle_trim = 0.0
            self.last_input_t = time.monotonic()

    def set_kill(self):
        with self.lock:
            self.kill_request = True
            self.killed = True
            self.arm_request = False
            self.armed = False
            self.throttle_trim = 0.0
            self.roll_sp = self.pitch_sp = self.yaw_rate_sp = 0.0
            self.last_input_t = time.monotonic()

    def set_mode(self, hold, mode):
        with self.lock:
            self.hold_request = bool(hold)
            if mode is not None and 0 <= mode <= MODE_POS_HOLD:
                self.flight_mode = int(mode)
            self.last_input_t = time.monotonic()

    def set_gains(self, loop_id, kp, ki, kd):
        with self.lock:
            self.gains[loop_id] = {"kp": kp, "ki": ki, "kd": kd}

    def request_capture(self):
        """Returns (ok, error_string). Mirrors camera_sd_request_capture()."""
        with self.lock:
            if not self.camera_ok:
                return False, "camera or sd unavailable"
            self.shots_requested += 1
            self.capture_flash_until = time.monotonic() + 0.25
            # The real firmware writes on another core; the failure shows up later, so decide
            # it here but let the counters reflect an SD that is present but unhappy.
            if not self.sd_ok:
                self.shots_failed += 1
            elif random.random() < 0.02:
                self.shots_failed += 1     # occasional write failure, so the UI path gets exercised
            else:
                self.shots_written += 1
            return True, None

    def set_preview(self, enable):
        with self.lock:
            if not self.camera_ok:
                return False
            self.preview_on = bool(enable)
            if not enable:
                self.preview_frames = 0
            return True

    # --- Simulation step -------------------------------------------------------------------

    def step(self, dt):
        with self.lock:
            now = time.monotonic()
            uptime = now - self.t0
            attitude_init = uptime > ATTITUDE_SETTLE_S

            # --- Browser watchdog (control_state_update) --------------------------------
            input_age = now - self.last_input_t if self.last_input_t else 1e9
            if input_age > INPUT_TIMEOUT_S:
                if not self.inputs_released:
                    print("[sim] browser stopped posting - releasing directional inputs")
                    self.inputs_released = True
                for key in BUTTON_KEYS:
                    self.buttons[key] = False
                if self.last_input_t and input_age > DISARM_TIMEOUT_S and self.arm_request:
                    print("[sim] browser gone for >3 s - dropping arm request")
                    self.arm_request = False
                    self.throttle_trim = 0.0

            # --- Setpoint integration ----------------------------------------------------
            self.roll_sp = update_axis(self.roll_sp,
                                       self.buttons["roll_right"], self.buttons["roll_left"],
                                       MAX_ROLL_PITCH_DEG, ROLL_PITCH_RAMP_DPS,
                                       ROLL_PITCH_DECAY_DPS, dt)
            self.pitch_sp = update_axis(self.pitch_sp,
                                        self.buttons["pitch_forward"], self.buttons["pitch_back"],
                                        MAX_ROLL_PITCH_DEG, ROLL_PITCH_RAMP_DPS,
                                        ROLL_PITCH_DECAY_DPS, dt)
            self.yaw_rate_sp = update_axis(self.yaw_rate_sp,
                                           self.buttons["yaw_right"], self.buttons["yaw_left"],
                                           MAX_YAW_RATE_DPS, YAW_RAMP_DPS2, YAW_DECAY_DPS2, dt)

            # Throttle trim is persistent - no decay branch, same as the firmware.
            if self.buttons["throttle_up"] and not self.buttons["throttle_down"]:
                self.throttle_trim += THROTTLE_TRIM_RATE_PER_S * dt
            elif self.buttons["throttle_down"] and not self.buttons["throttle_up"]:
                self.throttle_trim -= THROTTLE_TRIM_RATE_PER_S * dt
            self.throttle_trim = clamp(self.throttle_trim, 0.0, THROTTLE_MAX)
            if not self.arm_request:
                self.throttle_trim = 0.0

            # --- Arming gate ---------------------------------------------------------------
            if self.killed:
                self.armed = False
            else:
                self.armed = self.arm_request and attitude_init

            # --- Attitude: first-order lag towards the setpoint, plus a little noise -------
            tau = 0.15
            alpha = clamp(dt / tau, 0.0, 1.0)
            noise = 0.15 if self.armed else 0.02
            self.roll += (self.roll_sp - self.roll) * alpha + random.gauss(0.0, noise)
            self.pitch += (self.pitch_sp - self.pitch) * alpha + random.gauss(0.0, noise)
            if self.armed:
                self.yaw += self.yaw_rate_sp * dt
            self.yaw = (self.yaw + 180.0) % 360.0 - 180.0

            # --- Altitude ------------------------------------------------------------------
            if self.armed:
                # Net vertical acceleration from throttle above hover, with drag.
                accel = (self.throttle_trim - HOVER_THROTTLE) * 12.0 - self.climb_rate * 1.8
                self.climb_rate += accel * dt
                self.altitude += self.climb_rate * dt
                if self.altitude <= 0.0:
                    self.altitude = 0.0
                    self.climb_rate = max(0.0, self.climb_rate)
            else:
                # Disarmed: fall to the ground quickly and sit there.
                self.altitude = max(0.0, self.altitude - 2.0 * dt)
                self.climb_rate = 0.0 if self.altitude == 0.0 else -2.0

            # --- Body velocity from tilt ----------------------------------------------------
            if self.armed and self.altitude > FLOW_MIN_HEIGHT_M:
                self.velocity_x += (math.radians(self.pitch) * 9.81 - self.velocity_x * 1.2) * dt
                self.velocity_y += (math.radians(self.roll) * 9.81 - self.velocity_y * 1.2) * dt
            else:
                self.velocity_x *= max(0.0, 1.0 - 4.0 * dt)
                self.velocity_y *= max(0.0, 1.0 - 4.0 * dt)

            # --- Motors: X mixer on top of the throttle -------------------------------------
            if self.armed:
                roll_cmd = self.roll_sp / MAX_ROLL_PITCH_DEG * 0.12
                pitch_cmd = self.pitch_sp / MAX_ROLL_PITCH_DEG * 0.12
                yaw_cmd = self.yaw_rate_sp / MAX_YAW_RATE_DPS * 0.06
                base = self.throttle_trim
                self.motors = [
                    clamp(base - roll_cmd - pitch_cmd + yaw_cmd, 0.0, 1.0),
                    clamp(base + roll_cmd - pitch_cmd - yaw_cmd, 0.0, 1.0),
                    clamp(base + roll_cmd + pitch_cmd + yaw_cmd, 0.0, 1.0),
                    clamp(base - roll_cmd + pitch_cmd - yaw_cmd, 0.0, 1.0),
                ]
            else:
                self.motors = [0.0, 0.0, 0.0, 0.0]

            # --- Battery: sags under load, drains slowly ------------------------------------
            load = sum(self.motors) / 4.0
            self.battery -= (0.0006 + load * 0.004) * dt
            self.battery = max(BATTERY_EMPTY_V, self.battery)

            self.loop_hz = int(random.gauss(1000, 6)) if self.armed else int(random.gauss(1000, 3))

            # --- Link bookkeeping -------------------------------------------------------------
            if self.link_up:
                self.last_status_t = now
                self.rx_frames += 1
                self.tx_frames += 1

            self._attitude_init = attitude_init

    # --- Readback ---------------------------------------------------------------------------

    def status_json(self):
        """Byte-for-byte the shape of handler_status() in http_server.c."""
        with self.lock:
            now = time.monotonic()
            age_ms = int((now - self.last_status_t) * 1000)
            link = self.link_up and age_ms < 500
            alt_valid = link and self.altitude < TOF_MAX_RANGE_M
            vel_valid = link and self.altitude > FLOW_MIN_HEIGHT_M

            return {
                "link": link,
                "armed": link and self.armed,
                "killed": self.killed,
                "attitude_init": getattr(self, "_attitude_init", False),
                "alt_valid": alt_valid,
                "vel_valid": vel_valid,
                "mode": self.flight_mode,
                "roll": round(self.roll, 2),
                "pitch": round(self.pitch, 2),
                "yaw": round(self.yaw, 2),
                "altitude": round(self.altitude, 3),
                "climb": round(self.climb_rate, 3),
                "vx": round(self.velocity_x, 3),
                "vy": round(self.velocity_y, 3),
                "battery": round(self.battery, 2),
                "throttle_trim": round(self.throttle_trim, 3),
                "loop_hz": self.loop_hz,
                "motors": [round(m, 3) for m in self.motors],
                "status_age_ms": age_ms,
                "clients": self.clients,
                "shots_requested": self.shots_requested,
                "shots_written": self.shots_written,
                "shots_failed": self.shots_failed,
                "camera_ok": self.camera_ok,
                "sd_ok": self.sd_ok,
                "rx_frames": self.rx_frames,
                "tx_frames": self.tx_frames,
            }

    def debug_json(self):
        with self.lock:
            return {
                "uptime_s": round(time.monotonic() - self.t0, 1),
                "buttons": dict(self.buttons),
                "setpoints": {
                    "roll": round(self.roll_sp, 2),
                    "pitch": round(self.pitch_sp, 2),
                    "yaw_rate": round(self.yaw_rate_sp, 2),
                    "throttle": round(self.throttle_trim, 3),
                },
                "arm_request": self.arm_request,
                "kill_request": self.kill_request,
                "hold_request": self.hold_request,
                "link_up": self.link_up,
                "preview_on": self.preview_on,
                "preview_frames": self.preview_frames,
                "gains": {LOOP_NAMES[i]: self.gains[i] for i in range(LOOP_COUNT)},
            }


# --- Synthetic viewfinder ----------------------------------------------------------------------

def png_encode(width, height, rgb_rows):
    """Minimal PNG writer. Avoids a Pillow dependency for what is a few hundred lines of pixels."""
    raw = bytearray()
    for row in rgb_rows:
        raw.append(0)          # filter type 0 (None)
        raw.extend(row)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", header) +
            chunk(b"IDAT", zlib.compress(bytes(raw), 6)) +
            chunk(b"IEND", b""))


def render_preview(sim, width=160, height=120):
    """An artificial horizon that banks with roll and rises with pitch, plus a yaw-scrolling
    ground texture. Gives the viewfinder something that visibly answers the controls."""
    with sim.lock:
        roll = math.radians(clamp(sim.roll, -60.0, 60.0))
        pitch = sim.pitch
        yaw = sim.yaw
        flash = time.monotonic() < sim.capture_flash_until

    slope = math.tan(roll)
    center_x = width / 2.0
    horizon_center = height / 2.0 + pitch * 2.0
    scroll = int(yaw * 2.0) % 24

    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            horizon_y = horizon_center + (x - center_x) * slope
            distance = y - horizon_y

            if abs(distance) < 1.2:
                pixel = (235, 235, 235)                    # horizon line
            elif distance < 0:
                shade = clamp(0.55 + (y / height) * 0.5, 0.0, 1.0)
                pixel = (int(60 * shade), int(120 * shade), int(200 * shade))
            else:
                stripe = ((x + scroll) // 12) % 2
                shade = clamp(0.55 + distance / height * 0.5, 0.0, 1.1)
                base = (110, 84, 58) if stripe else (86, 66, 46)
                pixel = tuple(int(clamp(c * shade, 0, 255)) for c in base)

            # Centre crosshair.
            if (abs(y - height // 2) < 1 and 60 < x < 100) or \
               (abs(x - width // 2) < 1 and 50 < y < 70):
                pixel = (255, 200, 40)

            if flash:
                pixel = tuple(min(255, c + 120) for c in pixel)

            row.extend(pixel)
        rows.append(row)

    return png_encode(width, height, rows)


# --- HTTP --------------------------------------------------------------------------------------

class MockHandler(BaseHTTPRequestHandler):
    server_version = "MockTelemetry/1.0"
    protocol_version = "HTTP/1.1"

    sim = None            # set by main()
    html_path = None
    verbose = False

    def log_message(self, fmt, *args):
        # The dashboard posts at 20 Hz and polls status at 5 Hz. Logging every request buries
        # anything useful, so stay quiet unless asked.
        if self.verbose:
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # --- helpers -------------------------------------------------------------------------

    def _send(self, body, content_type, status=200, no_store=False):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        if no_store:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload, status=200):
        self._send(json.dumps(payload), "application/json", status, no_store=True)

    def _read_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            parsed = json.loads(raw.decode("utf-8"))
            return parsed if isinstance(parsed, dict) else {}
        except (ValueError, UnicodeDecodeError):
            return {}

    def _path(self):
        return self.path.split("?", 1)[0]

    # --- GET -----------------------------------------------------------------------------

    def do_GET(self):
        path = self._path()

        if path == "/":
            try:
                with open(self.html_path, "rb") as handle:
                    html = handle.read()
            except OSError as error:
                self._send("Cannot read %s: %s" % (self.html_path, error), "text/plain", 500)
                return
            # Read fresh on every request, exactly like the no-store the firmware sends: edit
            # index.html, hit reload, done.
            self._send(html, "text/html", no_store=True)
            return

        if path == "/api/status":
            self._send_json(self.sim.status_json())
            return

        if path == "/preview.jpg":
            with self.sim.lock:
                serve = self.sim.preview_on and self.sim.camera_ok
            if not serve:
                self._send_json({"ok": False, "error": "no preview frame"}, 503)
                return
            frame = render_preview(self.sim)
            with self.sim.lock:
                self.sim.preview_frames += 1
            # Served as PNG under a .jpg path. The browser goes by Content-Type; generating a
            # real JPEG would mean hand-rolling an encoder for no visible difference.
            self._send(frame, "image/png", no_store=True)
            return

        # --- mock-only fault injection ---
        if path == "/mock":
            self._send_json(self.sim.debug_json())
            return

        if path == "/mock/link":
            with self.sim.lock:
                self.sim.link_up = not self.sim.link_up
                state = self.sim.link_up
                if state:
                    self.sim.last_status_t = time.monotonic()
            print("[mock] link %s" % ("UP" if state else "DOWN"))
            self._send_json({"link_up": state})
            return

        if path == "/mock/camera":
            with self.sim.lock:
                self.sim.camera_ok = not self.sim.camera_ok
                state = self.sim.camera_ok
                if not state:
                    self.sim.preview_on = False
            print("[mock] camera %s" % ("OK" if state else "FAILED"))
            self._send_json({"camera_ok": state})
            return

        if path == "/mock/sd":
            with self.sim.lock:
                self.sim.sd_ok = not self.sim.sd_ok
                state = self.sim.sd_ok
            print("[mock] sd %s" % ("OK" if state else "FAILED"))
            self._send_json({"sd_ok": state})
            return

        if path == "/mock/reset":
            self.sim.reset()
            print("[mock] simulator reset")
            self._send_json({"ok": True})
            return

        self._send_json({"ok": False, "error": "not found"}, 404)

    # --- POST ----------------------------------------------------------------------------

    def do_POST(self):
        path = self._path()
        body = self._read_body()

        if path == "/api/input":
            self.sim.set_buttons(body)
            self._send_json({"ok": True})
            return

        if path == "/api/arm":
            if body.get("kill") is True:
                self.sim.set_kill()
                print("[mock] KILL from dashboard")
            else:
                armed = body.get("armed") is True
                self.sim.set_arm(armed)
                print("[mock] arm request: %s" % ("ARM" if armed else "DISARM"))
            self._send_json({"ok": True})
            return

        if path == "/api/mode":
            mode = body.get("mode")
            self.sim.set_mode(body.get("hold") is True,
                              int(mode) if isinstance(mode, (int, float)) else None)
            self._send_json({"ok": True})
            return

        if path == "/api/capture":
            ok, error = self.sim.request_capture()
            if not ok:
                self._send_json({"ok": False, "error": error})
                return
            self._send_json({"ok": True, "queued": True})
            return

        if path == "/api/gains":
            loop = body.get("loop")
            if not isinstance(loop, (int, float)) or not 0 <= int(loop) < LOOP_COUNT:
                self._send("bad loop id", "text/plain", 400)
                return
            loop = int(loop)
            kp = float(body.get("kp", 0.0))
            ki = float(body.get("ki", 0.0))
            kd = float(body.get("kd", 0.0))
            self.sim.set_gains(loop, kp, ki, kd)
            print("[mock] gains %-12s kp=%.4f ki=%.4f kd=%.4f" % (LOOP_NAMES[loop], kp, ki, kd))
            self._send_json({"ok": True})
            return

        if path == "/api/preview":
            enable = body.get("enable") is True
            if not self.sim.set_preview(enable):
                self._send_json({"ok": False, "error": "camera unavailable"})
                return
            self._send_json({"ok": True, "preview": enable})
            return

        self._send_json({"ok": False, "error": "not found"}, 404)


def sim_thread(sim):
    """Fixed-rate stepping, matching the 50 Hz UART TX task the real control state runs on."""
    period = 1.0 / SIM_HZ
    previous = time.monotonic()
    while True:
        time.sleep(period)
        now = time.monotonic()
        dt = now - previous
        previous = now
        sim.step(min(dt, 0.2))     # cap dt so a suspended laptop does not teleport the drone


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_html = os.path.normpath(
        os.path.join(here, "..", "components", "http_server", "www", "index.html"))

    parser = argparse.ArgumentParser(description="Mock telemetry module for dashboard testing.")
    parser.add_argument("--port", type=int, default=8080, help="listen port (default 8080)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="bind address (default 0.0.0.0, so a phone on the LAN can reach it)")
    parser.add_argument("--html", default=default_html,
                        help="dashboard file to serve (default: the firmware's index.html)")
    parser.add_argument("--no-camera", action="store_true", help="boot with the camera failed")
    parser.add_argument("--no-sd", action="store_true", help="boot with no SD card")
    parser.add_argument("--verbose", action="store_true", help="log every HTTP request")
    args = parser.parse_args()

    if not os.path.isfile(args.html):
        parser.error("dashboard not found: %s" % args.html)

    sim = DroneSim(camera_ok=not args.no_camera, sd_ok=not args.no_sd)
    threading.Thread(target=sim_thread, args=(sim,), daemon=True).start()

    MockHandler.sim = sim
    MockHandler.html_path = args.html
    MockHandler.verbose = args.verbose

    server = ThreadingHTTPServer((args.host, args.port), MockHandler)
    server.daemon_threads = True

    print("mock telemetry module")
    print("  dashboard : %s" % args.html)
    print("  listening : http://localhost:%d" % args.port)
    print("  faults    : /mock  /mock/link  /mock/camera  /mock/sd  /mock/reset")
    print("  note      : attitude reports 'initialising' for the first %.1f s, as on the drone"
          % ATTITUDE_SETTLE_S)
    print("Ctrl-C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
        server.shutdown()


if __name__ == "__main__":
    main()
