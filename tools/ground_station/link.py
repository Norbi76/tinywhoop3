"""UDP transport, receive thread and per-loop sample storage.

Everything in here runs off the GUI thread. The one rule that keeps that safe: the receive thread
only ever writes into the numpy ring buffers and emits Qt signals; it never touches a widget. All
drawing happens on the GUI thread, driven by the single timer in gs.py.
"""

import socket
import threading
import time

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

import protocol as P

# Columns of the ring buffer, in order.
COL_T, COL_SP, COL_MEAS, COL_P, COL_I, COL_D, COL_OUT, COL_FLAGS = range(8)
RING_COLUMNS = 8

# 6000 samples is 12 s at 500 Hz and 2 minutes at 50 Hz - comfortably more than the longest
# rolling window the UI offers, so the window length is never limited by storage.
RING_CAPACITY = 6000

# The firmware's timestamp is esp_timer_get_time() truncated to 32 bits, so it wraps every
# 2**32 microseconds - about 71.6 minutes. Long enough to forget about, short enough to hit
# during a tuning session.
US_WRAP = 1 << 32

# Considered offline after this long without a datagram.
LINK_TIMEOUT_S = 1.5


class Ring:
    """Fixed-size ring buffer of PID samples, backed by one preallocated numpy array.

    Deliberately not a deque of dicts or a list of tuples: at 500 Hz either of those allocates
    tens of thousands of short-lived objects per minute and the resulting GC pauses show up as
    visible stutter in the plots. This allocates once.
    """

    def __init__(self, capacity=RING_CAPACITY):
        self.capacity = capacity
        self.data = np.zeros((capacity, RING_COLUMNS), dtype=np.float64)
        self.head = 0          # next row to write
        self.count = 0         # rows valid so far, saturating at capacity
        self.anchor_us = None  # first raw timestamp seen, the origin for relative time
        self.lock = threading.Lock()

    def push(self, t_us, setpoint, measurement, p_term, i_term, d_term, output, flags):
        """Appends one sample. Called from the receive thread."""
        with self.lock:
            if self.anchor_us is None:
                self.anchor_us = t_us

            # Masked subtraction, so the difference stays correct across the 32-bit wrap. Anchor
            # per loop rather than globally because loops start streaming at different moments.
            elapsed_s = ((t_us - self.anchor_us) & (US_WRAP - 1)) / 1e6

            row = self.data[self.head]
            row[COL_T] = elapsed_s
            row[COL_SP] = setpoint
            row[COL_MEAS] = measurement
            row[COL_P] = p_term
            row[COL_I] = i_term
            row[COL_D] = d_term
            row[COL_OUT] = output
            row[COL_FLAGS] = flags

            self.head = (self.head + 1) % self.capacity
            if self.count < self.capacity:
                self.count += 1

    def snapshot(self, window_s=None):
        """Returns a chronologically ordered copy of the buffer.

        @param window_s Keep only the last N seconds, or None for everything held.
        @return An (n, 8) float64 array, possibly empty.
        """
        with self.lock:
            if self.count == 0:
                return np.empty((0, RING_COLUMNS), dtype=np.float64)

            if self.count < self.capacity:
                out = self.data[:self.count].copy()
            else:
                # Unwrap: the oldest row is at head once the buffer has filled.
                out = np.concatenate((self.data[self.head:], self.data[:self.head]))

        if window_s is not None and out.shape[0] > 0:
            cutoff = out[-1, COL_T] - window_s
            out = out[out[:, COL_T] >= cutoff]

        return out

    def clear(self):
        """Drops everything and re-anchors on the next sample."""
        with self.lock:
            self.head = 0
            self.count = 0
            self.anchor_us = None


class Link(QObject):
    """The UDP link to the telemetry module.

    Owns one socket, a receive thread and a keepalive thread. Both are daemons, so closing the
    window does not need an explicit shutdown handshake.
    """

    status_text = pyqtSignal(str)
    gains_received = pyqtSignal(object)   # dict from protocol.unpack_gains

    def __init__(self, host=P.MODULE_ADDRESS, port=P.UDP_PORT):
        super().__init__()
        self.host = host
        self.port = port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.5)
        # Ephemeral port: the module latches whatever address our first packet came from, so we
        # do not need a fixed one and binding a fixed port would collide with a second instance.
        self.socket.bind(("", 0))

        self.rings = {loop.id: Ring() for loop in P.LOOPS}

        self.seq = 0
        self.packets_in = 0
        self.samples_in = 0
        self.last_rx_time = 0.0
        self._packet_window_start = time.monotonic()
        self._packet_window_count = 0
        self.packets_per_second = 0.0

        self.status = None
        self.battery = None

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="gs-rx")
        self._rx_thread.start()
        self._ka_thread = threading.Thread(target=self._keepalive_loop, daemon=True, name="gs-ka")
        self._ka_thread.start()

    # --- Transmit ----------------------------------------------------------

    def send(self, msg_type, payload):
        """Sends one record as its own datagram. Uplink traffic is rare enough not to batch."""
        self.seq = (self.seq + 1) & 0xFFFF
        try:
            self.socket.sendto(P.encode_record(msg_type, payload, self.seq),
                               (self.host, self.port))
            return True
        except OSError as error:
            self.status_text.emit(f"send failed: {error}")
            return False

    def send_select(self, loop_id, divider):
        return self.send(P.MSG_PID_SELECT, P.pack_select(loop_id, divider))

    def send_inject(self, loop_id, mode, amplitude, period_s):
        return self.send(P.MSG_PID_INJECT, P.pack_inject(loop_id, mode, amplitude, period_s))

    def send_gains(self, loop_id, kp, ki, kd, i_limit, out_limit, d_cutoff_hz):
        return self.send(P.MSG_PID_GAINS,
                         P.pack_gains(loop_id, kp, ki, kd, i_limit, out_limit, d_cutoff_hz))

    def request_gains(self, loop_id):
        return self.send(P.MSG_PID_GAINS, P.pack_gains_read_request(loop_id))

    def send_control(self, armed, flags=0, flight_mode=0):
        """Sends a control frame. See the arming caveat in protocol.pack_control."""
        return self.send(P.MSG_CONTROL,
                         P.pack_control(0.0, 0.0, 0.0, 0.0, armed, flight_mode, flags))

    # --- Receive -----------------------------------------------------------

    def _keepalive_loop(self):
        """Sends an empty STATUS record once a second.

        The module latches the sender of ANY inbound packet as its telemetry peer. Without this,
        a session that only listens would never establish one, and a session that goes quiet
        would keep working only until the module rebooted. A zero-length record is enough: the
        firmware skips empty records rather than relaying them to the flight controller.
        """
        while self._running:
            self.send(P.MSG_STATUS, b"")
            time.sleep(1.0)

    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self.socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue

            now = time.monotonic()
            self.last_rx_time = now
            self.packets_in += 1
            self._packet_window_count += 1

            elapsed = now - self._packet_window_start
            if elapsed >= 1.0:
                self.packets_per_second = self._packet_window_count / elapsed
                self._packet_window_count = 0
                self._packet_window_start = now

            for msg_type, _seq, payload in P.split_records(data):
                self._handle_record(msg_type, payload)

    def _handle_record(self, msg_type, payload):
        if msg_type == P.MSG_PID_DEBUG:
            if len(payload) < P.S_DEBUG.size:
                return
            (t_us, loop_id, flags, setpoint, measurement,
             p_term, i_term, d_term, output) = P.S_DEBUG.unpack_from(payload)

            ring = self.rings.get(loop_id)
            if ring is None:
                return
            ring.push(t_us, setpoint, measurement, p_term, i_term, d_term, output, flags)
            self.samples_in += 1

        elif msg_type == P.MSG_PID_GAINS:
            gains = P.unpack_gains(payload)
            if gains is not None:
                self.gains_received.emit(gains)

        elif msg_type == P.MSG_STATUS:
            status = P.unpack_status(payload)
            if status is not None:
                self.status = status

        elif msg_type == P.MSG_BATTERY:
            battery = P.unpack_battery(payload)
            if battery is not None:
                self.battery = battery

    # --- State -------------------------------------------------------------

    def is_online(self):
        return (time.monotonic() - self.last_rx_time) < LINK_TIMEOUT_S

    def clear_all(self):
        for ring in self.rings.values():
            ring.clear()

    def close(self):
        self._running = False
        try:
            self.socket.close()
        except OSError:
            pass
