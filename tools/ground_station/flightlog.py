"""Flight log recorder: one file per flight, written while you fly.

WHAT THIS IS FOR, AND WHY IT IS NOT THE CSV LOGGER
    The per-loop "log to CSV" checkbox in panels.py records ONE PID loop's internals - it is a
    tuning instrument. This records the FLIGHT: what the drone reported, what the pilot commanded,
    and everything that happened, in one file you can reopen afterwards or hand to somebody else
    to read. The two are complementary and can both be on at once.

THE FORMAT IS NEWLINE-DELIMITED JSON, and every part of that choice is load-bearing:

    * ONE OBJECT PER LINE, APPEND-ONLY. A log is at its most valuable exactly when the session
      ended badly - a crash, a kernel panic, a laptop lid closed on a bad landing. A file that
      needs a closing bracket to be valid is a file you lose in that case. Every line here stands
      alone, so a truncated log is still a readable log, minus its last line.
    * SELF-DESCRIBING. Each record names its own type and its fields. Six months from now the
      file still says what it is without this module to explain it.
    * PLAIN TEXT. Greppable, diffable, and you can paste a slice of it into a conversation when
      asking somebody what went wrong.

    The cost is size: roughly 1.5 MB per minute at the default rates. A ten-minute flight is
    15 MB, which is fine on disk and still small enough to hand over whole.

    RECORDING IS ALWAYS PLAIN TEXT, never gzip, and that is deliberate. A gzip stream buffers
    inside the compressor, so flushing the text layer does NOT put those bytes on disk in a
    recoverable form - a recording killed mid-flight leaves a file that is entirely unreadable
    rather than one that is readable up to the last line, which throws away the whole point of
    the format. Gzip a log AFTER the flight if you want to keep a lot of them; the reader opens
    .ndjson.gz transparently.

RECORD TYPES
    header    exactly one, first line. Format version, wall-clock start, gain snapshot.
    status    the flight controller's own view: attitude, altitude, velocity, motors, flags. 50 Hz.
    control   what the telemetry module SENT the flight controller - the pilot's demand. 50 Hz.
              Present only if the module is running firmware that echoes it; see uart_link.c.
    pid       one PID loop's internals, decimated to PID_LOG_HZ. Only the selected loop streams.
    link      once a second: packet rate, sample rate, online/offline. How the radio was doing.
    event     anything discrete: arming, kill, mode changes, gain edits, markers, link up/down.
    footer    written on a clean stop. ITS ABSENCE IS INFORMATION - it means the session did not
              end normally, which is worth knowing when you read the log back.

TIME
    Every `t` is seconds since recording started, from time.monotonic(). Monotonic rather than
    wall clock because an NTP correction mid-flight would otherwise reorder the log or produce a
    negative duration. The wall-clock start is recorded once, in the header, so absolute times
    are still recoverable.

THREADING - THE RULE THIS MODULE EXISTS TO KEEP
    Records are handed in from link.py's RECEIVE THREAD. That thread must never block: it is the
    one draining the UDP socket, and a stalled disk write there shows up as lost telemetry, not
    as a slow log. So the public methods only append to a bounded queue, and a separate writer
    thread does the file I/O.

    The queue is BOUNDED and DROPS ON OVERFLOW rather than growing. A log with a counted gap is
    recoverable; a ground station that ran out of memory mid-flight is not. Drops are counted and
    reported in the footer and in the UI.

    THE WRITER THREAD OWNS THE FILE. It opens nothing, but it is the only thing that writes to,
    flushes and closes it. stop() asks it to finish and waits briefly; if it does not finish, the
    file is left alone rather than closed underneath it. Closing a file another thread is writing
    to raises from inside that thread, kills it, and loses the tail you were trying to save.

    THE FOOTER IS WRITTEN BY THE WRITER THREAD DIRECTLY, not queued. It has to be: the queue is
    full exactly when the disk is struggling, which is exactly when a dropped footer would make a
    clean shutdown look like a crash.
"""

import gzip
import io
import json
import os
import queue
import threading
import time

FORMAT_NAME = "tinywhoop-flightlog"
FORMAT_VERSION = 1

# PID samples arrive at up to 500 Hz for the selected loop. Logging all of them is 6 MB/minute of
# a signal that is already being plotted live, so they are thinned to this. 50 Hz lines the PID
# trace up with the status stream, which is what you want when reading them side by side; open
# the loop's own tab with "log to CSV" if you need the full-rate samples for tuning forensics.
PID_LOG_HZ = 50.0

# Decimation tolerance, as a fraction of the target interval. A sample counts as "due" once it is
# within this much of its ideal time.
#
# WHY A TOLERANCE IS NEEDED AT ALL. Several loops already arrive at almost exactly PID_LOG_HZ:
# alt, vel_x and vel_y run at 50 Hz with divider 1, and the overview tab's divider of 20 puts the
# three 1 kHz rate loops at 50 Hz too. Against an exact "one full interval must have passed" test,
# a source at the target rate that is a fraction of a percent fast - or merely jittery - fails on
# alternate samples and ALIASES DOWN to 25-37 Hz, giving an irregular trace that lines up with
# nothing. 10% of an interval is far more than the microseconds of jitter a real link produces and
# far less than the gap between 50 Hz and the next rate up.
PID_DECIMATE_TOLERANCE = 0.1

# A gap wider than this many intervals means the stream restarted, the tab changed, or the link
# dropped. The schedule is resynchronised to the new sample rather than trying to catch up through
# a burst of keeps for samples that never arrived.
PID_RESYNC_INTERVALS = 3

# The firmware's PID timestamp is esp_timer_get_time() truncated to 32 bits - it wraps every
# ~71.6 minutes. Same constant as link.py's, for the same masked-subtraction reason.
US_WRAP = 1 << 32

# Bounded so a slow or full disk costs samples instead of memory. 20000 records is about 130
# seconds of everything at full rate - far more head start than a working disk ever needs.
QUEUE_LIMIT = 20000

# The writer flushes at least this often while records are arriving continuously. It also flushes
# whenever the queue runs dry, so in practice the file on disk is current. A power loss or a crash
# then costs almost nothing, which matters because the interesting part of a bad flight is the end.
FLUSH_INTERVAL_S = 1.0

# How long the writer blocks waiting for a record before looking around. This is also how long a
# normal stop() takes, so it is short: stop() is called on the GUI thread, and one of the things
# that GUI thread is responsible for is the KILL button.
IDLE_POLL_S = 0.05

# How long stop() waits for the writer to drain. Past this the file is left to the daemon thread.
JOIN_TIMEOUT_S = 1.5

# How often the link statistics record is written.
LINK_RECORD_INTERVAL_S = 1.0


class FlightRecorder:
    """Records one flight to one file. Create it to start; call stop() to finish.

    Every public method is safe to call from any thread and none of them touch the disk.
    """

    def __init__(self, path, header_extra=None):
        self.path = path
        self.start_monotonic = time.monotonic()
        self.start_wall = time.time()

        self._queue = queue.Queue(maxsize=QUEUE_LIMIT)
        self._dropped = 0
        self._written = 0
        self._counts = {}
        self._lock = threading.Lock()
        self._running = True
        self._error = None
        self._stop_reason = None

        # Per-loop decimation schedule: the timestamp at which the next kept sample is due,
        # keyed by loop id. See pid() for why it is a schedule rather than "the last one kept".
        self._pid_due_us = {}

        self._last_link_record = 0.0

        header = {
            "rec": "header",
            "format": FORMAT_NAME,
            "version": FORMAT_VERSION,
            "t": 0.0,
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(self.start_wall)),
            "started_local": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.start_wall)),
            "pid_log_hz": PID_LOG_HZ,
            "note": "t is seconds since recording start (monotonic). See flightlog.py.",
        }
        if header_extra:
            header.update(header_extra)

        # The file is opened on THIS thread, not the writer thread, so that a bad path or a
        # read-only directory raises here where the caller can show it - rather than killing a
        # background thread and leaving the UI claiming to be recording into nothing.
        self._file = _open_write(path)
        self._file.write(json.dumps(header) + "\n")
        self._file.flush()

        self._thread = threading.Thread(target=self._writer_loop, daemon=True, name="gs-flightlog")
        self._thread.start()

    # --- Recording (safe from any thread, never blocks) --------------------

    def t(self):
        """Seconds since recording started."""
        return time.monotonic() - self.start_monotonic

    def _put(self, record):
        record["t"] = round(self.t(), 4)
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def status(self, status):
        """One flight controller status frame, as returned by protocol.unpack_status()."""
        record = dict(status)
        record["rec"] = "status"
        record["motor"] = [round(value, 4) for value in status["motor"]]
        self._put(record)

    def control(self, control):
        """One control frame the telemetry module sent, from protocol.unpack_control().

        This is the PILOT'S DEMAND, and it is the half of the picture the ground station could
        not see before the firmware echoed it. A log that shows the drone rolling right is
        ambiguous on its own; the same log next to a roll demand of zero is a diagnosis.
        """
        record = dict(control)
        record["rec"] = "control"
        self._put(record)

    def pid(self, loop_name, loop_id, t_us, setpoint, measurement, p, i, d, output, flags):
        """One PID debug sample, decimated to PID_LOG_HZ.

        Decimation is on the DRONE'S timestamp rather than on arrival time, so a burst of frames
        that were delayed together in one datagram still thins evenly instead of collapsing to a
        single sample.
        """
        interval_us = 1e6 / PID_LOG_HZ
        due_us = self._pid_due_us.get(loop_id)

        if due_us is None:
            # First sample of this loop is always kept, and starts the schedule.
            self._pid_due_us[loop_id] = t_us
        else:
            # Signed, wrap-aware difference. The firmware timestamp is 32-bit and wraps every
            # ~72 minutes; an unsigned subtraction of a slightly-early sample would come back as
            # four billion rather than as a small negative, and keep everything.
            delta = ((t_us - due_us + (US_WRAP >> 1)) & (US_WRAP - 1)) - (US_WRAP >> 1)

            if delta < interval_us * (1.0 - PID_DECIMATE_TOLERANCE):
                return

            if delta > interval_us * PID_RESYNC_INTERVALS:
                self._pid_due_us[loop_id] = t_us
            else:
                # Advance by exactly one interval rather than to this sample's time. Advancing to
                # the sample lets the kept rate drift upward - a 500 Hz source would keep one every
                # 10-something ms instead of every 20 ms, at double the intended size on disk.
                self._pid_due_us[loop_id] = int(due_us + interval_us) & (US_WRAP - 1)

        self._put({
            "rec": "pid",
            "loop": loop_name,
            "loop_id": loop_id,
            "sp": round(setpoint, 5),
            "meas": round(measurement, 5),
            "p": round(p, 5),
            "i": round(i, 5),
            "d": round(d, 5),
            "out": round(output, 5),
            "flags": int(flags),
        })

    def link(self, packets_per_second, samples_in, packets_in, online):
        """Link statistics. Rate-limited internally, so it is safe to call every tick."""
        now = self.t()
        if (now - self._last_link_record) < LINK_RECORD_INTERVAL_S:
            return
        self._last_link_record = now
        self._put({
            "rec": "link",
            "pkt_per_s": round(packets_per_second, 2),
            "packets_in": int(packets_in),
            "samples_in": int(samples_in),
            "online": bool(online),
        })

    def event(self, kind, **fields):
        """One discrete thing that happened. `kind` is a short stable string."""
        record = {"rec": "event", "kind": kind}
        record.update(fields)
        self._put(record)

    # --- State -------------------------------------------------------------

    def stats(self):
        """(records written, records dropped, bytes on disk) - for the UI readout."""
        with self._lock:
            written, dropped = self._written, self._dropped
        try:
            size = os.path.getsize(self.path)
        except OSError:
            size = 0
        return written, dropped, size

    def error(self):
        """The write error that stopped this recording, or None."""
        with self._lock:
            return self._error

    def stop(self, reason="stopped"):
        """Asks the writer to finish, and waits briefly for it.

        The footer is NOT queued - the writer writes it itself, after draining everything else,
        with the final counts. That is what makes "no footer" mean "this session died" rather
        than "the disk was busy for a moment at the end".

        The file is closed by the writer, not here. If the writer does not finish inside
        JOIN_TIMEOUT_S the file is left open and the daemon thread is left to it: closing a file
        out from under a thread that is writing to it raises inside that thread, kills it, and
        loses exactly the tail this method exists to save.
        """
        if not self._running:
            return

        self._stop_reason = reason
        self._running = False
        self._thread.join(timeout=JOIN_TIMEOUT_S)

    # --- Writer thread -----------------------------------------------------

    def _write(self, record):
        """Serialises one record. Returns False if the file has gone away."""
        try:
            self._file.write(json.dumps(record) + "\n")
        except (OSError, ValueError, TypeError) as error:
            # ValueError covers a file closed underneath us; TypeError an unserialisable value.
            # One bad record must not end the recording, so this is counted, not raised.
            with self._lock:
                self._error = str(error)
            return False

        with self._lock:
            self._written += 1
            kind = record.get("rec", "?")
            self._counts[kind] = self._counts.get(kind, 0) + 1
        return True

    def _flush(self):
        try:
            self._file.flush()
        except (OSError, ValueError) as error:
            with self._lock:
                self._error = str(error)

    def _writer_loop(self):
        last_flush = time.monotonic()

        while self._running:
            try:
                record = self._queue.get(timeout=IDLE_POLL_S)
            except queue.Empty:
                # Nothing pending: a good moment to put the tail on disk, so a power loss costs
                # nothing rather than up to FLUSH_INTERVAL_S of records.
                self._flush()
                last_flush = time.monotonic()
                continue

            self._write(record)

            now = time.monotonic()
            if (now - last_flush) >= FLUSH_INTERVAL_S:
                last_flush = now
                self._flush()

        # --- Shutdown: drain, then footer, then close -----------------------
        # Drained with get_nowait rather than by re-testing _running, so nothing queued before the
        # stop can be stranded by the loop above exiting between an Empty and a put.
        while True:
            try:
                record = self._queue.get_nowait()
            except queue.Empty:
                break
            self._write(record)

        if self._stop_reason is not None:
            with self._lock:
                counts = dict(self._counts)
                dropped = self._dropped
            self._write({
                "rec": "footer",
                "t": round(self.t(), 4),
                "reason": self._stop_reason,
                "duration_s": round(self.t(), 3),
                "records": counts,
                "dropped": dropped,
                "ended_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            })

        self._flush()
        try:
            self._file.close()
        except (OSError, ValueError):
            pass


def default_path(directory=None):
    """A timestamped log name in `directory` (the working directory by default)."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(directory or os.getcwd(), f"flight_{stamp}.ndjson")


def _open_write(path):
    """Recording is always plain text. See the module docstring for why never gzip."""
    return open(path, "w", encoding="utf-8", buffering=1024 * 64)


def _as_float(value, default=0.0):
    """Coerces a logged value to a float, never raising. See flightreport._num for the reasoning."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if number == number and abs(number) != float("inf") else default


def _open_read(path):
    if path.endswith(".gz"):
        return io.TextIOWrapper(gzip.open(path, "rb"), encoding="utf-8")
    return open(path, "r", encoding="utf-8")


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

class FlightLog:
    """A parsed flight log, ready to plot or summarise.

    Records are separated by type into plain lists of dicts. The file is small enough that this
    is fine - a 15 MB log is a few hundred thousand dicts, which parses in a second or two and
    then never has to be touched again. The live path is where the preallocated-numpy discipline
    matters; this one runs once, on a file, off the flight line.

    A TRUNCATED FILE IS NOT AN ERROR HERE. The last line of a log from a session that crashed is
    usually half-written; it is counted and skipped, because the other 99.9% of the file is
    exactly the data you opened it to look at. A truncated .gz is different - gzip cannot hand
    back a partial stream - so that raises, and the callers report it.
    """

    def __init__(self, path):
        self.path = path
        self.header = {}
        self.footer = None
        self.status = []
        self.control = []
        self.pid = []
        self.link = []
        self.events = []
        self.bad_lines = 0

        # Set by logtool's --slice. Without it duration_s would keep reporting the whole flight's
        # length after the records had been cut down to a two-second window, and every rate in the
        # report would come out divided by the wrong number.
        self.window = None

        buckets = {
            "status": self.status,
            "control": self.control,
            "pid": self.pid,
            "link": self.link,
            "event": self.events,
        }

        with _open_read(path) as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    self.bad_lines += 1
                    continue

                kind = record.get("rec")
                if kind == "header":
                    self.header = record
                elif kind == "footer":
                    self.footer = record
                else:
                    bucket = buckets.get(kind)
                    if bucket is not None:
                        bucket.append(record)
                    else:
                        self.bad_lines += 1

    # --- Derived -----------------------------------------------------------

    @property
    def complete(self):
        """True if the recording was stopped cleanly. False means the session died mid-flight."""
        return self.footer is not None

    @property
    def duration_s(self):
        if self.window is not None:
            return max(0.0, self.window[1] - self.window[0])
        if self.footer is not None:
            return _as_float(self.footer.get("duration_s"))
        # No footer, so the length is wherever the records stop. Coerced rather than cast: the
        # last line of a truncated log is exactly where a malformed value turns up.
        last = 0.0
        for bucket in (self.status, self.control, self.pid, self.link, self.events):
            if bucket:
                last = max(last, _as_float(bucket[-1].get("t")))
        return last

    def column(self, records, key, default=0.0):
        """One field from a record list as a float list, for plotting."""
        return [_as_float(record.get(key), default) for record in records]

    def pid_loops(self):
        """Loop names present in this log, in first-seen order."""
        seen = []
        for sample in self.pid:
            name = sample.get("loop")
            if name not in seen:
                seen.append(name)
        return seen

    def pid_for(self, loop_name):
        return [sample for sample in self.pid if sample.get("loop") == loop_name]
