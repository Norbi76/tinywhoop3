"""The per-loop tuning panel: three linked plots, the gain editor and the statistics readout.

This is where the tool's actual work happens. Each panel owns one PID loop's view:

  * three X-linked plots (setpoint/measurement, error, and the P/I/D contributions plus output),
    so a feature in one lines up across all three;
  * the gain editor, which reads the drone's live gains on tab selection and writes them back on
    apply;
  * the statistics readout (sample rate, RMS/peak error, saturation percentages, overshoot and
    settling time), recomputed a few times a second over the visible window.

Two invariants, both about not stuttering at 500 Hz:

  * Nothing here is called from the receive thread. Panels are redrawn by the single 30 Hz timer
    on the GUI thread in gs.py, reading whatever the ring buffers currently hold.
  * The error trace is COMPUTED here rather than received. It is exactly setpoint - measurement,
    and at 500 Hz those four bytes would be a fifth of the link budget for something this side
    already knows.

The overview panel at the bottom of the file is a deliberately different thing: it subscribes to
every loop at once, which means the per-loop sample rates differ (the decimation counter counts
each loop's own ticks). It is for spotting WHICH loop is misbehaving, not for measuring one.
"""

import csv
import os
import time

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QSpinBox, QSplitter, QVBoxLayout, QWidget,
)

import protocol as P
from flightlog import FlightLog
from flightreport import build_report
from link import COL_D, COL_FLAGS, COL_I, COL_MEAS, COL_OUT, COL_P, COL_SP, COL_T

# A setpoint jump smaller than this fraction of the window's setpoint range is treated as drift
# rather than as a commanded step.
STEP_DETECT_FRACTION = 0.35

# ...and it must also be this many times the typical sample-to-sample movement of the setpoint.
#
# The range test alone is not enough. On a setpoint that is pure noise, the largest single jump
# is naturally a large fraction of the total range - about 4.5 sigma against a range of 6.5 sigma
# - so a range test on its own reports a step, and an overshoot figure, for a loop that was never
# stepped. Comparing against the MEDIAN absolute jump discriminates properly: a real commanded
# step is one big jump among near-zero ones, so the median stays tiny, while noise has a median
# jump of the same order as its largest. It also correctly rejects a steady ramp, where every
# jump is the same size.
STEP_DETECT_JITTER_RATIO = 8.0

# Settling band, as a fraction of the step size. 5% is the conventional figure and is what the
# readout's label says.
SETTLING_BAND = 0.05

# Statistics are recomputed at a quarter of the redraw rate: they are read, not watched, and
# recomputing them 30 times a second is wasted work on a buffer of thousands of samples.
STATS_INTERVAL_S = 0.25

GAIN_FIELDS = [
    ("kp", "Kp", 6),
    ("ki", "Ki", 6),
    ("kd", "Kd", 6),
    ("i_limit", "I limit", 4),
    ("out_limit", "Out limit", 4),
    ("d_cutoff_hz", "D cutoff (Hz)", 2),
]


def _analyse_step(times, setpoints, measurements):
    """Finds the last commanded setpoint step and measures the response to it.

    @return (overshoot_percent, settling_time_s) with either entry None if not determinable.
    """
    if times.size < 20:
        return None, None

    deltas = np.diff(setpoints)
    if deltas.size == 0:
        return None, None

    span = float(np.ptp(setpoints))
    if span <= 0.0:
        return None, None   # a flat setpoint has no step in it

    absolute_deltas = np.abs(deltas)
    index = int(np.argmax(absolute_deltas))
    step_size = float(deltas[index])

    jitter = float(np.median(absolute_deltas))
    threshold = max(span * STEP_DETECT_FRACTION, jitter * STEP_DETECT_JITTER_RATIO)
    if abs(step_size) < threshold:
        return None, None

    # Everything after the step. Need enough of a tail for the response to mean anything.
    after = slice(index + 1, None)
    t_after = times[after]
    m_after = measurements[after]
    if t_after.size < 10:
        return None, None

    target = float(setpoints[-1])
    start = float(measurements[index])
    travel = target - start
    if abs(travel) < 1e-9:
        return None, None

    # Overshoot: how far past the target the measurement went, in the direction of travel.
    if travel > 0:
        peak = float(np.max(m_after))
        overshoot = (peak - target) / travel * 100.0
    else:
        peak = float(np.min(m_after))
        overshoot = (target - peak) / (-travel) * 100.0
    overshoot = max(overshoot, 0.0)

    # Settling time: the last moment the measurement was outside the 5% band. If it never left
    # the band the loop settled immediately, which is 0, not "unknown".
    band = abs(travel) * SETTLING_BAND
    outside = np.nonzero(np.abs(m_after - target) > band)[0]
    if outside.size == 0:
        settling = 0.0
    elif outside[-1] >= t_after.size - 1:
        settling = None   # still outside the band at the end of the window, so not settled yet
    else:
        settling = float(t_after[outside[-1]] - times[index])

    return overshoot, settling


class LoopPanel(QWidget):
    """One tab: everything needed to tune a single PID loop."""

    def __init__(self, loop, link, parent=None):
        super().__init__(parent)
        self.loop = loop
        self.link = link
        self.ring = link.rings[loop.id]

        self.window_s = 5.0
        self._last_stats_time = 0.0
        self._csv_writer = None
        self._csv_file = None
        self._csv_last_t = -1.0

        layout = QHBoxLayout(self)
        layout.addWidget(self._build_plots(), stretch=4)
        layout.addWidget(self._build_sidebar(), stretch=0)

    # --- Construction ------------------------------------------------------

    def _build_plots(self):
        pg.setConfigOptions(antialias=True)
        widget = pg.GraphicsLayoutWidget()

        self.plot_track = widget.addPlot(row=0, col=0, title="Setpoint and measurement")
        self.plot_error = widget.addPlot(row=1, col=0, title="Error (setpoint - measurement)")
        self.plot_terms = widget.addPlot(row=2, col=0, title="P / I / D contributions and output")

        for plot in (self.plot_track, self.plot_error, self.plot_terms):
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend(offset=(-10, 10))

        # X-linked so that panning or zooming any one of them lines all three up against the same
        # instant - which is the entire point of showing them stacked.
        self.plot_error.setXLink(self.plot_track)
        self.plot_terms.setXLink(self.plot_track)

        self.plot_track.setLabel("left", self.loop.sp_unit)
        self.plot_error.setLabel("left", self.loop.sp_unit)
        self.plot_terms.setLabel("left", self.loop.out_unit)
        self.plot_terms.setLabel("bottom", "time", units="s")

        self.curve_sp = self.plot_track.plot(pen=pg.mkPen("#f0c000", width=2), name="setpoint")
        self.curve_meas = self.plot_track.plot(pen=pg.mkPen("#20b0ff", width=1), name="measured")

        self.curve_error = self.plot_error.plot(pen=pg.mkPen("#ff5050", width=1), name="error")
        self.plot_error.addLine(y=0, pen=pg.mkPen("#808080", style=Qt.PenStyle.DashLine))

        self.curve_p = self.plot_terms.plot(pen=pg.mkPen("#50d050", width=1), name="P")
        self.curve_i = self.plot_terms.plot(pen=pg.mkPen("#d050d0", width=1), name="I")
        self.curve_d = self.plot_terms.plot(pen=pg.mkPen("#00c0c0", width=1), name="D")
        self.curve_out = self.plot_terms.plot(pen=pg.mkPen("#ffffff", width=2), name="output")

        return widget

    def _build_sidebar(self):
        sidebar = QWidget()
        sidebar.setFixedWidth(260)
        column = QVBoxLayout(sidebar)

        header = QLabel(f"<b>{self.loop.name}</b><br>"
                        f"{self.loop.block}<br>"
                        f"{self.loop.sp_unit} &rarr; {self.loop.out_unit} @ {self.loop.rate} Hz")
        header.setWordWrap(True)
        column.addWidget(header)

        column.addWidget(self._build_gain_group())
        column.addWidget(self._build_inject_group())
        column.addWidget(self._build_view_group())

        self.stats_label = QLabel("no data")
        self.stats_label.setFont(QFont("monospace", 9))
        self.stats_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        stats_box = QGroupBox("statistics")
        stats_layout = QVBoxLayout(stats_box)
        stats_layout.addWidget(self.stats_label)
        column.addWidget(stats_box)

        column.addStretch(1)
        return sidebar

    def _build_gain_group(self):
        box = QGroupBox("gains")
        form = QFormLayout(box)

        self.gain_boxes = {}
        for key, label, decimals in GAIN_FIELDS:
            spin = QDoubleSpinBox()
            spin.setDecimals(decimals)
            spin.setRange(-1e6, 1e6)
            spin.setSingleStep(10.0 ** -decimals * 10)
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(self._on_gain_edited)
            form.addRow(label, spin)
            self.gain_boxes[key] = spin

        buttons = QHBoxLayout()
        apply_button = QPushButton("apply")
        apply_button.clicked.connect(self.apply_gains)
        read_button = QPushButton("read back")
        read_button.clicked.connect(self.request_gains)
        buttons.addWidget(apply_button)
        buttons.addWidget(read_button)
        form.addRow(buttons)

        # Off by default on purpose: with it on, dragging a spinbox sends a gain set per
        # increment, and every one of those resets the loop's integrator on the drone.
        self.live_checkbox = QCheckBox("apply on edit")
        form.addRow(self.live_checkbox)

        return box

    def _build_inject_group(self):
        box = QGroupBox("test signal")
        form = QFormLayout(box)

        self.mode_combo = QComboBox()
        for name, _value in P.INJECT_MODES:
            self.mode_combo.addItem(name)
        form.addRow("mode", self.mode_combo)

        self.amplitude_box = QDoubleSpinBox()
        self.amplitude_box.setDecimals(3)
        self.amplitude_box.setRange(-1e4, 1e4)
        self.amplitude_box.setValue(5.0)
        form.addRow(f"amplitude ({self.loop.sp_unit})", self.amplitude_box)

        self.period_box = QDoubleSpinBox()
        self.period_box.setDecimals(2)
        self.period_box.setRange(0.05, 60.0)
        self.period_box.setValue(1.0)
        form.addRow("period (s)", self.period_box)

        send_button = QPushButton("send")
        send_button.clicked.connect(self.send_inject)
        form.addRow(send_button)

        return box

    def _build_view_group(self):
        box = QGroupBox("view")
        form = QFormLayout(box)

        self.window_box = QSpinBox()
        self.window_box.setRange(1, 60)
        self.window_box.setValue(int(self.window_s))
        self.window_box.setSuffix(" s")
        self.window_box.valueChanged.connect(self._on_window_changed)
        form.addRow("window", self.window_box)

        clear_button = QPushButton("clear")
        clear_button.clicked.connect(self.clear)
        form.addRow(clear_button)

        self.csv_checkbox = QCheckBox("log to CSV")
        self.csv_checkbox.toggled.connect(self._on_csv_toggled)
        form.addRow(self.csv_checkbox)

        return box

    # --- Actions -----------------------------------------------------------

    def _on_window_changed(self, value):
        self.window_s = float(value)

    def _on_gain_edited(self, _value):
        if self.live_checkbox.isChecked():
            self.apply_gains()

    def gain_values(self):
        return {key: self.gain_boxes[key].value() for key, _label, _decimals in GAIN_FIELDS}

    def set_gain_values(self, gains):
        """Fills the spinboxes without triggering a send, so a read-back cannot loop."""
        for key, _label, _decimals in GAIN_FIELDS:
            spin = self.gain_boxes[key]
            spin.blockSignals(True)
            spin.setValue(float(gains.get(key, 0.0)))
            spin.blockSignals(False)

    def apply_gains(self):
        values = self.gain_values()
        self.link.send_gains(self.loop.id, values["kp"], values["ki"], values["kd"],
                             values["i_limit"], values["out_limit"], values["d_cutoff_hz"])
        # Recorded because applying gains also RESETS that loop's integrator on the drone. A step
        # in behaviour with no obvious cause is very often this, and without the event in the log
        # there is nothing to connect the two.
        self._record_event("gains_applied", loop=self.loop.name, **values)

    def request_gains(self):
        self.link.request_gains(self.loop.id)

    def send_inject(self):
        mode = P.INJECT_MODES[self.mode_combo.currentIndex()][1]
        self.link.send_inject(self.loop.id, mode,
                              self.amplitude_box.value(), self.period_box.value())
        self._record_event("inject", loop=self.loop.name,
                           mode=P.INJECT_MODES[self.mode_combo.currentIndex()][0],
                           amplitude=self.amplitude_box.value(),
                           period_s=self.period_box.value())

    def _record_event(self, kind, **fields):
        """Writes an event to the flight log if one is being recorded, and nothing if not."""
        recorder = self.link.recorder
        if recorder is not None:
            recorder.event(kind, **fields)

    def clear(self):
        self.ring.clear()

        # The ring re-anchors its timebase on the next sample, so timestamps restart from zero.
        # Rolling the CSV over rather than appending keeps each file monotonic in t_s - appending
        # would produce two runs starting at 0 in one file, which is confusing to analyse later.
        if self.csv_checkbox.isChecked():
            self._close_csv()
            self._on_csv_toggled(True)

        self._csv_last_t = -1.0
        for curve in (self.curve_sp, self.curve_meas, self.curve_error,
                      self.curve_p, self.curve_i, self.curve_d, self.curve_out):
            curve.setData([], [])

    def _on_csv_toggled(self, enabled):
        if enabled:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(os.getcwd(), f"log_{self.loop.name}_{stamp}.csv")
            self._csv_file = open(path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(
                ["t_s", "setpoint", "measurement", "error", "p", "i", "d", "output", "flags"])
            self._csv_last_t = -1.0
            self.link.status_text.emit(f"logging {self.loop.name} to {os.path.basename(path)}")
        else:
            self._close_csv()

    def _close_csv(self):
        if self._csv_file is not None:
            self._csv_file.close()
        self._csv_file = None
        self._csv_writer = None

    def _write_csv(self, samples):
        """Appends every sample newer than the last one written."""
        if self._csv_writer is None or samples.shape[0] == 0:
            return

        fresh = samples[samples[:, COL_T] > self._csv_last_t]
        if fresh.shape[0] == 0:
            return

        for row in fresh:
            self._csv_writer.writerow([
                f"{row[COL_T]:.6f}", f"{row[COL_SP]:.6f}", f"{row[COL_MEAS]:.6f}",
                f"{row[COL_SP] - row[COL_MEAS]:.6f}",
                f"{row[COL_P]:.6f}", f"{row[COL_I]:.6f}", f"{row[COL_D]:.6f}",
                f"{row[COL_OUT]:.6f}", int(row[COL_FLAGS]),
            ])
        self._csv_last_t = float(fresh[-1, COL_T])

    # --- Redraw ------------------------------------------------------------

    def pump_csv(self):
        """Drains this loop's ring into its CSV, without redrawing anything.

        CALLED FOR EVERY PANEL ON EVERY TICK, not just the visible one, and that is the point.
        Writing the CSV used to happen inside refresh(), which gs.py only calls on the tab you are
        looking at - so a loop that was still streaming while you looked at a different tab filled
        its ring with nobody draining it, and the ring only holds the last few seconds. Coming back
        to the tab silently lost everything in between. Switching between two LOOP tabs hides this
        (the PID_SELECT stops the old stream), but the overview and replay tabs both leave a loop
        streaming, and the hole is invisible in the file afterwards.

        Cheap when not logging: one attribute test and a return.
        """
        if self._csv_writer is None:
            return
        # The whole buffer, not the display window: the display window is a viewing preference and
        # has no business deciding what gets recorded.
        self._write_csv(self.ring.snapshot())

    def refresh(self):
        """Redraws from the ring buffer. Called on the GUI thread only."""
        samples = self.ring.snapshot(self.window_s)

        if samples.shape[0] == 0:
            self.stats_label.setText("no data\nselect this tab to subscribe")
            return

        times = samples[:, COL_T]
        setpoints = samples[:, COL_SP]
        measurements = samples[:, COL_MEAS]
        errors = setpoints - measurements

        self.curve_sp.setData(times, setpoints)
        self.curve_meas.setData(times, measurements)
        self.curve_error.setData(times, errors)
        self.curve_p.setData(times, samples[:, COL_P])
        self.curve_i.setData(times, samples[:, COL_I])
        self.curve_d.setData(times, samples[:, COL_D])
        self.curve_out.setData(times, samples[:, COL_OUT])

        now = time.monotonic()
        if (now - self._last_stats_time) >= STATS_INTERVAL_S:
            self._last_stats_time = now
            self._update_stats(times, setpoints, measurements, errors, samples[:, COL_FLAGS])

    def _update_stats(self, times, setpoints, measurements, errors, flags):
        count = times.size
        span = float(times[-1] - times[0]) if count > 1 else 0.0
        rate = (count - 1) / span if span > 0 else 0.0

        rms = float(np.sqrt(np.mean(errors ** 2)))
        peak = float(np.max(np.abs(errors)))

        flags_int = flags.astype(np.int32)
        saturated = float(np.count_nonzero(flags_int & P.FLAG_OUT_SAT)) / count * 100.0
        clamped = float(np.count_nonzero(flags_int & P.FLAG_I_CLAMPED)) / count * 100.0

        overshoot, settling = _analyse_step(times, setpoints, measurements)
        overshoot_text = "  --" if overshoot is None else f"{overshoot:6.1f} %"
        if settling is None:
            settling_text = "  --" if overshoot is None else "not settled"
        else:
            settling_text = f"{settling * 1000.0:6.0f} ms"

        self.stats_label.setText(
            f"samples   {count:6d}\n"
            f"rate      {rate:6.1f} Hz\n"
            f"RMS err   {rms:9.4f} {self.loop.sp_unit}\n"
            f"peak err  {peak:9.4f} {self.loop.sp_unit}\n"
            f"out sat   {saturated:6.1f} %\n"
            f"I clamped {clamped:6.1f} %\n"
            f"---- step response ----\n"
            f"overshoot {overshoot_text}\n"
            f"settle 5% {settling_text}"
        )

    def closeEvent(self, event):
        self._close_csv()
        super().closeEvent(event)


class OverviewPanel(QWidget):
    """Eight small tracking-only plots, for watching every loop at once at a reduced rate."""

    def __init__(self, link, parent=None):
        super().__init__(parent)
        self.link = link
        self.window_s = 10.0
        self.curves = {}

        layout = QVBoxLayout(self)
        note = QLabel(
            "All eight loops at once, setpoint against measurement only. One divider is shared by "
            "loops that tick at different rates, so the per-loop sample rate is NOT uniform - see "
            "the README. Open a loop's own tab to stream it at full rate."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        widget = pg.GraphicsLayoutWidget()
        layout.addWidget(widget)

        for index, loop in enumerate(P.LOOPS):
            plot = widget.addPlot(row=index // 2, col=index % 2, title=loop.name)
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("left", loop.sp_unit)
            self.curves[loop.id] = (
                plot.plot(pen=pg.mkPen("#f0c000", width=2)),
                plot.plot(pen=pg.mkPen("#20b0ff", width=1)),
            )

    def refresh(self):
        for loop in P.LOOPS:
            samples = self.link.rings[loop.id].snapshot(self.window_s)
            sp_curve, meas_curve = self.curves[loop.id]
            if samples.shape[0] == 0:
                continue
            times = samples[:, COL_T]
            sp_curve.setData(times, samples[:, COL_SP])
            meas_curve.setData(times, samples[:, COL_MEAS])


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------

# Arm periods are shaded rather than drawn as a line, because what you almost always want to know
# is "was it flying when that happened", and a band answers that at a glance across every plot.
ARM_BRUSH = pg.mkBrush(60, 140, 80, 40)
KILL_BRUSH = pg.mkBrush(180, 40, 40, 60)
MARK_PEN = pg.mkPen("#ffb000", width=2, style=Qt.PenStyle.DashLine)
EVENT_PEN = pg.mkPen("#6080a0", width=1, style=Qt.PenStyle.DotLine)

# Events worth a line on the plots. The rest are in the report; drawing all of them would put a
# picket fence across a tuning session and hide the signal.
PLOTTED_EVENTS = {"arm_request", "disarm_request", "kill", "mode", "gains_applied", "inject",
                  "link_down", "link_up"}


class ReplayPanel(QWidget):
    """Opens a recorded flight and shows it: the traces, the arm bands, and the report.

    WHY THIS IS A TAB AND NOT A SEPARATE PROGRAM
      It reads the same pyqtgraph setup, and more usefully it sits next to the live tabs: land,
      open the log, look at what happened, change a gain on the loop's tab, fly again. A separate
      viewer would mean alt-tabbing between two windows that cannot see each other's state.

    WHAT IT IS NOT
      It is not live. Nothing here touches the link, and refresh() is deliberately a no-op - the
      30 Hz timer in gs.py calls refresh() on whatever tab is visible, and redrawing static data
      thirty times a second would burn a core for no reason.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.log = None
        self.report_text = ""
        self._regions = []
        self._lines = []

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_toolbar())

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_plots())
        splitter.addWidget(self._build_report())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter)

    # --- Construction ------------------------------------------------------

    def _build_toolbar(self):
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)

        open_button = QPushButton("open flight log...")
        open_button.clicked.connect(self.open_dialog)
        row.addWidget(open_button)

        self.reload_button = QPushButton("reload")
        self.reload_button.setEnabled(False)
        self.reload_button.clicked.connect(self.reload)
        row.addWidget(self.reload_button)

        self.file_label = QLabel("no log loaded")
        self.file_label.setFont(QFont("monospace", 9))
        row.addWidget(self.file_label)

        row.addStretch(1)

        # The point of this button: the report is written to be pasted somewhere. One click to
        # get it onto the clipboard is the difference between doing that and not bothering.
        copy_button = QPushButton("copy report")
        copy_button.clicked.connect(self.copy_report)
        row.addWidget(copy_button)

        return bar

    def _build_plots(self):
        widget = pg.GraphicsLayoutWidget()

        self.plot_attitude = widget.addPlot(row=0, col=0, title="Attitude - commanded vs actual")
        self.plot_height = widget.addPlot(row=1, col=0, title="Altitude and throttle demand")
        self.plot_velocity = widget.addPlot(row=2, col=0, title="Body velocity (optical flow)")
        self.plot_motors = widget.addPlot(row=3, col=0, title="Motor outputs")

        self.plots = (self.plot_attitude, self.plot_height,
                      self.plot_velocity, self.plot_motors)

        for plot in self.plots:
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend(offset=(-10, 10))
            # Every plot shares one X axis: the whole value of a flight log is lining a wobble in
            # one trace up against what the motors and the pilot were doing at that instant.
            if plot is not self.plot_attitude:
                plot.setXLink(self.plot_attitude)

        self.plot_attitude.setLabel("left", "deg")
        self.plot_height.setLabel("left", "m  /  0-1")
        self.plot_velocity.setLabel("left", "m/s")
        self.plot_motors.setLabel("left", "0-1")
        self.plot_motors.setLabel("bottom", "time", units="s")

        # Commanded traces are dashed, measured are solid, and a command/measurement pair shares
        # a colour. That is the whole reading convention and it is worth keeping consistent.
        self.curve_roll = self.plot_attitude.plot(pen=pg.mkPen("#20b0ff", width=1), name="roll")
        self.curve_roll_sp = self.plot_attitude.plot(
            pen=pg.mkPen("#20b0ff", width=2, style=Qt.PenStyle.DashLine), name="roll cmd")
        self.curve_pitch = self.plot_attitude.plot(pen=pg.mkPen("#f0c000", width=1), name="pitch")
        self.curve_pitch_sp = self.plot_attitude.plot(
            pen=pg.mkPen("#f0c000", width=2, style=Qt.PenStyle.DashLine), name="pitch cmd")
        self.curve_yaw = self.plot_attitude.plot(
            pen=pg.mkPen("#a0a0a0", width=1), name="yaw (drifts)")

        self.curve_alt = self.plot_height.plot(pen=pg.mkPen("#50d050", width=2), name="altitude m")
        self.curve_climb = self.plot_height.plot(pen=pg.mkPen("#00c0c0", width=1), name="climb m/s")
        self.curve_throttle = self.plot_height.plot(
            pen=pg.mkPen("#ff8000", width=2, style=Qt.PenStyle.DashLine), name="throttle cmd")

        self.curve_vx = self.plot_velocity.plot(pen=pg.mkPen("#20b0ff", width=1), name="forward")
        self.curve_vy = self.plot_velocity.plot(pen=pg.mkPen("#d050d0", width=1), name="right")

        motor_colours = ["#ff6060", "#ffb000", "#60c0ff", "#80e080"]
        motor_names = ["front-left", "front-right", "rear-left", "rear-right"]
        self.curve_motors = [
            self.plot_motors.plot(pen=pg.mkPen(colour, width=1), name=name)
            for colour, name in zip(motor_colours, motor_names)
        ]

        return widget

    def _build_report(self):
        self.report_view = QPlainTextEdit()
        self.report_view.setReadOnly(True)
        self.report_view.setFont(QFont("monospace", 9))
        self.report_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.report_view.setPlainText(
            "Open a flight log to see the report here.\n\n"
            "Logs are written by the REC button in the top bar and land in the ground station's\n"
            "working directory as flight_<date>_<time>.ndjson.\n\n"
            "The same report is available without the GUI:\n"
            "    python3 logtool.py flight_....ndjson\n")
        return self.report_view

    # --- Loading -----------------------------------------------------------

    def open_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open flight log", "",
            "Flight logs (*.ndjson *.ndjson.gz);;All files (*)")
        if path:
            self.load(path)

    def reload(self):
        """Re-reads the same file. Useful while a flight is still being recorded."""
        if self.log is not None:
            self.load(self.log.path)

    def load(self, path):
        # EOFError is in the list because gzip raises it on a truncated .gz, and it is neither an
        # OSError nor a ValueError. The file dialog offers .ndjson.gz, so it is reachable.
        try:
            log = FlightLog(path)
        except (OSError, ValueError, EOFError) as error:
            self.report_view.setPlainText(f"Could not read {path}:\n\n{error}")
            return

        # The report and the draw are inside the guard too. They walk records that may have come
        # from an older version, a different tool, or a half-written line, and a panel left half
        # updated - new file name, new report, previous flight's curves - is worse than a clean
        # error message. Either the whole panel changes or none of it does.
        try:
            report = build_report(log)
        except Exception as error:                       # noqa: BLE001 - see above
            self.report_view.setPlainText(
                f"Read {path} ({len(log.status)} status records) but could not summarise it:\n\n"
                f"{type(error).__name__}: {error}")
            return

        self.log = log
        self.file_label.setText(os.path.basename(path))
        self.reload_button.setEnabled(True)
        self.report_text = report
        self.report_view.setPlainText(report)

        try:
            self._draw(log)
        except Exception as error:                       # noqa: BLE001
            self._clear_overlays()
            self.report_view.setPlainText(
                f"{report}\n\n--- plotting failed ---\n{type(error).__name__}: {error}")

    def copy_report(self):
        if self.report_text:
            QApplication.clipboard().setText(self.report_text)

    # --- Drawing -----------------------------------------------------------

    def _draw(self, log):
        self._clear_overlays()

        status = log.status
        control = log.control

        if status:
            times = _column(status, "t")
            self.curve_roll.setData(times, _column(status, "roll"))
            self.curve_pitch.setData(times, _column(status, "pitch"))
            self.curve_yaw.setData(times, _column(status, "yaw"))
            self.curve_alt.setData(times, _column(status, "altitude"))
            self.curve_climb.setData(times, _column(status, "climb_rate"))
            self.curve_vx.setData(times, _column(status, "velocity_x"))
            self.curve_vy.setData(times, _column(status, "velocity_y"))

            # Built row by row into a fixed-width array rather than np.array(list-of-lists):
            # one record with a short or long motor list makes that raise on an inhomogeneous
            # shape, which would take the whole plot down over one bad line.
            motors = np.zeros((len(status), 4), dtype=np.float64)
            for row_index, record in enumerate(status):
                values = record.get("motor")
                if isinstance(values, (list, tuple)):
                    for column_index, value in enumerate(values[:4]):
                        try:
                            motors[row_index, column_index] = float(value)
                        except (TypeError, ValueError):
                            pass
            for index, curve in enumerate(self.curve_motors):
                curve.setData(times, motors[:, index])
        else:
            for curve in (self.curve_roll, self.curve_pitch, self.curve_yaw, self.curve_alt,
                          self.curve_climb, self.curve_vx, self.curve_vy, *self.curve_motors):
                curve.setData([], [])

        if control:
            command_times = _column(control, "t")
            self.curve_roll_sp.setData(command_times, _column(control, "roll_sp"))
            self.curve_pitch_sp.setData(command_times, _column(control, "pitch_sp"))
            self.curve_throttle.setData(command_times, _column(control, "throttle"))
        else:
            # No echo in this log. Left empty rather than zeroed: a flat line at zero would read
            # as "the pilot commanded nothing", which is a very different claim from "unknown".
            for curve in (self.curve_roll_sp, self.curve_pitch_sp, self.curve_throttle):
                curve.setData([], [])

        self._draw_arm_bands(status)
        self._draw_events(log.events)

        if status:
            self.plot_attitude.setXRange(0, float(times[-1]), padding=0.02)

    def _draw_arm_bands(self, status):
        """Shades every armed stretch across all four plots."""
        if not status:
            return

        from flightreport import STATUS_FLAG_KILLED, find_arm_sessions

        for session in find_arm_sessions(status):
            killed = any(int(r.get("flags", 0)) & STATUS_FLAG_KILLED for r in session.rows)
            for plot in self.plots:
                region = pg.LinearRegionItem(
                    values=(session.start_t, session.end_t),
                    brush=KILL_BRUSH if killed else ARM_BRUSH,
                    movable=False)
                # Behind the curves, so shading never hides a trace.
                region.setZValue(-10)
                plot.addItem(region)
                self._regions.append((plot, region))

    def _draw_events(self, events):
        for event in events:
            kind = event.get("kind")
            if kind not in PLOTTED_EVENTS and kind != "mark":
                continue

            try:
                t = float(event.get("t", 0.0))
            except (TypeError, ValueError):
                continue
            is_mark = kind == "mark"
            line = pg.InfiniteLine(
                pos=t, angle=90,
                pen=MARK_PEN if is_mark else EVENT_PEN,
                label=(event.get("text") or "mark") if is_mark else kind,
                labelOpts={"position": 0.92 if is_mark else 0.05,
                           "color": "#ffb000" if is_mark else "#6080a0",
                           "movable": False})
            self.plot_attitude.addItem(line)
            self._lines.append((self.plot_attitude, line))

            if is_mark:
                # Markers only, on the other plots: they are the times the pilot flagged, so they
                # earn a line everywhere. Every event everywhere would be a picket fence.
                for plot in self.plots[1:]:
                    other = pg.InfiniteLine(pos=t, angle=90, pen=MARK_PEN)
                    plot.addItem(other)
                    self._lines.append((plot, other))

    def _clear_overlays(self):
        for plot, item in self._regions + self._lines:
            plot.removeItem(item)
        self._regions = []
        self._lines = []

    # --- Timer -------------------------------------------------------------

    def refresh(self):
        """No-op. gs.py's 30 Hz timer calls this on the visible tab; replay data never changes."""
        return


def _column(records, key):
    """One field as a float array, tolerating missing and unparseable values."""
    out = np.zeros(len(records), dtype=np.float64)
    for index, record in enumerate(records):
        try:
            value = float(record.get(key, 0.0))
        except (TypeError, ValueError):
            continue
        if value == value:            # leave NaN as the 0.0 already in place
            out[index] = value
    return out
