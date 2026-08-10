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
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

import protocol as P
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

    def request_gains(self):
        self.link.request_gains(self.loop.id)

    def send_inject(self):
        mode = P.INJECT_MODES[self.mode_combo.currentIndex()][1]
        self.link.send_inject(self.loop.id, mode,
                              self.amplitude_box.value(), self.period_box.value())

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

    def refresh(self):
        """Redraws from the ring buffer. Called on the GUI thread only."""
        samples = self.ring.snapshot(self.window_s)
        self._write_csv(samples)

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
