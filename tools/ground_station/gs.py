#!/usr/bin/env python3
"""Ground station main window and entry point.

WHAT THIS FILE DOES
  Owns the window, the tab bar (one tab per PID loop plus an overview), the top status bar, the
  gain-profile save/load, and - importantly - THE SINGLE REDRAW TIMER.

HOW THE PIECES FIT
  link.py runs a receive thread that only writes into numpy ring buffers and emits Qt signals.
  This file's 30 Hz timer is the only thing that reads those buffers and repaints. That split is
  what keeps a 500 Hz sample stream from turning into 500 widget updates a second.

  Switching tabs is not just a UI change: it sends a PID_SELECT to the drone, so only the loop you
  are looking at streams at full rate. See README.md's bandwidth section for why that is necessary
  rather than merely tidy.

FLIGHT RECORDING
  The REC button starts a flightlog.FlightRecorder and hands it to the link, which then feeds it
  every frame as it arrives. Everything discrete that happens in this window - arming, kill, mode
  and gain changes, tab selections, markers, the link going up and down - is written into the same
  file as an event, so the log explains itself without needing this session's memory.

  Recording is deliberately INDEPENDENT of the per-loop "log to CSV" checkbox. That one captures
  one PID loop for tuning; this one captures the flight. Both can be on at once.

Run with the ground station's directory on the path:

    python3 gs.py

Connect to the drone's Wi-Fi AP first - see README.md.
"""

import json
import os
import sys
import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QInputDialog, QLabel, QMainWindow, QMessageBox,
    QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

import flightlog
import protocol as P
from link import Link
from panels import LoopPanel, OverviewPanel, ReplayPanel

# One timer drives every redraw. Drawing from the receive thread instead would touch Qt widgets
# off the GUI thread, which is undefined behaviour and in practice crashes under load.
REDRAW_HZ = 30

# KILL sends repeatedly rather than once: the link is UDP over a SoftAP that may be exactly the
# thing going wrong at the moment the button is pressed, and a single lost datagram must not be
# the difference between the motors stopping and not.
KILL_REPEAT_MS = 500
KILL_INTERVAL_MS = 25

# The recording readout is refreshed at this rate rather than at REDRAW_HZ. It shows elapsed time
# and file size, neither of which is worth a stat() call thirty times a second.
RECORD_STATUS_INTERVAL_S = 0.5

# Stopping a recording opens it in the replay tab, but parsing and plotting happen on the GUI
# thread. At the documented ~1.5 MB/minute this is roughly a five-minute flight; past it the log
# is opened by hand instead of freezing the window for several seconds.
REPLAY_AUTOLOAD_MAX_BYTES = 8 * 1024 * 1024


class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("tinywhoop PID ground station")
        self.resize(1400, 900)

        self.link = Link()
        self.link.gains_received.connect(self._on_gains_received)
        self.link.status_text.connect(self._on_status_text)

        self._status_message = ""
        self._status_message_time = 0.0
        self._kill_deadline = 0.0
        self._armed_request = False

        self._recorder = None
        self._last_record_status = 0.0
        self._was_online = None        # None until the first tick, so the first state is not
                                       # reported as a transition

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.addWidget(self._build_top_bar())

        self.tabs = QTabWidget()
        self.panels = {}
        for loop in P.LOOPS:
            panel = LoopPanel(loop, self.link)
            self.panels[loop.id] = panel
            self.tabs.addTab(panel, loop.name)

        self.overview = OverviewPanel(self.link)
        self.tabs.addTab(self.overview, "overview")

        self.replay = ReplayPanel()
        self.tabs.addTab(self.replay, "replay")

        self.tabs.currentChanged.connect(self._on_tab_changed)
        layout.addWidget(self.tabs)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(1000 / REDRAW_HZ))

        self.kill_timer = QTimer(self)
        self.kill_timer.timeout.connect(self._on_kill_tick)

        # M anywhere in the window drops a marker. A flight worth marking is a flight where you
        # are not looking at the screen, so it must not depend on hitting a particular button.
        self._mark_shortcut = QShortcut(QKeySequence("M"), self)
        self._mark_shortcut.activated.connect(self.mark)

        # Shift+M is the same marker with a typed note. Separate binding on purpose: it opens a
        # modal dialog, which is the last thing wanted while the drone is in the air, so it must
        # not be reachable by the key you are mashing one-handed.
        self._note_shortcut = QShortcut(QKeySequence("Shift+M"), self)
        self._note_shortcut.activated.connect(self.annotate)

        # Subscribe to whatever tab opened first.
        self._on_tab_changed(self.tabs.currentIndex())

    # --- Construction ------------------------------------------------------

    def _build_top_bar(self):
        bar = QWidget()
        layout = QHBoxLayout(bar)

        self.link_label = QLabel("LINK")
        self.link_label.setFont(QFont("monospace", 11, QFont.Weight.Bold))
        layout.addWidget(self.link_label)

        self.rate_label = QLabel("0 pkt/s")
        self.rate_label.setFont(QFont("monospace", 10))
        layout.addWidget(self.rate_label)

        self.battery_label = QLabel("battery --")
        self.battery_label.setFont(QFont("monospace", 10))
        layout.addWidget(self.battery_label)

        self.state_label = QLabel("state --")
        self.state_label.setFont(QFont("monospace", 10))
        layout.addWidget(self.state_label)

        self.record_button = QPushButton("REC")
        self.record_button.setCheckable(True)
        self.record_button.setFixedHeight(30)
        self.record_button.setMinimumWidth(70)
        self.record_button.toggled.connect(self._on_record_toggled)
        layout.addWidget(self.record_button)

        self.mark_button = QPushButton("MARK (m)")
        self.mark_button.setToolTip(
            "Stamp this instant in the flight log. Press it the moment something looks wrong -\n"
            "the report lists markers first, so it is where you look afterwards.\n"
            "M marks silently; Shift+M asks for a note (on the ground - it is a modal dialog).")
        self.mark_button.clicked.connect(self.mark)
        layout.addWidget(self.mark_button)

        self.record_label = QLabel("not recording")
        self.record_label.setFont(QFont("monospace", 10))
        layout.addWidget(self.record_label)

        layout.addStretch(1)

        profile_save = QPushButton("save profile")
        profile_save.clicked.connect(self.save_profile)
        layout.addWidget(profile_save)

        profile_load = QPushButton("load profile")
        profile_load.clicked.connect(self.load_profile)
        layout.addWidget(profile_load)

        read_all = QPushButton("read all from drone")
        read_all.clicked.connect(self.read_all_gains)
        layout.addWidget(read_all)

        self.arm_button = QPushButton("ARM")
        self.arm_button.setCheckable(True)
        self.arm_button.toggled.connect(self._on_arm_toggled)
        layout.addWidget(self.arm_button)

        kill_button = QPushButton("KILL")
        kill_button.setFixedHeight(44)
        kill_button.setMinimumWidth(120)
        kill_button.setStyleSheet(
            "QPushButton { background-color: #c00000; color: white; font-weight: bold; "
            "font-size: 16px; }")
        kill_button.clicked.connect(self.kill)
        layout.addWidget(kill_button)

        return bar

    # --- Streaming ---------------------------------------------------------

    def _on_tab_changed(self, index):
        """Subscribes the drone's debug stream to whatever is now visible.

        Only one selection is live at a time. This is a bandwidth decision, not a limitation of
        the protocol - see the README: one rate loop at 500 Hz is already about half the UART.
        """
        widget = self.tabs.widget(index)

        # The replay tab is not live and must not change what the drone is streaming: switching
        # to it to look at a previous flight should not silently retune what the next one records.
        if widget is self.replay:
            return

        if widget is self.overview:
            self._record_event("loop_selected", loop="overview",
                               divider=P.OVERVIEW_DIVIDER)
            self.link.send_select(P.LOOP_ALL, P.OVERVIEW_DIVIDER)
            return

        for loop in P.LOOPS:
            if self.panels[loop.id] is widget:
                self._record_event("loop_selected", loop=loop.name, divider=loop.divider)
                self.link.send_select(loop.id, loop.divider)
                # Pull the live gains in as well, so the spinboxes show what the drone actually
                # has rather than whatever was last typed into them.
                self.link.request_gains(loop.id)
                return

    # --- Periodic ----------------------------------------------------------

    def _on_tick(self):
        # Only the visible tab redraws. Drawing all nine would spend most of a 30 Hz budget on
        # curves nobody is looking at.
        current = self.tabs.currentWidget()
        if current is not None:
            current.refresh()

        # CSV logging, however, is drained for EVERY panel regardless of what is on screen.
        # A loop keeps streaming while you look at the overview or replay tab, and its ring only
        # holds a few seconds, so leaving the write to refresh() punched silent holes in the file.
        # Costs one attribute test per panel when nothing is logging.
        for panel in self.panels.values():
            panel.pump_csv()

        self._update_top_bar()
        self._update_record_status()

    def _update_top_bar(self):
        online = self.link.is_online()

        # Link transitions are recorded as events. Reading a log, "everything went strange here"
        # and "the link dropped here" must not be two separate deductions.
        if self._was_online is not None and online != self._was_online:
            self._record_event("link_up" if online else "link_down")
        self._was_online = online

        recorder = self._recorder
        if recorder is not None:
            recorder.link(self.link.packets_per_second, self.link.samples_in,
                          self.link.packets_in, online)
        self.link_label.setText("LINK OK" if online else "NO LINK")
        self.link_label.setStyleSheet(
            "color: #00a000;" if online else "color: #c00000;")

        self.rate_label.setText(f"{self.link.packets_per_second:5.1f} pkt/s")

        # The flight controller has no battery divider fitted: its status frame carries a fixed
        # nominal voltage and it never sends a BATTERY frame. Both are parsed anyway, so this
        # starts reporting real numbers the moment sensing is added.
        if self.link.battery is not None:
            voltage = self.link.battery["battery_voltage"]
            percent = self.link.battery["battery_percent"]
            self.battery_label.setText(f"battery {voltage:4.2f} V  {percent:3d} %")
        elif self.link.status is not None:
            voltage = self.link.status["battery_voltage"]
            self.battery_label.setText(f"battery {voltage:4.2f} V  (nominal)")

        status = self.link.status
        if status is not None:
            killed = bool(status["flags"] & P.STATUS_FLAG_KILLED)
            state = "KILLED" if killed else ("ARMED" if status["armed"] else "disarmed")
            self.state_label.setText(f"state {state}  loop {status['loop_hz']} Hz")

        if self._status_message and (time.monotonic() - self._status_message_time) < 5.0:
            self.statusBar().showMessage(self._status_message)

    def _on_status_text(self, text):
        self._status_message = text
        self._status_message_time = time.monotonic()

    def _on_gains_received(self, gains):
        panel = self.panels.get(gains["loop_id"])
        if panel is not None:
            panel.set_gain_values(gains)

    # --- Flight recording --------------------------------------------------

    def _on_record_toggled(self, checked):
        if checked:
            self._start_recording()
        else:
            self._stop_recording("stopped by the operator")

    def _start_recording(self):
        path = flightlog.default_path()
        try:
            recorder = flightlog.FlightRecorder(path, header_extra={
                "tool": "gs.py",
                "module_address": self.link.host,
                "udp_port": self.link.port,
                # What was on screen when recording started, so the PID stream in the log has a
                # name attached without having to infer it from the samples.
                "selected_tab": self.tabs.tabText(self.tabs.currentIndex()),
                "gains_at_start": {loop.name: self.panels[loop.id].gain_values()
                                   for loop in P.LOOPS},
            })
        except OSError as error:
            # Opening the file failed - a full disk, a read-only directory. Fail loudly and put
            # the button back, rather than showing REC while nothing is being written.
            QMessageBox.warning(self, "Could not start recording", str(error))
            self.record_button.setChecked(False)
            return

        self._recorder = recorder
        # Published to the link LAST, so the receive thread never sees a half-built recorder.
        self.link.recorder = recorder

        recorder.event("recording_started", file=os.path.basename(path))
        recorder.event("link", online=self.link.is_online())

        # Pull every loop's gains from the drone so the log records what was ACTUALLY flying,
        # not what happens to be sitting in the spinboxes. They arrive as gains_read events.
        self.read_all_gains()

        self.record_button.setText("REC \u25cf")
        self.record_button.setStyleSheet(
            "QPushButton { background-color: #c00000; color: white; font-weight: bold; }")
        self._on_status_text(f"recording to {os.path.basename(path)}")

    def _stop_recording(self, reason, load_replay=True):
        recorder = self._recorder
        if recorder is None:
            return

        # Unpublish FIRST so the receive thread stops handing it records, then close.
        self.link.recorder = None
        self._recorder = None

        recorder.event("recording_stopped", reason=reason)
        recorder.stop(reason)

        written, dropped, size = recorder.stats()
        self.record_button.setText("REC")
        self.record_button.setStyleSheet("")
        self.record_label.setText("not recording")

        message = (f"saved {os.path.basename(recorder.path)} - "
                   f"{written} records, {size / 1e6:.1f} MB")
        if dropped:
            message += f", {dropped} DROPPED (disk too slow)"
        self._on_status_text(message)

        # Offer it straight to the replay tab: the point of recording is looking at it afterwards,
        # and making that free is the difference between doing it and not.
        #
        # Not unconditionally, though. Parsing and plotting a long flight takes seconds on the GUI
        # thread, and there are two moments where that is unacceptable: quitting (the user is gone
        # and the window would hang), and the mid-flight auto-stop on a write error (the drone may
        # still be in the air, and this thread owns the KILL button). Both pass load_replay=False.
        if load_replay and size <= REPLAY_AUTOLOAD_MAX_BYTES:
            self.replay.load(recorder.path)
        elif load_replay:
            self._on_status_text(
                f"{message} - too large to open automatically, use the replay tab's open button")

    def mark(self):
        """Stamps the current instant in the log. Bound to the MARK button and to M."""
        recorder = self._recorder
        if recorder is None:
            self._on_status_text("not recording - nothing to mark")
            return

        recorder.event("mark", text="")
        self._on_status_text(f"marked t={recorder.t():.1f} s")

    def annotate(self):
        """Adds a marker with a typed note. For use on the ground - mark() is the in-flight one."""
        recorder = self._recorder
        if recorder is None:
            return
        text, accepted = QInputDialog.getText(self, "Add a note", "Note for this instant:")
        if accepted:
            recorder.event("mark", text=text)

    def _record_event(self, kind, **fields):
        """Records an event if recording, and does nothing if not. Every hook goes through this."""
        recorder = self._recorder
        if recorder is not None:
            recorder.event(kind, **fields)

    def _update_record_status(self):
        recorder = self._recorder
        if recorder is None:
            return

        now = time.monotonic()
        if (now - self._last_record_status) < RECORD_STATUS_INTERVAL_S:
            return
        self._last_record_status = now

        written, dropped, size = recorder.stats()
        elapsed = recorder.t()
        text = f"REC {int(elapsed) // 60:d}:{int(elapsed) % 60:02d}  {size / 1e6:5.1f} MB"
        if dropped:
            text += f"  {dropped} DROPPED"
        self.record_label.setText(text)

        error = recorder.error()
        if error is not None:
            # A write error means the log is no longer trustworthy. Stop rather than keep a REC
            # indicator on over a file that is not growing.
            self._on_status_text(f"recording error: {error}")
            # Straight to _stop_recording rather than through the button, so the replay load is
            # skipped: this can fire while the drone is flying and this thread runs the KILL timer.
            self.record_button.blockSignals(True)
            self.record_button.setChecked(False)
            self.record_button.blockSignals(False)
            self._stop_recording(f"write error: {error}", load_replay=False)

    # --- Arming ------------------------------------------------------------

    def _on_arm_toggled(self, checked):
        self._armed_request = checked
        self.arm_button.setText("DISARM" if checked else "ARM")
        self._record_event("arm_request" if checked else "disarm_request", source="ground station")
        self.link.send_control(armed=1 if checked else 0)
        if checked:
            self._on_status_text(
                "arm requested - the dashboard's own 50 Hz frames override this, see README")

    def kill(self):
        """Latches the flight controller's kill.

        Unlike the arm request, this one sticks: the flight controller LATCHES the kill flag and
        only releases it once a frame arrives asking for neither kill nor arm. So a single frame
        that gets through is enough, and repeating for 500 ms is purely insurance against loss.
        """
        self.arm_button.setChecked(False)
        self._record_event("kill", source="ground station")
        self._kill_deadline = time.monotonic() + KILL_REPEAT_MS / 1000.0
        self.kill_timer.start(KILL_INTERVAL_MS)
        self._on_kill_tick()
        self._on_status_text("KILL sent")

    def _on_kill_tick(self):
        self.link.send_control(armed=0, flags=P.CTRL_FLAG_KILL)
        if time.monotonic() >= self._kill_deadline:
            self.kill_timer.stop()

    # --- Gain profiles -----------------------------------------------------

    def read_all_gains(self):
        """Asks the drone for all eight loops' gains.

        Spaced out rather than sent as a burst: the telemetry module relays a handful of uplink
        frames per 20 ms TX cycle, so eight at once would partly land in the next cycle anyway.
        """
        for index, loop in enumerate(P.LOOPS):
            QTimer.singleShot(index * 60, lambda l=loop: self.link.request_gains(l.id))
        self._on_status_text("requested gains for all 8 loops")

    def save_profile(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save gain profile", "gains.json",
                                              "JSON (*.json)")
        if not path:
            return

        profile = {loop.name: self.panels[loop.id].gain_values() for loop in P.LOOPS}
        try:
            with open(path, "w") as handle:
                json.dump(profile, handle, indent=2, sort_keys=True)
        except OSError as error:
            QMessageBox.warning(self, "Save failed", str(error))
            return

        self._on_status_text(f"saved {path}")

    def load_profile(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load gain profile", "", "JSON (*.json)")
        if not path:
            return

        try:
            with open(path) as handle:
                profile = json.load(handle)
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, "Load failed", str(error))
            return

        # Loaded into the spinboxes only. Nothing is sent until "apply" is pressed on a tab -
        # pushing eight gain sets at once would reset every integrator on the drone, which is not
        # something to do by accident while it is in the air.
        for loop in P.LOOPS:
            if loop.name in profile:
                self.panels[loop.id].set_gain_values(profile[loop.name])

        self._on_status_text(f"loaded {path} into the editors - press apply per loop to send")

    # --- Shutdown ----------------------------------------------------------

    def closeEvent(self, event):
        # Stopped before the link closes, so the footer is written and the log reads as complete.
        # A log without a footer means the session died; quitting normally is not that.
        self._stop_recording("ground station closed", load_replay=False)
        for panel in self.panels.values():
            panel._close_csv()
        self.link.close()
        super().closeEvent(event)


def main():
    application = QApplication(sys.argv)
    application.setAttribute(Qt.ApplicationAttribute.AA_DontUseNativeMenuBar, False)

    window = GroundStation()
    window.show()

    return application.exec()


if __name__ == "__main__":
    sys.exit(main())
