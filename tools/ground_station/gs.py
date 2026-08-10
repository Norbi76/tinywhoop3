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

Run with the ground station's directory on the path:

    python3 gs.py

Connect to the drone's Wi-Fi AP first - see README.md.
"""

import json
import sys
import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

import protocol as P
from link import Link
from panels import LoopPanel, OverviewPanel

# One timer drives every redraw. Drawing from the receive thread instead would touch Qt widgets
# off the GUI thread, which is undefined behaviour and in practice crashes under load.
REDRAW_HZ = 30

# KILL sends repeatedly rather than once: the link is UDP over a SoftAP that may be exactly the
# thing going wrong at the moment the button is pressed, and a single lost datagram must not be
# the difference between the motors stopping and not.
KILL_REPEAT_MS = 500
KILL_INTERVAL_MS = 25


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

        self.tabs.currentChanged.connect(self._on_tab_changed)
        layout.addWidget(self.tabs)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(int(1000 / REDRAW_HZ))

        self.kill_timer = QTimer(self)
        self.kill_timer.timeout.connect(self._on_kill_tick)

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

        if widget is self.overview:
            self.link.send_select(P.LOOP_ALL, P.OVERVIEW_DIVIDER)
            return

        for loop in P.LOOPS:
            if self.panels[loop.id] is widget:
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

        self._update_top_bar()

    def _update_top_bar(self):
        online = self.link.is_online()
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

    # --- Arming ------------------------------------------------------------

    def _on_arm_toggled(self, checked):
        self._armed_request = checked
        self.arm_button.setText("DISARM" if checked else "ARM")
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
