# PID ground station

A desktop tuning tool for the eight cascaded PID loops on the flight controller. It plots one
loop's setpoint, measurement, error, individual P/I/D contributions and output in real time, and
reads and writes that loop's gains live over the drone's Wi-Fi AP.

USB is not an option in flight. Everything here goes over UDP through the telemetry module.

## How it connects

```
flight_controller  --UART 460800, framed--  telemetry_module  --UDP 14550, Wi-Fi--  this tool
```

The telemetry module does not interpret the tuning messages. It forwards every UART frame it
receives into a UDP datagram, and splits every inbound datagram back into UART frames. The web
dashboard on `http://192.168.4.1/` keeps working exactly as before and is completely independent
of this tool - you can run both at once.

**Peer discovery**: the module has no idea where you are until you talk to it. Any packet you send
latches your address as its telemetry peer, and a keepalive goes out once a second to keep it
fresh. That is why you can restart this tool, or drop and rejoin the Wi-Fi, without touching the
drone.

## Running it

```bash
pip install -r requirements.txt
```

Join the drone's Wi-Fi AP — the credentials are `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` in
[`telemetry_module/components/wifi_ap/include/wifi_ap.h`](../../telemetry_module/components/wifi_ap/include/wifi_ap.h),
currently `TinyWhoopAP` / `whoop12345`. Then:

```bash
python3 gs.py
```

The link indicator in the top bar goes green within a second or two of the first datagram. If it
stays red: check you are on the drone's AP and not your house network, and check the module's
console for `Ground station link listening on UDP 14550`.

## What each plot means

Selecting a loop's tab subscribes the drone's debug stream to that loop and pulls its live gains
into the editors. Three plots, X-linked so a feature lines up across all three:

1. **Setpoint and measurement** — what the loop was asked for against what it got. This is the
   plot you tune by. A measurement that lags, oscillates around, or never reaches the setpoint
   tells you which term needs work.
2. **Error** — `setpoint - measurement`, with a zero line. The error is not transmitted; it is
   computed here, because at 500 Hz those four bytes are a fifth of the link budget for something
   the receiver already knows.
3. **P, I, D and output** — the individual contributions, plus the post-clamp output. This is
   where you see *why* the loop is behaving as it does: a D term that is mostly noise means the
   derivative cutoff is too high; an I term that dominates means Ki is too large or the loop is
   winding up; an output pinned to its limit means no gain change will help until that is fixed.

The statistics panel updates about four times a second:

| Readout | Meaning |
|---|---|
| `rate` | Sample rate actually arriving, from the sample timestamps. Well below the expected figure means frames are being dropped. |
| `RMS err` / `peak err` | Tracking quality over the visible window. |
| `out sat` | Percentage of samples with `PID_FLAG_OUT_SAT` — the output hit its clamp. Anything sustained here means the loop has no authority left. |
| `I clamped` | Percentage with `PID_FLAG_I_CLAMPED` — the integrator hit its limit or anti-windup froze it. |
| `overshoot` | Peak excursion past the setpoint after a detected step, as a percentage of the step size. |
| `settle 5%` | Time from the step until the measurement stayed inside ±5% of it. |

The last two are the point of the tool. Use the **test signal** group to produce them: a *doublet*
(+A for half a period, −A for half a period, then nothing) is the one to use in the air, because it
excites the loop in both directions and leaves the aircraft where it started. A *step* gives the
cleanest numbers but displaces the drone. A *square* alternates continuously, for watching the
loop settle repeatedly.

## Why a plot can be empty

A loop publishes samples only on the iterations it actually runs, so an empty plot usually means
the loop is not executing rather than that the link is down. Check the link indicator first — if
it is green, the loop is simply not being called:

| Loop | Runs only when |
|---|---|
| `rate_*`, `ang_*` | Armed **and** throttle above the idle cutoff. `flight_control_update()` returns early on every disarmed path and while the throttle is on the floor. |
| `alt` | Altitude hold engaged **and** `nav.altitude_valid` — so the ToF must be fitted and reading. |
| `vel_x`, `vel_y` | Position hold engaged **and** `nav.velocity_valid` — so the optical flow sensor must be fitted and seeing texture. |

This is the intended behaviour, not a limitation: the outer loops self-disengage the instant their
sensor stops being trustworthy, and the inner loops are held off while disarmed so the integrators
cannot wind up against the ground.

## Tuning order

**Work from the inside out. Never tune an outer loop before the loop beneath it is solid** — an
outer loop is measuring the response of the inner loop, so tuning it against a bad inner loop just
bakes the inner loop's error into the outer loop's gains.

1. **Rate loops** (`rate_roll`, `rate_pitch`, `rate_yaw`) — props off first, then on a tether.
   Raise Kp until it oscillates, back off about 30%, then add D until the oscillation damps, then
   a little I to remove steady-state droop.
2. **Angle loops** (`ang_roll`, `ang_pitch`) — P only to begin with. These command rate setpoints,
   so their behaviour is meaningless until step 1 is done.
3. **Velocity loops** (`vel_x`, `vel_y`) — needs the optical flow sensor working and
   `velocity_valid` set.
4. **Altitude** (`alt`) — last, and needs the ToF working.

Applying gains **resets that loop's integrator to zero**. This is deliberate: raising Ki with a
stale accumulator dumps it into the output as a step, and on a 50 g quad that step is the ceiling.
It also means "apply on edit" (off by default) resets the integrator on every spinbox increment —
leave it off while flying.

A **zero in any of the three limit fields** (`I limit`, `Out limit`, `D cutoff`) means "leave this
one as it is", not "set it to zero". Kp/Ki/Kd are different — a zero gain there is applied, because
zeroing a gain is a normal thing to want. The limits are guarded because a zero output limit would
clamp that loop's output to zero for the rest of the flight, and the usual way to send one is by
accident: applying before a read-back has landed, or loading a profile that only carries kp/ki/kd.

> **Unit note:** the `alt` loop is labelled m/s here to match the loop table it was specified
> against, but the firmware runs it as a *position* loop — `altitude_setpoint` is in **metres**
> (clamped 0.05–1.20 m) and the measurement is `nav.altitude`. So on the `alt` tab an inject
> amplitude of `5.0` is five metres, not five m/s, and the axis labels read m/s for a metres
> signal. `MAX_CLIMB_RATE_MS` only governs how fast the throttle stick slews that setpoint.

## Bandwidth: why only one loop streams at full rate

The UART between the two boards is 460800 baud, 8N1, so 46080 bytes/second. A debug frame is 30
bytes of payload plus 8 bytes of framing = **38 bytes**. The status frame (61 B at 50 Hz) and the
IMU frame (44 B at 10 Hz) are always present, costing 3490 B/s before any tuning traffic.

| Case | Debug traffic | With baseline | Link used |
|---|---|---|---|
| All 8 loops at 50 Hz | 15200 B/s | 18690 B/s | **41%** |
| `rate_roll` alone at 500 Hz | 19000 B/s | 22490 B/s | **49%** |

Half the link for one loop is why the tab you are looking at is the only one streaming at rate.
Streaming all three rate loops at 500 Hz would be 57000 B/s — more than the UART can carry, and
the frames would simply be dropped at the flight controller's queue rather than arriving late.

### The overview tab is not uniform, and cannot be

The overview tab selects `PID_LOOP_ALL` with one divider. But the loops do not tick at the same
rate — the decimation counter counts *that loop's* ticks — so a single divider gives different
sample rates per loop. At the divider of 20 the overview uses:

| Loops | Loop rate | Sample rate |
|---|---|---|
| `rate_roll`, `rate_pitch`, `rate_yaw` | 1000 Hz | 50 Hz |
| `ang_roll`, `ang_pitch` | 250 Hz | 12.5 Hz |
| `alt`, `vel_x`, `vel_y` | 50 Hz | 2.5 Hz |

The overview is for spotting which loop is misbehaving, not for measuring one. Open that loop's
own tab to get it at full rate.

## Arming, and why KILL is the only reliable button

**The telemetry module sends its own control frame at 50 Hz whether or not you are here**, and the
flight controller acts on whichever frame arrived most recently. So an ARM or DISARM request from
this tool is overwritten within 20 ms by whatever the web dashboard is asking for. The ARM toggle
is advisory — use the dashboard for arming.

**KILL is different, and it is reliable**: the flight controller *latches* the kill flag. One frame
that gets through cuts the motors and keeps them cut, and the latch only clears once a frame
arrives requesting neither kill nor arm (i.e. once the dashboard's arm toggle is also off). The
button repeats for 500 ms purely as insurance against a lost datagram.

Battery reads a constant nominal voltage with no percentage. There is no ADC divider on the pack,
so `flight_control_get_status()` reports a fixed 3.8 V and the flight controller never sends a
battery frame. This tool parses both message types, so it will start reporting real numbers the
moment sensing is fitted — nothing here needs changing.

## Flight logging

The **REC** button in the top bar records the whole flight to one file:
`flight_<date>_<time>.ndjson` in the working directory. This is a different thing from the
per-loop **log to CSV** checkbox — that one captures a single PID loop for tuning, this one
captures the flight. Both can be on at once.

What goes in, and where each stream comes from:

| Record | Rate | Contents |
|---|---|---|
| `header` | once | when, the gains on every loop at the moment recording started, which tab was open |
| `status` | 50 Hz | the flight controller's own view: attitude, altitude, climb, body velocity, all four motor outputs, loop rate, mode, flags |
| `control` | 50 Hz | **what the pilot commanded** — the frame the telemetry module sent the flight controller |
| `pid` | 50 Hz | the streaming loop's setpoint, measurement, P/I/D and output, thinned from up to 500 Hz |
| `link` | 1 Hz | packet rate and online/offline, so a bad radio moment is visible as itself |
| `event` | — | arming, kill, mode and gain changes, tab selections, link transitions, your markers |
| `footer` | once | on a clean stop. **Its absence means the session died**, which the report tells you |

About 1.5 MB per minute. Newline-delimited JSON, one record per line, appended as you fly — so a
log from a session that ended in a crash is still readable right up to the moment it stopped. That
is the reason for the format, and it is why recording is **always plain text**: a gzip stream
buffers inside the compressor, so a log killed mid-flight would be entirely unreadable rather than
readable up to the last line. Gzip them afterwards if you want to keep a lot — everything that
reads a log opens `.ndjson.gz` transparently.

### The control echo, and why it needed a firmware change

The ground station used to see only what the drone *reported*. The control frame is built and
sent by the telemetry module, so nothing on this side ever knew what the pilot had asked for —
and a log showing the drone rolling right is ambiguous until you can see whether roll right was
being commanded.

`uart_link.c` now echoes every outgoing control frame to the ground station. It costs nothing on
the UART that flies the drone (the frame is already sent by then) and 1.15 kB/s on Wi-Fi. It goes
through a queue rather than straight into `gs_link_forward()`, because that function's batch
buffer is lock-free *on the condition* that only the RX task touches it.

Each echo carries its own sequence number, so the ground station can tell when command frames
were lost in transit. Gaps are recorded as `control_gap` events and the report prints them —
which matters, because the command-versus-actual section refuses to pair a status frame with a
command more than 0.2 s old. A hole in the command trace is the one thing that could make that
section draw a confident wrong conclusion, so it excludes what it cannot pair and says how much.

**A module running older firmware simply never sends these.** Everything still records; the log
just has no command trace, and the report says so instead of guessing.

### Markers

**M** — or the MARK button — stamps the current instant. Press it the moment something looks
wrong; you do not need to be looking at the screen, and the report lists markers first, so it is
where you start reading afterwards. **Shift+M** adds a typed note, which opens a modal dialog and
is therefore for use on the ground.

### Reading a log back

The **replay** tab opens one: attitude with the commanded angles dashed over it, altitude against
throttle demand, body velocity, and all four motors — X-linked, with armed periods shaded, kills
shaded red, and your markers drawn across every plot. The report sits beside the plots, and
**copy report** puts it on the clipboard.

Stopping a recording loads it into the replay tab automatically.

Without the GUI, and without needing PyQt6, numpy or pyqtgraph installed at all:

```bash
python3 logtool.py flight_20260827_181500.ndjson       # the report
python3 logtool.py flight.ndjson --slice 41.0 48.5     # just that window
python3 logtool.py flight.ndjson --events              # just the timeline
python3 logtool.py flight.ndjson --csv flight.csv      # status stream for a spreadsheet
```

### What the report actually tells you

It is built around separating causes that look identical from the air:

- **Link first.** A gap in the downlink explains everything after it, so nothing else is worth
  reading across one. It also flags the flight controller reporting its *own* control link
  unhealthy — a different link, and the one that disarms you after 300 ms.
- **How each arm session ended** — the pilot letting go, a KILL (and from which of the two places
  it can come), or a watchdog. These are indistinguishable in the status stream alone.
- **Command versus actual**, including a "hands off" figure: the mean attitude over every frame
  where roll and pitch were commanded within a degree of level. A drone holding a tilt while
  being asked to stay level is a trim or balance problem, and no gain change fixes it.
- **Motor balance.** At a steady hover the four outputs should sit close together. A spread wider
  than 15% is called out, with which corner is working hardest — that corner is the one being
  held up, so the airframe is falling towards the opposite one. Cross-check it against the drift
  direction above.
- **Sensor validity as a percentage of armed time**, next to what each flag gates. "POS HOLD did
  nothing" is almost always `velocity_valid` never being set, and that is invisible unless you
  look for it.
- **Per-loop saturation and integrator clamping.** A loop pinned at its output limit cannot be
  fixed by tuning it, and it is worth knowing that before spending an evening trying.

Where a number cannot be computed the report says so rather than printing a default that reads
like a measurement.

## Gain profiles

Gain profiles save and load all eight loops to a JSON file. **Loading only fills the editors** —
nothing is sent until you press apply on a tab, because pushing eight gain sets at once would
reset every integrator on the drone.

## Files

| File | What it holds |
|---|---|
| `protocol.py` | Struct formats, message ids and the `LOOPS` table. Mirrors `telemetry_uart.h` — change both together. |
| `link.py` | UDP socket, receive thread, keepalive thread, per-loop numpy ring buffers. |
| `panels.py` | The per-loop tuning panel, the overview panel and the replay panel. |
| `gs.py` | Main window, tab handling, top bar, recording controls, entry point. |
| `flightlog.py` | The flight recorder and the reader. No Qt, no numpy. |
| `flightreport.py` | Turns a log into the text report. Standard library only. |
| `logtool.py` | Command-line front end to the two above. |

Two invariants worth keeping if you edit this:

- **Never redraw from the receive thread.** One 30 Hz timer on the GUI thread drives everything.
- **The ring buffers are preallocated numpy arrays, not lists of tuples.** At 500 Hz a
  list-of-tuples allocates tens of thousands of short-lived objects a minute, and the resulting GC
  pauses are visible as stutter in the plots.
- **Nothing on the receive thread may block.** The flight recorder is called from the middle of
  the socket loop and only ever appends to an in-memory queue; a separate writer thread does the
  disk I/O. A stalled write there would show up as lost telemetry, not as a slow log.

`flightlog.py`, `flightreport.py` and `logtool.py` deliberately import none of PyQt6, numpy or
pyqtgraph, so a log can be read on a machine that has none of them installed.
