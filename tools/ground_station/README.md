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

## Logging

The **log to CSV** toggle on each loop writes `log_<loop>_<timestamp>.csv` in the working
directory, with a header row and one row per sample: `t_s, setpoint, measurement, error, p, i, d,
output, flags`. Logging follows what is being streamed, so only the visible loop is captured.

Gain profiles save and load all eight loops to a JSON file. **Loading only fills the editors** —
nothing is sent until you press apply on a tab, because pushing eight gain sets at once would reset
every integrator on the drone.

## Files

| File | What it holds |
|---|---|
| `protocol.py` | Struct formats, message ids and the `LOOPS` table. Mirrors `telemetry_uart.h` — change both together. |
| `link.py` | UDP socket, receive thread, keepalive thread, per-loop numpy ring buffers. |
| `panels.py` | The per-loop tuning panel and the overview panel. |
| `gs.py` | Main window, tab handling, top bar, entry point. |

Two invariants worth keeping if you edit this:

- **Never redraw from the receive thread.** One 30 Hz timer on the GUI thread drives everything.
- **The ring buffers are preallocated numpy arrays, not lists of tuples.** At 500 Hz a
  list-of-tuples allocates tens of thousands of short-lived objects a minute, and the resulting GC
  pauses are visible as stutter in the plots.
