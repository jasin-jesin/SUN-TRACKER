# Sun Tracker – Perimeter-Sensing 2-Axis (4 LDRs at the Edges)
## Overview

A two-axis solar tracker that uses four light sensors placed at the edges (left, right, top, bottom) instead of the common center cluster. This perimeter-sensing layout gives a wider baseline, reduces the “dead zone” around the cross-divider, and improves response under uneven lighting (cloud edges, reflections).

Axes: Pan (azimuth) + Tilt (elevation) via 2× servos

Sensors: 4× LDRs (or photodiodes) mounted at the perimeter

MCU: Arduino or ESP32 (both mappings provided)

Use case: Teaching, demos, small PV panel alignment, robotics projects

## Key Features

Perimeter sensor geometry: better differential signal and fewer false neutrals

Deadband + hysteresis: prevents servo chatter in steady sun

Soft limits: protect servos from mechanical stress

Quick calibration: normalize sensors so build tolerances don’t matter

Portable build: lightweight frame designed for classrooms/workshops

## Bill of Materials

1× Microcontroller (Arduino Uno/Nano or ESP32/ESP32-S3)

2× Servos (SG90/ MG90S for light loads; MG996R if mounting a small panel)

4× LDRs (5–10 k typical @ daylight) + 4× fixed resistors (10 k as divider pair)

1× Cross-frame or “plus” bracket to mount LDRs at left / right / top / bottom edges

Panel/plate to track (optional for demo)

Breadboard or PCB, wires, 5 V supply (servos powered separately from MCU 5 V/3.3 V)

Optional: 2× limit switches (end-stops), 1× OLED for angles/telemetry, buzzer

## Wiring
Voltage & Ground

Servos: power from a dedicated 5–6 V rail (capable ~1–2 A peak), shared GND with MCU

LDR dividers: connect to MCU analog inputs; top to 3.3 V/5 V (match your board), bottom to GND

Arduino (Uno/Nano) example
Signal	Pin
Left LDR (A0)	A0
Right LDR (A1)	A1
Top LDR (A2)	A2
Bottom LDR (A3)	A3
Pan Servo (PWM)	D9
Tilt Servo (PWM)	D10
ESP32 / ESP32-S3 example
Signal	Pin
Left LDR	GPIO32
Right LDR	GPIO33
Top LDR	GPIO34*
Bottom LDR	GPIO35*
Pan Servo (PWM)	GPIO14
Tilt Servo (PWM)	GPIO27

* Read-only ADC pins on many ESP32 variants—good for sensors.

Servo power tip: add a 100–470 µF electrolytic near the servo rail.

## How It Works
Perimeter Geometry

Placing sensors at the edges increases the angular separation between opposing pairs:

Horizontal error (Eₓ) = Right - Left

Vertical error (Eᵧ) = Top - Bottom

With a wider baseline, small sun misalignments produce larger |E|, giving cleaner control.

Control Loop (Differential + Deadband + Hysteresis)

Read and normalize the four sensor values (scale 0–1).

Compute errors:

Eₓ = R_norm - L_norm

Eᵧ = T_norm - B_norm

If |Eₓ| < DB and |Eᵧ| < DB → hold position (deadband).

Else, step servos toward reducing error, with step size proportional to |E| and capped by soft limits.

Apply hysteresis (slightly different engage/release thresholds) to avoid micro-oscillation.

Optional: swap step controller for PI/PID if you want smooth, continuous motion.

Calibration (5 min)

Shade Test: briefly cover each LDR; confirm the correct channel dips—fix wiring if not.

Normalization: capture a short rolling average per sensor on startup (ambient), then scale readings so all four have similar response ranges.

Deadband: start with 0.03–0.06 (3–6%); increase if servos chatter.

Hysteresis: set release ≈ 70–80% of engage threshold.

Step size: begin with 0.5–1.0° per loop at max error; cap at 3–4° to prevent overshoot.

Soft limits: e.g., Pan 20–160°, Tilt 20–140° (adjust to your mechanics).

Parameters (suggested defaults)
ADC_SAMPLES        = 8            ; moving average window
DEADBAND           = 0.05         ; 5%
HYSTERESIS_FACTOR  = 0.75
STEP_DEG_MAX       = 3.0
STEP_DEG_MIN       = 0.2
PAN_MIN, PAN_MAX   = 20°, 160°
TILT_MIN, TILT_MAX = 20°, 140°
LOOP_DELAY_MS      = 50–100

Build & Run

Wire sensors and servos; power servos from a solid 5–6 V source; share GND.

Flash the sketch (Arduino) or PlatformIO project (ESP32).

Watch serial telemetry: raw L,R,T,B → normalized → Eₓ,Eᵧ → commanded angles.

Aim a torch/sunlamp to test tracking indoors before outdoor trials.

Results & Observations

Faster lock-on versus center-cluster trackers—edge sensors produce larger differential early.

Fewer false neutrals when the sun is near divider lines.

Low-light behavior: wider baseline still yields usable error until all sensors saturate low.

Attempts & Failures (What I Fixed)

Horizontal expansion not compensated: early frames warped slightly under heat → reprinted with slots and added standoffs.

Incorrect measurements: LDR holes misaligned → updated CAD to parametric spacing.

Servo wire clearance: initial design pinched the horn/wires at extreme tilt → added wire channel and increased horn offset.

keep this section growing with photos/gifs—recruiters love the engineering story.

Folder Structure
.
├─ /src                 # code
├─ /hardware            # STL/CAD, laser files, mechanical drawings
├─ /schematics          # fritzing/EDA files & PNGs
├─ /docs                # photos, gifs, test logs
└─ README.md            # this file

Upgrade Ideas

PI/PID loop for smoother motion

RTC + geo calc fallback (track even in clouds)

Limit switches or current-based stall detect

Small OLED for angles and light levels

Weather-proof enclosure + UV-stable sensors

Troubleshooting

Chatter / twitching: increase DEADBAND or add HYSTERESIS; check servo power and add bulk capacitor.

Tracks the wrong way: swap servo direction or invert error sign for that axis.

No movement: hit soft limits? check angle caps; verify shared ground.

One sensor dominates: re-normalize; check divider resistor values match.

License

MIT (change if you prefer).

Credits

Concept & build by Jasin Jesin. Perimeter-sensing geometry, classroom-friendly design, and documentation tailored for teaching.
