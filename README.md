# WLED UART Pixel Bridge

A WLED usermod that sends raw RGB frames from one ESP32 running WLED to another ESP32 over a dedicated UART. For installation in a car, place a 3.3 V differential transceiver such as a MAX3485 at each end and carry the UART as an A/B twisted pair plus a reference ground.

Targeted and reviewed against current WLED `main` behavior, including the serial/Adalight realtime path and usermod configuration API.
dasd
## What it provides

- Master and Receiver roles in the same usermod.
- RGB888 raw-pixel transport.
- Default 2,000,000 baud and up to 60 frames per second.
- CRC-16/CCITT-FALSE over every frame.
- 32-bit sequence number with gap/loss diagnostics.
- Triple buffering on both controllers.
- Latest-frame-wins queueing: old animation frames are discarded rather than shown late.
- Receiver uses WLED's serial/Adalight realtime mode and performs exactly one physical `strip.show()` per applied bridge frame.
- Brightness is transported separately, so WLED brightness and local current limiting still work on the receiver.
- Corrupt and incomplete frames are never displayed.
- Configurable startup guard keeps the Receiver dark until a stable run of valid frames is present.
- Configurable communication-loss failsafe can keep the last frame, switch to black, or fade to black.
- Configurable reacquisition guard can require stable frames before Master output is restored after a failsafe event.
- Receiver statistics expose CRC errors, framing errors, missing sequence numbers, gap events, duplicates, resyncs and error/loss rates.

## Files

Copy the complete directory:

```text
usermods/uart_pixel_bridge/
├── library.json
├── uart_pixel_bridge.cpp
└── README.md
```

Then add the usermod to your `platformio_override.ini`. For a classic ESP32 with WLED AudioReactive, use:

```ini
[platformio]
default_envs = esp32dev_V4

[env:esp32dev_V4]
custom_usermods =
  audioreactive
  uart_pixel_bridge
```

No WLED core source file needs to be changed. The current WLED build system discovers a usermod library through `custom_usermods`, and the module registers itself using `REGISTER_USERMOD`.

## Logical LED arrangement on the Master

The Master must contain the entire logical animation range, including the pixels that physically exist on the Receiver.

Example:

```text
Master physical output:  LEDs   0–99
Remote logical output:   LEDs 100–399
UART Pixel Bridge:
  source-start = 100
  led-count    = 300
```

Create a second WLED LED output of 300 pixels on an unused, unconnected GPIO so WLED renders those logical pixels. The usermod copies LEDs 100–399 and sends them over UART. The dummy output consumes WS281x output time even though its pin is not connected; eliminating that overhead would require a new WLED Bus implementation rather than a pure usermod.

## Receiver LED arrangement

Configure the Receiver's physical LED output normally. Then configure:

```text
target-start = 0
led-count    = 300
```

`led-count`, baud and framing must match on both devices. `source-start` is relevant only on the Master; `target-start` is relevant only on the Receiver.

## Suggested settings

### Master

```text
enabled:       true
role:          Master / sender
uart:          UART 1
tx pin:        17
baud:          2000000
source-start:  first logical remote pixel
led-count:     number of remote pixels
max-fps:       60
reverse:       according to physical orientation
```

### Receiver

```text
enabled:       true
role:          Receiver
uart:          UART 1
rx pin:        16
baud:          2000000
target-start:  0
led-count:     same as Master
max-fps:       ignored
timeout-ms:    500
startup-guard: true
startup-valid-frames: 2
failsafe: true
failsafe-hold-ms: 500
failsafe-action: Fade to black after hold
failsafe-fade-ms: 500
reacquire-guard: true
reacquire-valid-frames: 2
```

Do not configure the chosen UART pins as LED outputs, buttons, relays, I2C, or other WLED functions.


## Receiver startup and communication-loss policy (v1.4.0)

All controls below are available in **Config -> Usermods -> UART Pixel Bridge**. They only affect Receiver role.

```text
startup-guard
  true  = keep the controlled LED range black after boot until startup-valid-frames consecutive valid frames arrive
  false = do not suppress the Receiver's normal WLED output while waiting for the Master

startup-valid-frames
  1..20 consecutive CRC-valid, sequence-continuous frames required before the first Master frame is displayed

failsafe
  true  = the bridge owns communication-loss behavior
  false = fall back to the older WLED realtime timeout behavior

failsafe-hold-ms
  0..60000 ms to keep the last valid Master frame before running the configured failsafe action

failsafe-action
  Keep last frame          = remain on the final Master frame indefinitely
  Black after hold         = switch the controlled range to black after failsafe-hold-ms
  Fade to black after hold = linearly fade the final Master frame to black

failsafe-fade-ms
  0..10000 ms fade duration; only used by Fade to black after hold

reacquire-guard
  true  = after a failsafe event, require reacquire-valid-frames stable frames before showing Master data again
  false = display the first valid frame immediately when communication returns

reacquire-valid-frames
  1..20 consecutive valid frames required after a failsafe event
```

Recommended automotive defaults are the values shown in the Receiver example above: 2 startup frames, 500 ms hold, 500 ms fade, and 2 reacquisition frames. At roughly 40-45 FPS, requiring two frames adds only about 45-50 ms after the Master becomes stable.

When startup guard is enabled, the Receiver enters WLED's serial realtime mode before allowing normal effects to reach the controlled output and explicitly paints the controlled range black. This avoids a boot-time flash of a local Receiver preset. When failsafe is enabled, the usermod keeps the realtime lock refreshed while communication is absent so WLED does not briefly restore a local effect between hold/fade/black states.

If either feature is disabled, that part of the old behavior is restored. In particular, with `failsafe: false`, loss of bridge traffic is again governed by `timeout-ms`, after which WLED can return to its local state.

## MAX3485 wiring: one-way link

This usermod implements the required pixel direction: Master to Receiver. The MAX3485 direction pins can therefore be fixed in hardware.

### Master module

```text
ESP32 TX pin → DI
3.3 V        → VCC
GND          → GND
3.3 V        → DE
3.3 V        → /RE
```

### Receiver module

```text
RO           → ESP32 RX pin
3.3 V        → VCC
GND          → GND
GND          → DE
GND          → /RE
```

### Cable

```text
Master A   ───────── Receiver A
Master B   ───────── Receiver B
Master GND ───────── Receiver GND
```

Use A/B as a twisted pair. If the modules use opposite A/B naming conventions and no frames are received, swap A and B at one end.

At 2 Mbaud, terminate the two physical ends of the bus with approximately 120 ohms between A and B. Check whether the modules already contain or expose a jumper for the termination resistor before adding another one.

## Protocol

All multibyte integers are little-endian.

```text
Offset  Size  Field
0       4     Magic: "WPB1"
4       1     Protocol version: 1
5       1     Flags; bit 0 reports Master reverse mode
6       4     Sequence number
10      2     LED count
12      1     WLED brightness
13      1     Reserved
14      2     Payload length in bytes
16      3*N   RGB888 payload
16+3*N  2     CRC-16/CCITT-FALSE
```

The CRC starts at the protocol-version byte and ends at the final payload byte. Magic and the CRC field itself are excluded. Parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.

## Diagnostics and bench testing

The Receiver tracks transport quality without printing continuously to Serial, so diagnostics do not disturb timing. The counters are available on the WLED Info page and in `/json/state` under `UART Pixel Bridge`.

Important fields:

```text
received                  valid frames whose CRC passed
applied                   valid frames actually shown (one physical show per applied frame)
dropped                   queued frames discarded locally to keep latency low
crc-errors                complete frames rejected by CRC-16
framing-errors            invalid protocol headers/lengths
sequence-gap-events       number of forward jumps in the 32-bit sequence
missing-sequences         total sequence numbers absent between valid frames
duplicate-sequences       repeated sequence numbers
sequence-resyncs          large backwards jumps, normally a Master reboot/reset
crc-error-rate-pct        CRC failures / (valid frames + CRC failures)
sequence-loss-rate-pct    missing sequence numbers / (valid frames + missing)
last-received-sequence    newest valid frame received
last-applied-sequence     newest frame actually displayed
stats-age-ms              time since statistics were reset
```

`missing-sequences` is deliberately broader than `crc-errors`: a frame with a bad magic/header may never reach CRC validation, and a frame can also disappear on the Master before transmission if its queue falls behind. Compare the Receiver's `missing-sequences` with the Master's `dropped` counter when diagnosing the physical link.

Statistics can be reset without rebooting by posting this WLED state JSON to the device:

```json
{
  "UART Pixel Bridge": {
    "reset-stats": true
  }
}
```

For a clean long-duration test, reset statistics on both devices, run the same effect/FPS for several hours, then compare Master `sent`/`dropped` with Receiver `received`, `crc-errors`, `missing-sequences` and `dropped`.


### Receiver single-show timing (v1.3.0)

The Receiver intentionally uses `REALTIME_MODE_ADALIGHT`, matching WLED's native serial realtime path. Do not change this to `REALTIME_MODE_GENERIC`: current WLED calls `strip.show()` from inside `realtimeLock()` for Generic mode. Since the bridge also calls `strip.show()` after copying the received pixels, Generic mode causes two physical LED transmissions per received frame. On a 700-pixel WS281x output that is roughly two 21 ms wire times and limits output to about 20–24 FPS with a periodic cadence hitch.

With the serial/Adalight realtime mode, `realtimeLock()` only refreshes the realtime timeout; the bridge performs the single `strip.show()` after the frame is copied. This keeps WLED's framebuffer authoritative while avoiding the duplicate physical show.

## Capacity examples at 2 Mbaud

UART 8N1 uses approximately 10 line bits per payload byte.

```text
300 LEDs: 900 RGB bytes; approximately 4.6 ms plus header
500 LEDs: 1500 RGB bytes; approximately 7.6 ms plus header
1000 LEDs: 3000 RGB bytes; approximately 15.1 ms plus header
```

A single WS2812/WS2815 output itself needs approximately 30 microseconds per pixel, so 500 pixels require about 15 ms to reach the LEDs. For that size, the LED output is generally a tighter frame-rate limit than the 2 Mbaud UART.

## Failure behavior

- Bad magic or invalid lengths: frame discarded.
- CRC mismatch: frame discarded and the consecutive-valid-frame streak is reset.
- UART falls behind during normal active playback: only the latest queued frame is retained.
- With startup guard enabled: controlled LEDs remain black until the configured valid-frame threshold is reached.
- With failsafe enabled: communication loss follows the configured hold/action/fade policy and local Receiver effects remain suppressed.
- With failsafe disabled: WLED's normal `timeout-ms` realtime expiry behavior is used.
- With reacquisition guard enabled: bridge output resumes only after the configured number of consecutive valid frames.
- Logical source range extends past the Master's total LED count: missing pixels are transmitted as black.

## Notes

- This is intentionally a one-way pixel data link. A second differential pair can later be added for telemetry, but it is not required for reliable LED output.
- Both controllers should run the same WLED build and the same version of this usermod.
- Use automotive power protection, local decoupling and a proper 3.3 V regulator. The MAX3485 link does not protect the ESP32 power rail from vehicle transients.

## v1.2.1 receiver stability fix

Version 1.2.1 disables the experimental direct `BusManager` receiver path introduced in v1.2.0. Direct bus writes bypass WLED's WS2812FX realtime framebuffer, so any normal WLED show path can repaint the physical bus from stale framebuffer contents and produce rapid visible flicker.

The Receiver now always uses WLED's native realtime path:

- `strip.setRealtimePixelColor(...)`
- `strip.show()`

The larger UART RX driver buffer, triple frame buffers, CRC/sequence diagnostics, and timing statistics from v1.2.0 are retained.

`/json/state` keeps `fast-receiver: false` for backwards compatibility and adds `receiver-output: "wled-realtime"`.


## v1.4.0 startup/failsafe state machine

The Receiver now exposes a small state machine in `/json/state`:

```text
waiting   startup guard is waiting for stable Master frames
active    normal Master frames are being displayed
holding   communication was lost and the last frame is being retained
fading    the configured fade-to-black is in progress
blackout  communication was lost and the controlled range is black
```

Additional diagnostics include `first-valid-frame-ms`, `last-frame-age-ms`, `consecutive-valid-frames`, `failsafe-events`, `startup-frames-suppressed`, `reacquire-frames-suppressed`, and `failsafe-shows`.
