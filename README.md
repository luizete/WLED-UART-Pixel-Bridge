# WLED UART Pixel Bridge

A WLED usermod that sends raw RGB frames from one ESP32 running WLED to another ESP32 over a dedicated UART. For installation in a car, place a 3.3 V differential transceiver such as a MAX3485 at each end and carry the UART as an A/B twisted pair plus a reference ground. However, just a single wire between RX and TX will likely work fine.

Targeted and reviewed against WLED 16.0.1, including the serial/Adalight realtime path and usermod configuration API.

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
default_envs = esp32dev

[env:esp32dev]
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

Baud and framing must match on both devices. `target-start` is relevant only on the Receiver.

## Sending the whole strip (v1.5.0)

With `send-all` enabled on the Master, it ignores its own `source-start`, `led-count` and `reverse` and sends every LED of the strip (`strip.getLengthTotal()`, at most 2048). The bridge restarts by itself when the WLED LED configuration changes the total. Every Receiver then cuts its own window out of the frame:

```text
Receiver:
  source-start = first LED of the received frame used here
  led-count    = LEDs taken from the frame (buffers are sized for this only)
  target-start = first physical LED written
  reverse      = reverse this window when applying
```

A frame that does not contain `source-start + led-count` LEDs counts as a framing error and is not shown. If a frame arrives with the reverse flag set (older Master, or `send-all` off with `reverse` on), the Receiver undoes it before cutting its window; the output is reversed when exactly one of the frame flag and the Receiver `reverse` is set.

Frame size is `18 + 3·N` bytes at 10 line bits per byte, so keep `max-fps ≤ baud / (10 · (18 + 3·N))`; above that the Master drops frames (`dropped` rises). At 2 Mbaud: 600 LEDs → 110 fps, 1000 → 66 fps, 2048 → 32 fps.

**Upgrade order:** update the Receivers first (with `source-start = 0` they behave as before against an old Master), then the Master, enabling `send-all` and disabling `reverse`, and set `source-start`/`reverse` on each Receiver. A v1.4.0 Receiver rejects every frame larger than its `led-count`.

## Command input (v1.6.0)

A second, independent UART accepts commands from an external controller, for example a Raspberry Pi Pico that reads ACC, lock/unlock and door signals and selects the matching preset or playlist. It works in both roles, but connect it to the **Master**: the Master renders everything and the Receiver mirrors its frames, so a command on the Receiver is hidden while bridge frames arrive.

```text
cmd-enabled   true
cmd-uart      UART 2 (must differ from the bridge UART; ESP32/ESP32-S3 have UART 1 and 2)
cmd-pin 0     RX pin  ← controller TX
cmd-pin 1     TX pin  → controller RX (optional, -1 = no replies)
cmd-baud      115200
cmd-checksum  false (true = reject lines without *HH)
```

The command port restarts by itself when the settings are saved. 3.3 V controllers such as the Pico connect directly (TX→RX, RX→TX, common GND) over a short wire; use a transceiver pair for long runs, as for the bridge.

### Format

One command per line, terminated by `\n` (`\r\n` is accepted). A line starting with `{` is the WLED JSON API, the same object accepted by `POST /json/state`; anything else is a text command (case-insensitive).

```text
PING                 → PONG
PS 5 / PL 5          apply preset or playlist 5 (1–250); restarts it even if it is already running
ON / OFF / TOGGLE
BRI 128              global brightness 0–255
FX 9                 effect on the selected segments
PAL 6 / SX 128 / IX 200
COL FF8000           primary color of the selected segments
NEXT                 next entry of the running playlist
STATE                → {"on":true,"bri":128,"ps":5,"pl":5,"fx":9}
{"ps":5}             any WLED JSON state, e.g.
{"playlist":{"ps":[1,2,3],"dur":[50,50,50],"transition":7,"repeat":0,"end":4}}
```

Every line gets one reply on `cmd-pin 1`: `OK`, `PONG`, the `STATE` object, or `ERR <reason>` (`unknown command`, `json`, `busy`, `checksum`, `line too long`, a range message). After boot the usermod sends `READY`, so the controller can resend the current scene after an ESP32 reset.

The simplest design is to keep the logic in WLED presets and have the controller only pick IDs, for example: preset 10 = unlock welcome playlist, 11 = lock goodbye, 12 = ACC on, 13 = off. Presets and playlists are edited in the WLED UI without touching the controller firmware.

### Optional checksum

Append `*HH`, the XOR of every byte before the `*` in two hex digits (NMEA style): `PS 10*22`. A line with a wrong checksum is rejected; with `cmd-checksum` enabled, a line without one is rejected too. Replies to a checksummed line carry their own checksum. Partial lines are dropped after 250 ms without data, and lines over 1024 bytes are rejected; sending a bare `\n` before each command clears any noise left on the line.

### Pi Pico example (MicroPython)

```python
from machine import UART, Pin
import time

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=300)

def send(cmd):
    cs = 0
    for c in cmd.encode():
        cs ^= c
    uart.write(b"\n%s*%02X\n" % (cmd.encode(), cs))
    return uart.readline() or b""      # e.g. b"OK*04\n"; b"" on timeout

while not send("PING").startswith(b"PONG"):   # wait until WLED is up
    time.sleep(0.5)
send("PS 10")                                  # unlock welcome
```

The usermod's Info page and `/json/state` report `cmd-running`, `cmd-error`, `cmd-ok`, `cmd-errors`, `cmd-checksum-errors`, `cmd-overflows`, `cmd-last` and `cmd-last-age-ms`; `reset-stats` also clears these counters.

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

**Gamma (WLED 16):** frames carry WLED's framebuffer colors, before gamma correction and without master brightness (the same values as the web live preview). On the Receiver, uncheck **Config → Sync Interfaces → Disable realtime gamma correction** (checked by default) and keep the color gamma setting equal to the Master's; otherwise the Receiver shows washed-out colors. `/json/state` reports `realtime-gamma: true` when the Receiver applies gamma.


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

### Segment table frame (`WPS1`)

A running Master also sends WLED's active segment table on the same UART, whenever it changes and at least once per second. It uses the same header layout and CRC, with its own sequence counter (so it never opens gaps in the `WPB1` sequence). Receivers ignore it.

```text
Offset  Size  Field
0       4     Magic: "WPS1"
4       1     Protocol version: 1
5       1     Flags; bit 0 = WLED on (global brightness > 0)
6       4     Sequence number (independent of WPB1)
10      2     Segment count S (active segments only; at most 32, or 64 with PSRAM)
12      1     WLED brightness
13      1     Reserved
14      2     Payload length = 7*S
16      7*S   Per segment: id (1), start (2), stop (2, exclusive), on (1), opacity (1)
16+7*S  2     CRC-16/CCITT-FALSE
```

`id` is the index in WLED's segment list (the `id` in `/json/state`); `start`/`stop` are logical indices in the same space as the `WPB1` pixels.

## Diagnostics and bench testing

The Receiver tracks transport quality without printing continuously to Serial, so diagnostics do not disturb timing. The counters are available on the WLED Info page and in `/json/state` under `UART Pixel Bridge`.

Important fields:

```text
received                  valid frames whose CRC passed
applied                   valid frames actually shown (one physical show per applied frame)
dropped                   queued frames discarded locally to keep latency low
crc-errors                complete frames rejected by CRC-16
framing-errors            invalid protocol headers/lengths, or a frame without this Receiver's window
stream-led-count          LEDs per frame (Master: sent; Receiver: last header seen)
segments-sent             WPS1 segment table frames written by the Master
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
