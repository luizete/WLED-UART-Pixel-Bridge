# Installation checklist

1. Start from a WLED 16.0.1 checkout.
2. Copy `usermods/uart_pixel_bridge` into the WLED repository's `usermods` directory.
3. For AudioReactive builds, use the included `platformio_override.ini.example`: it builds `esp32s3_4M_qspi` (ESP32-S3 Super Mini) by default and also has an `esp32dev` env, both with `audioreactive` and `uart_pixel_bridge`.
4. Build and flash the same binary to both ESP32 boards.
5. Configure one board as Master and the other as Receiver under **Config → Usermods**.
6. Usermod settings are applied with a live bridge restart when saved; a full ESP32 reboot is normally unnecessary.
7. Wire two MAX3485 modules as described in the README.
8. Start at 921600 baud for initial bench testing if your modules or wiring are questionable, then move to 2000000 baud.
9. Confirm the Receiver Info page reports `Receiver active` after Master frames arrive. With startup guard enabled it should report `Receiver waiting` and keep the controlled LEDs black until the configured valid-frame threshold is reached.
10. Configure startup/failsafe policy under **Config → Usermods** (`startup-guard`, `startup-valid-frames`, `failsafe`, hold/action/fade settings, and reacquisition guard).
11. Reset counters before a long bench run by posting `{ "UART Pixel Bridge": { "reset-stats": true } }` to the WLED JSON state API.
12. Optional: enable the command input (`cmd-enabled`, `cmd-uart`, `cmd-pin`) on the Master and send `PING` from the controller; it should answer `PONG`.
