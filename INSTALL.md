# Installation checklist

1. Start from a current WLED `main` checkout compatible with the included usermod API.
2. Copy `usermods/uart_pixel_bridge` into the WLED repository's `usermods` directory.
3. For classic ESP32 + AudioReactive, use the included `platformio_override.ini.example`, which builds `esp32dev_V4` with both `audioreactive` and `uart_pixel_bridge`.
4. Build and flash the same binary to both ESP32 boards.
5. Configure one board as Master and the other as Receiver under **Config → Usermods**.
6. Usermod settings are applied with a live bridge restart when saved; a full ESP32 reboot is normally unnecessary.
7. Wire two MAX3485 modules as described in the README.
8. Start at 921600 baud for initial bench testing if your modules or wiring are questionable, then move to 2000000 baud.
9. Confirm the Receiver Info page reports `Receiver active` after Master frames arrive. With startup guard enabled it should report `Receiver waiting` and keep the controlled LEDs black until the configured valid-frame threshold is reached.
10. Configure startup/failsafe policy under **Config → Usermods** (`startup-guard`, `startup-valid-frames`, `failsafe`, hold/action/fade settings, and reacquisition guard).
11. Reset counters before a long bench run by posting `{ "UART Pixel Bridge": { "reset-stats": true } }` to the WLED JSON state API.
