# Flippflop

A Flipper Zero app for controlling external RF hardware over UART: a CC1101
sub-GHz radio and an ADF4351 wideband PLL, each running on its own companion
ESP32 board.

## Repo layout

- `flipperzero_app/` — the Flipper Zero app (`flippflop.c`). Runs on the
  Flipper itself, sends UART commands to whichever ESP32 board is connected.
- `esp32/cc1101/` — ESP32 Arduino sketch that drives the CC1101 radio.
- `esp32/adf4351/` — ESP32 Arduino sketch that drives the ADF4351 PLL.

Only one ESP32 board (and therefore one sketch) is connected to the Flipper
at a time — the Flipper's "CC1101 Control" and "ADF4351 Control" menus are
two modes for that single UART connection, not simultaneous multi-device
control.

## Building and flashing the Flipper Zero app

The Flipper app is built with [`ufbt`](https://github.com/flipperdevices/flipperzero-ufbt),
the community micro build tool — no full firmware checkout needed.

```sh
pip install --upgrade ufbt
cd flipperzero_app
ufbt          # downloads the Flipper SDK on first run, builds dist/flippflop.fap
```

To get it onto a physical Flipper Zero, either:

- **`ufbt launch`** — builds, installs, and runs the app directly on a
  Flipper connected over USB, or
- Copy the built `dist/flippflop.fap` onto the Flipper's SD card under
  `apps/Tools/` (e.g. via qFlipper's file manager), then launch it from the
  Flipper's Apps menu.

## Flashing the ESP32 boards

The `esp32/adf4351/` and `esp32/cc1101/` sketches are separate firmware for
the external ESP32 boards, **not** for the Flipper Zero. Open the relevant
`.ino` file in the Arduino IDE (or use `arduino-cli`/PlatformIO) and upload
it to the ESP32 board over USB in the usual way. See each sketch's header
comment for the wiring (SPI pins to the CC1101/ADF4351 chip).
