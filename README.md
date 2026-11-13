# tinytufty2040

What TinyGo can do on a Pimoroni Tufty 2040 badge (RP2040, 133 MHz rated / 200 MHz under TinyGo, 264 KB RAM, 320×240 parallel ST7789): a home page of live demos running in tiles, each opening full screen on a 64×48 "LED matrix" of round dots, plus a showcase of [tinygui](https://github.com/itohio/tinygui) widgets, layouts and overlays.

| Demo | What it does |
|---|---|
| Life | Game of Life with ageing cells; glider gun, Acorn, R-pentomino |
| Water | a liquid that levels and follows gravity in 8 directions, pours, fills |
| Fire | a burning liquid: fuel pools and sloshes with gravity and seeds flames that rise against it |
| Pong | CPU vs CPU in its tile, you vs CPU full screen |
| Boids | a flock in a world frame; the screen is a box you turn and shake through it |

Gravity and the boids' box follow the buttons: the Tufty has no motion sensor (see [docs/overview.md](docs/overview.md#motion-sensor)). An IMU on the Qw/ST port, read through an [EasyRobot](https://github.com/itohio/EasyRobot) driver, is next.

## Layout

| Path | Role |
|---|---|
| `app/` | the whole UI and the demos; no board code; runs in the simulator and on the badge |
| `main_tufty2040.go`, `badge_tufty2040.go`, `board/` | firmware: banded framebuffer, panel driver, buttons, sensors, USB console |
| `sim/` | host simulator: scripted button presses, PNG shots |
| `bench/`, `bringup/`, `i2cprobe/` | benchmark, bring-up and I2C-probe firmware |
| `tools/` | `badgectl` and helpers to flash and drive the badge over USB serial |
| `docs/` | [overview](docs/overview.md), [console commands](docs/commands.md), [demo and benchmark plan](docs/demo-benchmark-plan.md) |

## Build

Needs Go 1.23+, TinyGo 0.43, and a tinygui checkout next to this repo (`go.mod` replaces `github.com/itohio/tinygui` with `../tinygui`).

```sh
go test ./...
go run ./sim -out /tmp/tufty
tinygo build -target=tufty2040 -o tufty.uf2 .
python3 tools/reboot2boot.py && cp tufty.uf2 /Volumes/RPI-RP2/
tools/badgectl stats
```

Buttons: UP/DOWN focus (or change a value while editing), A select, B back (on a page's top level: the page shown before), C next page (hold: previous), USER held > 1 s screensaver.
