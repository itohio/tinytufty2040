# tinytufty2040

What TinyGo can do on a Pimoroni Tufty 2040 badge (RP2040, 133 MHz rated / 200 MHz under TinyGo, 264 KB RAM, 320×240 parallel ST7789): a home page of live demos running in tiles, each opening full screen on a 64×48 "LED matrix" of round dots, plus a showcase of [tinygui](https://github.com/itohio/tinygui) widgets, layouts and overlays.

| Demo | What it does |
|---|---|
| Life | Game of Life with ageing cells; glider gun, Acorn, R-pentomino |
| Water | EasyRobot's particle liquid that levels and follows gravity, pours, fills |
| Fire | a burning liquid: fuel pools and sloshes with gravity and seeds flames that rise against it; flames on a 160×120 field, drawn interpolated full screen |
| Pong | CPU vs CPU in its tile, you vs CPU full screen; paddles and ball at pixel positions |
| Boids | a flock in a world frame; the screen is a box the badge's roll (or the buttons) turns and shakes through it |
| Motion | with an IMU: the badge in 3D from a Mahony filter, a trail of where its top points, roll/pitch/yaw, gyro bias |

An MPU6050 on the debug pads (I2C0, GPIO4/5, INT GPIO3) feeds one shared motion state (EasyRobot's Mahony filter, `x/math/filter/ahrs`, and strapdown, `x/math/nav/strapdown`, with the gyro bias learnt at rest): Water and Fire follow its tilt, the boids' box turns and moves with the badge, Pong's paddle follows its pitch, the Motion page shows its path; without one, the buttons do (see [docs/overview.md](docs/overview.md#motion-sensor)).

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

Needs Go 1.25+, TinyGo 0.43, a tinygui checkout next to this repo (`go.mod` replaces `github.com/itohio/tinygui` with `../tinygui`), and an EasyRobot checkout next to it too (`go.mod` replaces `github.com/itohio/EasyRobot` with `../EasyRobot`: the MPU6050 driver, Mahony filter and strapdown).

```sh
go test ./...
go run ./sim -out /tmp/tufty
tinygo build -target=tufty2040 -o tufty.uf2 .
python3 tools/reboot2boot.py && cp tufty.uf2 /Volumes/RPI-RP2/
tools/badgectl stats
```

Buttons: UP/DOWN focus (or change a value while editing), A select, B back (on a page's top level: the page shown before), C next page (hold: previous), USER held > 1 s screensaver.
