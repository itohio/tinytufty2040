
# Overview — the Tufty 2040 demo

TinyGo + [tinygui](https://github.com/itohio/tinygui) on the Pimoroni Tufty 2040 badge (RP2040, 320×240 ST7789, 264 KB RAM): a home page of live demo tiles (Life, water, burning-liquid fire, Pong, boids), the tinygui widget showcase pages, and overlays drawn over running demos. Generic algorithms (liquid engine, Life, boids, Mahony) are moving to [EasyRobot](https://github.com/itohio/EasyRobot); tinygui keeps GUI building blocks only (scope: [tinygui ADR 0001](https://github.com/itohio/tinygui/blob/main/docs/decisions/0001-library-scope-and-layers.md)).

## Layout of the code

| Path | Build | Role |
|---|---|---|
| `app/` | any | the whole UI, TinyGUI only: pages, theme, example widgets, input mapping, serial commands; no `machine` import |
| `main_tufty2040.go` | `tufty2040` | firmware: frame loop, buttons, USER via `ui.PeekButton`, USB-serial console via `ui.CommandStreamMux` + `ui.SerialReader`, framebuffer dump |
| `board/` | `tufty2040` | `board.LCD`: ST7789 320×240 on the 8-bit parallel bus, bit-banged via RP2040 SIO; init and orientation from Pimoroni's driver; write timing sized for the 200 MHz CPU clock (83 ns per byte, ST7789 needs ≥ 66 ns) |
| `bringup/` | `tufty2040` | bring-up firmware without TinyGUI: LED heartbeat, backlight, panel init, full-screen colour cycle, serial log with heap state |
| `badge_tufty2040.go` | `tufty2040` | light sensor, battery (1.24 V reference), VBUS, user LED and backlight PWM, button debouncing/auto-repeat |
| `sim/` | host (`!tinygo`) | simulator: same `app`, fake hardware, scripted button presses, PNG screenshots |
| `bench/`, `i2cprobe/` | `tufty2040` | benchmark firmware (pushes, Life, upscaling, fire, LED rendering) and the I2C probe (bus pull-ups, scan, positive control) |
| `tools/` | host | `badgectl` (console, dumps, badge-cam photos, the simulator's tour), `reboot2boot.py`, `capture.py` |
| `github.com/itohio/tinygui/examples/framebuffer` | any | RGB565 RAM framebuffer (bands) implementing `ui.Displayer`; the firmware pushes it, the simulator encodes it |

The firmware draws each frame in three 80-row bands into a 51 200 B RAM buffer (`framebuffer.NewBand`) and pushes each band in one burst, so the panel never shows the clear-then-draw sequence ([tinygui findings](https://github.com/itohio/tinygui/blob/main/docs/analysis/findings.md) F-06 avoided in the example, not fixed in the library). A full 153 600 B frame buffer ran the heap out of memory on the badge; two 120-row bands still did once the heap fragmented ([tinygui findings](https://github.com/itohio/tinygui/blob/main/docs/analysis/findings.md) F-30, F-31). Commands from USB serial and from the buttons share one channel: [commands](commands.md).

## Board facts used

From TinyGo's `src/machine/board_tufty2040.go` (0.43.0), Pimoroni's `pimoroni-pico` sources and the TinyGo board page (all read 2026-11-10):

| Item | Pins / value | Source |
|---|---|---|
| Panel | ST7789, 320×240, 8-bit parallel: CS 10, DC 11, WR 12, RD 13, DB0–7 = GPIO14–21, backlight GPIO2 (PWM1 A) | board file; `picographics.cpp` `ParallelPins {10, 11, 12, 13, 14, 2}` |
| Orientation | mounted upside down → ROTATE_180: MADCTL `0xB0`, CASET 0–319, RASET 0–239 | `picographics.cpp` ("Tufty display is upside-down"), `st7789.cpp` `configure_display` |
| Buttons | A 7, B 8, C 9, UP 22, DOWN 6 active-high (pull-down); USER/BOOT 23 active-low | board file; `button_test.py` (`invert=False` / `invert=True`) |
| User LED | GPIO25 (PWM4 B) | board file, `led_pwm.py` |
| Light sensor | ADC0 / GPIO26, powered by SENSOR_POWER GPIO27 | board file, `autobright.py` |
| Battery | ADC3 / GPIO29 ×3 divider, 1.24 V reference on ADC2 / GPIO28, VBUS detect GPIO24 | `battery.py` |
| Flash | 8 MB on the board, but TinyGo's `tufty2040` target links with `--defsym=__flash_size=1020K` (`targets/tufty2040.json`), so firmware plus any assets kept in flash must fit in 1 020 KB unless the linker setting is changed | TinyGo 0.43 `targets/tufty2040.json`; Pimoroni (8 MB) |

Confirmed on the device: orientation (MADCTL `0xB0`; the badge cam itself mirrored until its firmware fix), button polarity, the 200 MHz bus timing. Open: whether red is weak on the glass or only through the camera (orange reads yellow-green on camera, the framebuffer is right).

## Pages and the features they exercise

| Page | TinyGUI features |
|---|---|
| Demos | the home page: every demo runs live in a tile drawn by its own view (64×48 px, the focused tile 128×96), placed by the `layout.Wrap` arranger (rows re-wrap when the focused tile grows); A opens the demo full screen, B there returns; a speed `Dropdown` (½, ×1, ×2) among the tiles; B on the page opens a choice `Dialog` (Pause/Resume all, Reset all… → modal confirm, About → popup), all drawn over the running tiles by `container.Overlay`; `tiles` console command for each tile's budget |
| Dash | `Label` with zero-allocation text providers, `VolumeGauge`, `SolidGauge`, custom sparkline (ring buffer) and bar chart widgets, `HList`/`VList` nesting, `Selectable` to skip static rows |
| Gauge | `Gauge` vertical and horizontal, `MultiGauge`, `VolumeGauge`, four `animation` curves (linear, ease-in, ease-out, ease-in-out) ping-ponging, `InteractiveGauge`, `InteractiveMultiGauge`, `InteractiveLabel[int]` with commit callback restarting the animations |
| Ctrl | `Toggle` (drives the user LED), `InteractiveLabel[int]`/`[float32]` with formatters (LED level, backlight, gain), `InteractiveLabelChoice` (accent colour recolours the UI), example `durationLabel` using `ui.DurationAdd/Sub`, `MultilineLabel` |
| Image | four PNG icons decoded at build time (`gen_pngs.go` → `pngs.go`, RGB565: no PNG decoder on the badge) as `Bitmap16` in an `InteractiveWidgetChoice`, `InteractiveWidgetChoice` over `Bitmap16` icons, animated `Bitmap16` plasma (48×30), `Bitmap8` (RGB332), `Separator` |
| Flow | `Grid`, `HFlow` and `VFlow` panels, laid out by `HList` |
| List | `ScrollChoice` with 8 rows (all visible: no scrolling since the heap trim), `ScrollObserver` scrollbar, navigator descent (A enters, B leaves) |
| Log | `InteractiveLog` fed by `NavigatorObserver`, button and serial events; navigator depth and the number of pages built |
| Life | Game of Life on a 64×48 `CellGrid` (round LEDs, ages fade; RGB565 scanline fast path), 3 px in the page; Run/Pause (`Toggle`), pattern (`InteractiveLabelChoice`: Random, glider gun, Acorn, R-pentomino); **Full** at 5 px, where UP/DOWN change the pattern, A runs/pauses, B returns; `ui.Clipper` band skipping |
| Water | falling-liquid automaton on a 64×64 grid, `CellGrid` at 3 px (192×192), foam/depth shading; gravity in 8 directions (`InteractiveLabelChoice`), Pour and Fill (`Toggle`); 3 steps per frame; **Full** shows the grid alone on the whole screen (LEDs 5 px: 64×48 at 320×240, or 4 px: 64×60), rows following gravity; there UP/DOWN rotate gravity, A pours, B returns, C changes page |
| Fire | a burning liquid (owner 2026-11-12): fuel on the shared liquid engine (`liquid.go`, as water) pools in a corner or spreads and follows **Gravity** (8 directions); it is never used up: it sloshes and every fuel cell open to the air seeds flames, which rise against gravity (Doom-fire heat spread turned with gravity); **Fuel** pours a stream, **Splash** drops more fuel (up to 400 cells), Flames low/mid/tall; `CellGrid` 3 px in the page, **Full** 5 px: UP/DOWN turn gravity, A splash, B back |
| Pong | fixed-point physics drawn at pixel positions (`pongView`): paddles, rounded ball with a trail, net, 3×5 score digits; CPU vs CPU in the page (level choice), **You** (PLAY) goes full screen with your paddle following the badge as you move it up and down (with the IMU; owner 2026-11-13), 1:1 on the glass: the paddle moves on screen as far and as fast as the badge (a cell is 0.6375 mm, the whole travel 2.55 cm; `pongCellsPerM`), from the shared motion state's displacement along the badge's own up axis (owner 2026-11-13: badge frame, not world); at an edge it stops and turns back with the badge at once; UP/DOWN still work, A pause, B back |
| Boids | Reynolds flocking of 32/64/128 boids in a **world frame**; the screen is a box moving through it (owner 2026-11-12): turning (the badge's roll from the shared motion state's attitude, or UP/DOWN in full screen, Spin on the page), moving (the badge's displacement in its own frame, 1:1 on the glass) or shaking (A) the box sweeps its walls through the flock, which also feels the badge's own acceleration, boids keep their world heading and only react when a wall reaches them; drawn at pixel positions (`boidsView`): heads coloured by on-screen heading, fading trails |
| Motion | only with an IMU: the **world path** (the shared motion state's world position) **seen from the badge** (owner 2026-11-13): a perspective camera at the latest position, set back along the view axis far enough to show the whole path (≥ 15 cm), turned with the badge's orientation, so turning the badge turns the path around where you are; a 10 cm bar at the latest position's distance; the world's axes in a small faint inset; x/y/z cm; roll/pitch/yaw relative to the badge (0/0/0 upright facing you; roll clockwise +, pitch top towards you +, yaw from the last path reset); at rest / moving; **Path HERE** (or A in Full) starts a new path; **Full** on the whole screen |
| (any) | tab bar with an eased underline (`animation`), scrolling to keep the active tab in view (6 visible); pages in `container.Pages`, every page built in setup (`BuildAll`: nothing is built later; the demo simulations, sensor history and log are models that outlive their pages), B on a page's top level returns to the page shown before (8 back); `container` idle timeout, `RandomContext` screensaver (hold USER > 1 s) |

Controls: UP/DOWN move focus (or change the value while editing; held = auto-repeat with the large step), A selects / commits, B cancels / goes back (on a page's top level: to the page shown before; on Demos: its options), C next page (held: previous page), USER hold > 1 s screensaver.

`tools/soak [rounds]` uses the badge like a person, buttons only: every Demos tile full screen and back, the speed dropdown, the options dialog, then C through every page with its usual interactions (edits, list scrolling, each demo's Full view), `mem` (live heap after GC) every round. `TestMenuRoundAllocatesNothing` plays the same round on the host, three times: 0 allocations under TinyGo (`tinygo test -gc=leaking ./app`).

Serial console and buttons share one command channel; the full command set, the matching buttons and the host tool are in [commands](commands.md). The badge reports USB serial number `df6750955f2a3536`.

## Build and run

The module builds against a tinygui checkout next to this repo (`replace github.com/itohio/tinygui => ../tinygui` in `go.mod`).

```sh
cd .
GOTOOLCHAIN=local GOPROXY=off go test ./...                           # app tests (demos, allocation-free frames)
GOTOOLCHAIN=local GOPROXY=off go run ./sim -out /tmp/tufty            # PNG shots of the scripted tour
GOTOOLCHAIN=local GOPROXY=off tinygo build -target=tufty2040 -size short -o tufty.uf2 .
python3 tools/reboot2boot.py && cp tufty.uf2 /Volumes/RPI-RP2/        # flash over USB (1200-baud touch)
tools/badgectl stats "page 9" dump                                    # drive it over USB serial
```

Flashing replaces the badge's MicroPython firmware. If the badge drops off USB, press its power button once (it is a hardware toggle); for the bootloader, hold BOOT/USER and tap RESET. Restoring MicroPython needs Pimoroni's Tufty UF2.

## Motion sensor

The Tufty 2040 has none on board: the I2C bus (GPIO4 SDA, GPIO5 SCL, GPIO3 INT) goes only to the Qw/ST connector and the debug pads (`i2cprobe` proved the bare bus empty, with a positive control). The owner's badge has an MPU6050 soldered to the debug pads (0x68), sides aligned with the badge and facing away from the screen: badge right = sensor −X, up = +Y, out of the screen = −Z (`imuSample` maps it). The firmware drains the sensor's FIFO every frame (100 Hz, `imuDrain`; 1024 bytes hold 0.85 s, longer than any frame) and passes every sample to `App.SetIMU`.

**One shared motion state** (owner 2026-11-13; `app/motion.go`), updated once per sample, read by every demo (no demo integrates anything itself):
- **attitude:** EasyRobot's Mahony filter (6-axis). The gyro bias is learnt at rest. The gain is high until the filter has turned from flat to the badge's pose, then steady. It runs on the gyro alone while the badge accelerates (> 0.15 m/s²) or turns (> 0.2 rad/s): the accelerometer correction takes a push for a tilt and lags a turn.
- **world position:** EasyRobot's strapdown (`x/math/nav/strapdown`) on that attitude, gravity removed, held at zero velocity at rest. It is fed only once the filter has settled: fed from power-on, it learnt a bias while the filter still turned, and a tilt then moved the path metres.
- **in the badge's own frame** (owner: "vertical position in the badge frame of reference … same for water and fire"): the displacement (each world step turned into the badge's axes as it is then, summed) and the specific force (the accelerometer: gravity plus the badge's own acceleration, low-passed over 30 ms).

Who reads what (`applyMotion`):
- The boids' window: turns with the attitude, moves with the displacement in the badge's own frame, 1:1 on the glass; the flock feels the badge's own acceleration.
- Pong's paddle: the displacement along the badge's up axis, 1:1.
- The Motion page: the world path, seen from a camera at the latest position, turned with the badge.

Host tests from synthetic 100 Hz traces (`app/motionstate_test.go`):
- a 45° tilt and back moves nothing;
- tilted 45°, a stroke along the badge's up axis moves the paddle 1:1 and one along world vertical by cos 45°;
- a 10 cm move shifts the boids' window 1:1;

## Status (2026-11-12)

Verified on the badge (USB serial `df6750955f2a3536`) through `badgectl`, framebuffer dumps and the badge cam: Demos page 13–15 fps, demo pages 15–24 fps, full-screen demos 18–25 fps; frames draw without heap allocation (test `TestPagesDrawFramesWithoutAllocating`). Measurements per demo: [demo and benchmark plan](demo-benchmark-plan.md).

## Size

`tinygo build -target=tufty2040 -size full`: TinyGUI packages 25 582 B code (`widget` 15 632, `ui` 4 574, `container` 3 676, `layout` 870, `animation` 830); app 21 887; PNG decoder 13 596 flash + 34 880 RAM; `fmt` + `strconv` ≈ 34.5 KB flash, linked by the library's default `fmt.Sprintf` formatter in `InteractiveLabel` even though the demo supplies its own formatters ([tinygui findings](https://github.com/itohio/tinygui/blob/main/docs/analysis/findings.md) F-26).

## Library defects found while building it

Recorded in [tinygui findings](https://github.com/itohio/tinygui/blob/main/docs/analysis/findings.md) as F-22…F-27: the `Scroll` layout bug (fixed with a regression test), `ScrollChoice` change callbacks that miss navigator moves, the idle timer that ignores navigator moves, no active-segment indicator in `InteractiveMultiGauge`, the forced `fmt` dependency, and the 34 KB inflate window the PNG decoder keeps in RAM. Missing clipping (F-13) forced the app to repaint the tab bar and footer after the body (`app/app.go`, `Draw`).
