
# Tufty 2040 showcase: command set

One command channel, two sources (owner, 2026-11-12: "inject commands to the command channel from either source - uart or actual button"). Physical buttons and USB-serial lines end in the same calls, `App.Press(button, long)` and `App.UserHold(duration)` (`app/app.go`), so a serial `a` and a press of A are the same event; both log `press a` to the Log page and the console. Part of [overview](overview.md).

## Button convention

Same meaning on every page (owner 2026-11-12: "up and down … A, B, and C … so you decide what these do"); the footer always shows the hints for the current mode.

| Button | Browsing | Editing a value | Footer hint |
|---|---|---|---|
| **UP / DOWN** | move the focus to the previous / next row (held: repeats) | increase / decrease the value (held: large steps) | `up/down focus` · `up/down change` |
| **A** | select: start editing the focused row, flip a toggle, enter a list | confirm (commit); on a multi-gauge: next segment | `A select` · `A ok` |
| **B** | back: leave a list | cancel: revert the edit | `B back` · `B cancel` |
| **C** | next page (held ≥ 500 ms: previous page) | cancel the edit, then next page | `C page` |
| **USER/BOOT** | hold > 1 s: screensaver on/off | — | — |

## How to send

- **USB serial** (USB CDC, any baud rate): one command per line, `\n`-terminated. The badge reports USB serial number `df6750955f2a3536` (its flash unique ID, the same number MicroPython reported); find it by that, not by port name.
- **Host tool**: `tools/badgectl`: `badgectl a up up a` sends commands and prints the replies; `--grab` adds a badge-cam photo after each; `--tour` reboots the badge and replays the simulator tour, saving framebuffer dumps, badge-cam photos and simulator PNGs side by side; `--monitor` prints the console (for testing the buttons by hand).
- Serial handlers only queue the command (16 deep, extra lines dropped); `App.Update` executes it at the start of the next frame on the UI goroutine ([tinygui concurrency](https://github.com/itohio/tinygui/blob/main/docs/architecture/concurrency.md)).
- **One serial writer** (`console`, review R-TG-1 T4/T5): every byte the badge sends — log lines, replies, dumps — goes through a 2 KB ring that one goroutine drains into USB (2 KB, not 4: the heap is the RAM `.bss` leaves). A host that keeps the port open without reading stalls only that goroutine, never the frame loop: a line that doesn't fit is dropped and counted (`console: lines dropped (queue full) <n>` once there is room). A dump (`FB …`) holds the ring from its header to `END`, so no other line lands inside it; lines written meanwhile are dropped and counted.

## Commands

| Command | Does | Same as button | Notes |
|---|---|---|---|
| `up` / `up long` | focus previous row; while editing: increase (`long` = large step) | UP (press; held ≥ 500 ms auto-repeats as `long` every 120 ms) | |
| `down` / `down long` | focus next row; while editing: decrease | DOWN (as UP) | |
| `a` / `a long` | select: start editing the focused row, flip a toggle, enter the list; while editing: commit (multi-gauge: next segment) | A (fires on release; held ≥ 500 ms = `long`) | |
| `b` / `b long` | cancel an edit (revert), or leave the list | B (on release) | |
| `c` / `c long` | next page (`long`: previous page); cancels an edit first | C (on release) | |
| `user <ms>` | report a USER hold of `<ms>`; > 1000 ms toggles the screensaver | USER/BOOT held (measured by `ui.PeekButton`, the loop pauses while it is down) | |
| `page <n\|name>` | jump to page `n`, or to the page of that name in any case (`page pong`). With an IMU: 0 Calib, 1 Demos, 2 Dash, 3 Gauge, 4 Ctrl, 5 Image, 6 Flow, 7 List, 8 Log, 9 Life, 10 Water, 11 Fire, 12 Pong, 13 Boids, 14 Motion; without one the indices shift down by one and Motion is absent. Anything else leaves the page as it is and logs `page: a number 0-N or a page name` | — | |
| `led <0-100>` | user LED brightness (0 = off) | — (Ctrl page: User LED, LED level) | |
| `bl <10-100>` | backlight brightness; **100 is DC** (pin driven high, no PWM: no stripes in camera photos), below 100 a 200 kHz PWM; the default is 100 on USB power, 80 on battery | — (Ctrl page: Backlight) | |
| `saver` / `saver 1` / `saver 0` | toggle / switch on / switch off the screensaver (`RandomContext` burn-in shift) | USER hold > 1 s toggles | any button press leaves it |
| `stats` | log the build ID; `fps`, `draw_us`, `push_us`, page, edit state; the `mem` lines; frame allocations (update/draw/push); then the frame profile since the last `stats`, four lines: `prof period_us <µs> imu_us <µs> update_us <µs>` and `prof draw_us <µs> push_us <µs> other_us <µs> n <frames>` (averages per frame), `prof imu samples <n> bus_us <µs> maths_us <µs>` (per IMU sample: the I2C reads, then the motion maths), and `prof imu err <n> ovf <n> since boot` (the drain's I2C errors and FIFO overflow restarts: both 0 when healthy). period is start to start; imu the IMU drain and shared motion state; update the demos' steps; draw and push all bands; other the rest of the loop between frames (buttons, the yield, the console). Read it after a few seconds on one page | — | |
| `mem` | collect garbage, then `mem live_kb <n> free_kb <n> heap_kb <n> gc <n>`: free after the collection is the headroom (between collections `heap_alloc` also counts garbage); then `mallocs <n> frees <n>` since boot: growth between two `mem` lines while idle means something outside the frame allocates | — | |
| `frames` | the frame-time histogram since the last call: `frames <n> max_ms <ms> gc <n>`, then `ms` lines of buckets (0-40, 40-60, 60-80, 80-120, 120-200, 200-500, 500+ ms: frames in each) | — | |
| `memshow` / `memshow 1` / `memshow 0` | toggle / show / hide the heap use (`used/sys K`, refreshed once a second, counts garbage) in the footer | — | |
| `version` | log `build <ID>`: the firmware's build ID, `<tinytufty2040 commit>[+]/<tinygui commit>[+]@<MMDD-HHMM>` (`+`: uncommitted changes), set by `tools/build` | — | |
| `solid [0-255]` | the whole screen one grey level (255 white) for camera tests; no argument returns to the UI | — | |
| `motion` | the shared motion state as the demos read it, two lines: `motion P <x> <y> <z> um disp <x> <y> <z> um` (the shown world position, the hybrid of `docs/overview.md`, and the badge-frame displacement, in µm; the Motion page shows whole cm) and `rest <n>% of <samples> settled <0/1> pad <px> moves <n>` (since the last `motion`: how much of the time the strapdown called rest; Pong's paddle row in px; how many times the strapdown left rest) | — | measuring the paddle at rest |
| `tiles` | frame budget of each Demos tile: `tile <name> tick_us <µs> draw_us <µs>` (draw summed over the three bands) | — | |
| `light` / `light <raw>` / `light -1` | log the light sensor (`light raw <last reading> median <of the last 5> pct <level> backlight <n>`; the level is on a log scale, 64 → 0 %, 16384 → 100 %, filtered: median of 5, then an average, the readout moving by whole steps — `app/light.go`); with a number, force that reading (0: the darkest room); `-1`: the real sensor again | — | |
| `tilt` | the in-plane gravity the demos use (after the IMU axis map): `gravity <d.d> deg toward screen left\|right, in-plane <m/s²>` — the angle from screen-down, the badge's lower side, and the in-plane size (small = lying nearly flat, the angle means little); `gravity: no direction` when flat or without a motion sensor | — | |
| `dump` | send the framebuffer: line `FB 320 240`, 153 600 bytes big-endian RGB565 (all three bands), line `END` | — | what was drawn, not what the glass shows |
| `cal [clear \| set ox oy oz sx sy sz gx gy gz]` | the calibration as numbers, a line each: `cal fit <offset\|offset+scale> in_use <0\|1> rms <m/s²> cond <spread>`, `cal offset <x> <y> <z> m/s2`, `cal scale <x> <y> <z>`, `cal gyro_bias <x> <y> <z> rad/s` (the fit shown on the Calibration page), then for every orientation measured `cal pose <orientation> acc <x> <y> <z>` (m/s², the raw mean), `… gyro <x> <y> <z>` (rad/s) and `… noise <accel> <gyro>`. Each new fit logs the same without asking. **Saved across power-off** (the flash's last 4 KB sector, a 64-byte record with the ranges it was made at and a CRC): a full fit (six orientations) is saved when A puts it in use; at boot a whole record made at the present ranges is put in use (`cal: the saved calibration is in use`), otherwise the log says why. `cal` opens with what is saved: `cal saved: none`, `cal saved: in use` (the fit in use is the saved one), or `cal saved: not used: <why>` (`made at another range`, `damaged (CRC)`, `another record version`, or `a newer fit is in use`: one not saved, such as an offsets-only fit). `cal clear` erases it; `cal set` puts nine numbers in use and saves them: offsets m/s², scales, gyro bias rad/s (offsets within 2 m/s², scales 0.8–1.25, gyro within 0.1 rad/s) | — | |
| `help` | list the commands | — | |
| `madctl <n>` | write the panel's MADCTL register (orientation probe; default 176 = `0xB0`) | — | firmware only (`main_tufty2040.go`) |
| `pins` | print raw button levels and VBUS | — | firmware only |
| `imu` | the motion sensor in use (MPU6050 at 0x68 drives gravity and the Motion page), or a probe for a known one (WHO_AM_I) and what answers | — | firmware only |
| `reboot` | restart the firmware (`arm.SystemReset`); scripted tours start from it | — | firmware only; the badge re-enumerates with the same serial number |

Every press also counts in the Dash page's bar chart, so serial and physical presses are both visible on screen.
