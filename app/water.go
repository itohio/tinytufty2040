package app

import (
	ui "github.com/itohio/tinygui"
	"image/color"
	"time"

	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Water: EasyRobot's particle liquid (fluid.go; the engine Water shares with
// Fire's fuel), drawn as an LED matrix on widget.CellGrid: the engine's
// raster, 64×48 (its box's aspect), shaded in place: the surface is foam,
// deeper water darker.
const (
	waterW, waterH = 64, 48
	waterPitch     = 3
	waterShades    = 7
	// engine steps per frame: gravity is 0.02 cell per step² (the engine's
	// tuning), so a couple per frame make it flow at a believable speed
	waterStepsPerFrame = 2
)

var waterPalette = []color.RGBA{
	{12, 16, 24, 255}, // empty: unlit LED
	{20, 60, 140, 255}, {25, 80, 165, 255}, {30, 105, 190, 255}, {35, 130, 210, 255},
	{40, 160, 225, 255}, {70, 195, 240, 255},
	{215, 245, 255, 255}, // foam (surface)
}

type water struct {
	share  *liquidShare // the engine, shared with Fire's fuel (fluid.go)
	fl     fluid
	dir    int        // gravity's nearest lattice direction (buttons, the page's choice, shading)
	pinned bool       // dir was picked by hand: the IMU's tilt waits until the page is shown again
	pour   bool       // a stream lifted from the surface pours in opposite to gravity
	g      [2]float32 // the gravity Water last set (screen axes, m/s²), passed on after a claim
	shade  [waterW * waterH]uint8
	cells  int
	stepUS int64
	grid   *widget.CellGrid
	// full-screen LED views of the same grid: 64×48 at 5 px (320×240) or at
	// 4 px (256×192)
	full    [2]*widget.CellGrid
	tiles   [2]*widget.CellGrid // the menu tile: at 1 and 2 px
	fullOn  bool
	fullIdx int
	txt     textBuf
	txt2    textBuf
	log2    func(p1, p2 string) // App.log2: two parts, no string built
}

func newWater(share *liquidShare) *water {
	w := &water{share: share, fl: share.fl, g: [2]float32{0, gravity}}
	w.grid = widget.NewCellGrid(waterW, waterH, waterPitch, 1, widget.CellRound, w.shade[:], waterPalette, colBG)
	w.full[0] = widget.NewCellGrid(waterW, waterH, 5, 1, widget.CellRound, w.shade[:], waterPalette, colBG)
	w.full[1] = widget.NewCellGrid(waterW, waterH, 4, 1, widget.CellRound, w.shade[:], waterPalette, colBG)
	w.tiles[0] = widget.NewCellGrid(waterW, waterH, 1, 0, widget.CellSquare, w.shade[:], waterPalette, colBG)
	w.tiles[1] = widget.NewCellGrid(waterW, waterH, 2, 0, widget.CellSquare, w.shade[:], waterPalette, colBG)
	w.claim()
	return w
}

// claim takes the shared engine for Water (its page or full screen is
// shown), laying the basin afresh if Fire had it or the tile ran fewer.
func (w *water) claim() { w.claimN(waterN) }

// preview takes the engine for Water's Demos tile: fewer particles, a
// quicker step (owner 2026-11-13).
func (w *water) preview() { w.claimN(waterPreviewN) }

func (w *water) claimN(n int) {
	if w.share.claim(w, n) {
		w.useGravity()
		w.render()
	}
}

func (w *water) active() bool { return w.share.owner == w }

// step advances the liquid and shades it for the grid (only while Water
// has the engine).
func (w *water) step() {
	if !w.active() {
		return
	}
	t0 := time.Now()
	if w.pour { // two units a step, lifted from the surface, enter opposite to gravity
		g := gravityVec[w.dir]
		w.fl.Move(2, 0.5-float32(g[0])*0.4, 0.5-float32(g[1])*0.4)
	}
	w.fl.Step()
	w.render()
	w.stepUS = time.Since(t0).Microseconds()
}

func (w *water) render() {
	w.fl.Raster(w.shade[:], waterW, waterH) // shadeCells turns it into shades in place
	w.shadeCells()
}

// fill resets to the basin's liquid lying level.
func (w *water) fill() {
	w.claim()
	w.fl.Reset()
	w.render()
}

// setTilt: gravity from the motion state (screen axes, m/s²).
func (w *water) setTilt(gx, gy float32) {
	w.dir, _, _ = latticeSplit(gx, gy)
	w.g = [2]float32{gx, gy}
	if w.active() {
		w.fl.SetGravity(gx, gy)
	}
}

// useGravity passes the last gravity on to the engine (after a claim).
func (w *water) useGravity() { w.fl.SetGravity(w.g[0], w.g[1]) }

// turn rotates gravity by steps of 45° (buttons); it holds like a pick.
func (w *water) turn(by int) { w.pick((w.dir + by) & 7) }

// pick sets gravity to a lattice direction chosen by hand (the page's
// Gravity choice, the buttons) and holds it against the IMU until the page
// is shown again (unpin): without the hold, the shared motion state put the
// badge's own gravity back at every sample.
func (w *water) pick(dir int) {
	w.dir, w.pinned = dir&7, true
	w.useDir()
}

// unpin follows the badge's gravity again (its page shown afresh).
func (w *water) unpin() { w.pinned = false }

// useDir sets the engine's gravity to the lattice direction dir (the
// buttons, the page's choice).
func (w *water) useDir() {
	gx, gy := gravityOf(w.dir)
	w.g = [2]float32{gx, gy}
	if w.active() {
		w.fl.SetGravity(gx, gy)
	}
}

func (w *water) count() int { return w.fl.Count() }

var waterFullNames = []string{"5 px", "4 px"}

// fullView returns the full-screen grid. Call from Update, not Draw.
func (w *water) fullView() ui.Widget {
	if !w.fullOn {
		return nil
	}
	return w.full[w.fullIdx]
}

func (w *water) tileViews() (ui.Widget, ui.Widget) { return w.tiles[0], w.tiles[1] }
func (w *water) showFull()                         { w.claim(); w.fullOn = true }

// waterDepthShade maps depth below the surface (along gravity) to a palette
// index: 0 is the surface (foam), deeper is darker. A table, not a division.
var waterDepthShade = [16]uint8{7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 1}

// shadeCells turns the engine's raster in shade (liquid per cell, 0 for
// none) into the palette index of every cell, in place (no second 4 KB
// buffer: the badge ran out of memory at boot with one). It scans from the
// side gravity comes from, so the cell against gravity already holds its
// depth (stored as depth+1 during the scan) and a cell not yet scanned
// still holds the raster: one read per cell.
func (w *water) shadeCells() {
	g := gravityVec[w.dir]
	gx, gy := g[0], g[1]
	y0, y1, dy := 0, waterH, 1
	if gy < 0 {
		y0, y1, dy = waterH-1, -1, -1
	}
	x0, x1, dx := 0, waterW, 1
	if gx < 0 {
		x0, x1, dx = waterW-1, -1, -1
	}
	n := 0
	for y := y0; y != y1; y += dy {
		uy := y - gy
		for x := x0; x != x1; x += dx {
			i := y*waterW + x
			if w.shade[i] == 0 {
				w.shade[i] = 0
				continue
			}
			n++
			d := uint8(1)
			if ux := x - gx; ux >= 0 && uy >= 0 && ux < waterW && uy < waterH {
				if u := w.shade[uy*waterW+ux]; u > 0 {
					d = min(u+1, 16)
				}
			}
			w.shade[i] = d
		}
	}
	for i, d := range w.shade[:] { // a slice: ranging the array copies it to the heap (F-35)
		if d > 0 {
			w.shade[i] = waterDepthShade[d-1]
		}
	}
	w.cells = n
}

func (a *App) pageWater() *page {
	w := a.water
	grav := widget.NewInteractiveLabelChoice(52, 11, gravityNames,
		widget.WithLabelChoiceIndex(&w.dir), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG),
		widget.WithLabelChoiceChange(func(i int, s string) { w.pick(i); a.log2("gravity ", s) }))
	pour := widget.NewToggle(44, 12, fontBody, colFG, "ON", "OFF", colGood, colTrack,
		func() bool { return w.pour }, func(v bool) { w.pour = v })
	reset := widget.NewToggle(44, 12, fontBody, colFG, "FILL", "FILL", colTrack, colTrack,
		func() bool { return false }, func(bool) { w.fill(); a.log("water refilled") })
	full := widget.NewToggle(44, 12, fontBody, colFG, "SHOW", "SHOW", colTrack, colTrack,
		func() bool { return false }, func(bool) { w.showFull(); a.log("water full screen") })
	size := widget.NewInteractiveLabelChoice(52, 11, waterFullNames,
		widget.WithLabelChoiceIndex(&w.fullIdx), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	// explicit height: findings F-33
	p := a.newPageWith("Water", layout.VFlow(4, waterH*waterPitch),
		still(w.grid),
		newField("Gravity", 42, 104, grav),
		newField("Pour", 42, 104, pour),
		newField("Reset", 42, 104, reset),
		newField("Full", 42, 104, full),
		newField("LEDs", 42, 104, size),
		still(a.text(104, func() string { return w.txt.reset().str("lit ").int(w.cells).str(" of ").int(waterW * waterH).String() }, colMuted)),
		still(a.text(104, func() string { return w.txt2.reset().str("step ").int(int(w.stepUS)).str(" us").String() }, colMuted)),
	)
	p.demo = w
	return p
}

func (w *water) tick() {
	for range waterStepsPerFrame {
		w.step()
	}
}

// fullPress: UP/DOWN rotate gravity, A pours, B back to the page, C leaves
// and changes page (returns false so the App carries on).
func (w *water) fullPress(b Button) bool {
	switch b {
	case BtnUp, BtnDown:
		if b == BtnUp {
			w.turn(1)
		} else {
			w.turn(-1)
		}
		w.log2("gravity ", gravityNames[w.dir])
	case BtnA:
		w.pour = !w.pour
	case BtnB:
		w.fullOn = false
	case BtnC:
		w.fullOn = false
		return false
	}
	return true
}

func (w *water) reset() { w.fill() }
