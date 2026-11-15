package app

import (
	ui "github.com/itohio/tinygui"
	"image/color"
	"time"

	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Fire burns a liquid fuel (owner: "a little bit of liquid that piles up in
// the corner or spreads over the edge ... the liquid never consumed though,
// but it sloshes and seeds the fire"). The fuel is the water's liquid
// engine: it pools, spreads and follows gravity, and is never used up.
// The fuel is the low-resolution seed (64×48 cells); the flames run on a
// 160×120 heat field drawn in real pixels by fireView (owner: only the
// water is pixelated): 1:1 on the page, interpolated full screen. Air next to the fuel, against gravity, takes full heat; heat
// rises against gravity from there, cooling and flickering (the "Doom
// fire" spread, turned with gravity). The heat field is also the picture:
// the flame palette, and the fuel marked in a dim amber.
const (
	fireW, fireH = 64, 48   // fuel cells
	heatW, heatH = 160, 120 // flame field (2.5 per fuel cell)
	fireMax      = 36       // hottest flame (palette index)
	fireFuel     = fireMax + 1
	// the fuel is the shared engine's liquid (fluid.go: one instance for
	// Water and Fire): its amount is the engine's. Nothing
	// adds or removes fuel: a splash takes it from the pool's surface.
	fireSplash = liquidMove // fuel units a splash lifts out of the pool and drops back in
)

var (
	fireHeights = []string{"low", "mid", "tall"}
	// The classic Doom fire couples the sideways shift and the cooling to the
	// same two random bits k (shift k-1: -1..+2, cool only when k is odd);
	// that coupling makes the tongues (independent bits only diffused the
	// heat into horizontal bands). An odd k cools by fireCool[height][two
	// more random bits]: mean cooling per row 0.75 / 0.375 / 0.25, so flames
	// reach about 48 / 96 / 144 of the 120 rows (low / mid / tall).
	fireCool = [3][4]uint8{{1, 1, 2, 2}, {1, 1, 1, 0}, {1, 1, 0, 0}}
	// heat-field column / row where fuel cell i starts (i·2.5)
	fireHX = func() (t [fireW + 1]int) {
		for i := range t {
			t[i] = i * heatW / fireW
		}
		return
	}()
	fireHY = func() (t [fireH + 1]int) {
		for i := range t {
			t[i] = i * heatH / fireH
		}
		return
	}()
)

// firePalette is the classic Doom fire palette (index 0 the dark) plus
// the fuel's dim amber.
var firePalette = func() []color.RGBA {
	hex := [fireMax + 2]uint32{
		0x120e0e, 0x1f0707, 0x2f0f07, 0x470f07, 0x571707, 0x671f07, 0x771f07, 0x8f2707, 0x9f2f07,
		0xaf3f07, 0xbf4707, 0xc74707, 0xdf4f07, 0xdf5707, 0xdf5707, 0xd75f07, 0xd75f07, 0xd7670f,
		0xcf6f0f, 0xcf770f, 0xcf7f0f, 0xcf8717, 0xc78717, 0xc78f17, 0xc7971f, 0xbf9f1f, 0xbf9f1f,
		0xbfa727, 0xbfa727, 0xbfaf2f, 0xb7af2f, 0xb7b72f, 0xb7b737, 0xcfcf6f, 0xdfdf9f, 0xefefc7,
		0xffffff,
		0x7a4c14, // fuel: dim amber, visible next to the flames
	}
	p := make([]color.RGBA, len(hex))
	for i, h := range hex {
		p[i] = color.RGBA{uint8(h >> 16), uint8(h >> 8), uint8(h), 255}
	}
	return p
}()

type fire struct {
	pinned    bool                 // dir was picked by hand: the IMU's tilt waits until the page is shown again
	laidDir   int                  // the direction the tile's own fuel layer was laid for (layFuel), -1: none
	share     *liquidShare         // the engine, shared with Water (fluid.go)
	fl        fluid                // the fuel
	g         [2]float32           // the gravity Fire last set (screen axes, m/s²), passed on after a claim
	occ       [fireW * fireH]uint8 // the engine's raster: fuel per cell
	dir, alt  int                  // gravity split between two lattice directions (latticeSplit) for the flames
	pAlt      uint32               // the odds (of 256) that a flame row follows alt
	heat      [heatW * heatH]uint8 // flame heat 0..fireMax, fuel marked fireFuel
	height    int
	stepUS    int64
	odd       bool      // flips every step: spread's scan direction along rows
	grid      *fireView // the page: the heat field at 1:1 (160×120)
	full      *fireView // full screen: 320×240, interpolated
	tiles     [2]*fireView
	fullOn    bool
	txt, txt2 textBuf
	log2      func(p1, p2 string) // App.log2: two parts, no string built
}

func newFire(share *liquidShare) *fire {
	f := &fire{share: share, fl: share.fl, height: 1, g: [2]float32{0, gravity}, laidDir: -1}
	f.grid = newFireView(f, heatW, heatH)
	f.full = newFireView(f, ScreenW, ScreenH)
	f.tiles = [2]*fireView{newFireView(f, tileW, tileH), newFireView(f, 2*tileW, 2*tileH)}
	f.reset()
	return f
}

// reset lays the fuel as a level layer along the bottom (gravity down).
func (f *fire) reset() {
	f.claim()
	clear(f.heat[:])
	f.dir, f.alt, f.pAlt = 0, 0, 0
	f.g = [2]float32{0, gravity}
	f.fl.SetGravity(f.g[0], f.g[1])
	f.fl.Reset()
	f.fl.Raster(f.occ[:], fireW, fireH)
}

// claim takes the shared engine for Fire (its page or full screen is shown),
// laying the fuel afresh if Water had it.
func (f *fire) claim() {
	if f.share.claim(f, fireN) {
		f.fl.SetGravity(f.g[0], f.g[1])
		f.fl.Raster(f.occ[:], fireW, fireH)
	}
}

func (f *fire) active() bool { return f.share.owner == f }

// splash lifts fireSplash units of fuel off the pool's surface and drops
// them back in as a blob from the side opposite gravity: the fuel sloshes,
// its amount stays the same (the engine's Move).
func (f *fire) splash() {
	f.claim()
	g := gravityVec[f.dir]
	f.fl.Move(fireSplash, 0.5-float32(g[0])*0.4, 0.5-float32(g[1])*0.4)
	f.fl.Raster(f.occ[:], fireW, fireH)
}

// setTilt: gravity from the motion sensor (screen axes): the fuel's engine
// gets it, the flames lean between the two lattice directions next to it.
func (f *fire) setTilt(gx, gy float32) {
	f.dir, f.alt, f.pAlt = latticeSplit(gx, gy)
	f.g = [2]float32{gx, gy}
	if f.active() {
		f.fl.SetGravity(gx, gy)
	}
}

// turn rotates gravity by steps of 45° (buttons); it holds like a pick.
func (f *fire) turn(by int) { f.pick((f.dir + by) & 7) }

// pick sets gravity to a lattice direction chosen by hand and holds it
// against the IMU until the page is shown again (see water.pick).
func (f *fire) pick(dir int) {
	f.dir, f.pinned = dir&7, true
	f.useDir()
}

// unpin follows the badge's gravity again (its page shown afresh).
func (f *fire) unpin() { f.pinned = false }

// useDir sets gravity to the lattice direction dir (the buttons, the page's
// choice): the flames go straight against it.
func (f *fire) useDir() {
	f.alt, f.pAlt = f.dir, 0
	gx, gy := gravityOf(f.dir)
	f.g = [2]float32{gx, gy}
	if f.active() {
		f.fl.SetGravity(gx, gy)
	}
}

func (f *fire) count() int { return f.fl.Count() }

// step burns one frame. Only the fuel needs the shared engine: while Water
// holds it, the fuel stays as it last lay and the flames keep rising from
// it (returning early left the Demos tile black: Fire is built first, then
// Water takes the engine).
func (f *fire) step() {
	t0 := time.Now()
	if f.active() {
		f.fl.Step()
		f.fl.Raster(f.occ[:], fireW, fireH)
		f.laidDir = -1
	} else if f.laidDir != f.dir {
		f.layFuel() // Water holds the engine: the fuel can't flow, so it's laid where it would lie
	}
	f.spread()
	f.markFuel()
	f.stepUS = time.Since(t0).Microseconds()
}

// spread moves heat one cell against gravity, in place, the way the
// classic Doom fire does: every cell writes its heat, cooled, to the cell
// beyond it against gravity, shifted sideways by fireJitter (symmetric;
// Doom's own -2..+1 drifted left). Scanning from the side opposite gravity,
// each cell is read before anything writes over it. Along a row the scan
// direction alternates (by row and by step): when two cells write the same
// cell the later wins, so one fixed direction carried heat to that side; cells nobody writes
// keep last step's heat, which draws the streaks along the flames
// (gathering from a neighbour instead gave runs of equal values across
// them: neighbours soon share one source). Fuel (marked fireFuel) writes
// full, flickering heat: the fuel's surface is the source.
func (f *fire) spread() {
	d := f.dir // from the IMU: the other direction next to G in proportion
	if rand32()&255 < f.pAlt {
		d = f.alt
	}
	g := gravityVec[d]
	gx, gy := g[0], g[1]
	cool := &fireCool[f.height]
	h := &f.heat
	f.odd = !f.odd
	if gy != 0 {
		y0, y1, dy := 0, heatH, 1 // gravity down: flames go up, read the top rows first
		if gy < 0 {
			y0, y1, dy = heatH-1, -1, -1
		}
		for y := y0; y != y1; y += dy {
			ty := y - gy
			if ty < 0 || ty >= heatH {
				continue
			}
			x, xEnd, dx := 0, heatW, 1
			if f.odd != (y&1 == 1) {
				x, xEnd, dx = heatW-1, -1, -1
			}
			for ; x != xEnd; x += dx {
				r := rand32()
				tx := x - gx + fireJitter(r)
				if tx < 0 || tx >= heatW {
					continue
				}
				h[ty*heatW+tx] = fireCool1(h[y*heatW+x], uint8(r>>4&1)*cool[r>>2&3], r)
			}
		}
		return
	}
	x0, x1, dx := 0, heatW, 1 // gravity right: flames go left, read the left columns first
	if gx < 0 {
		x0, x1, dx = heatW-1, -1, -1
	}
	for x := x0; x != x1; x += dx {
		tx := x - gx
		if tx < 0 || tx >= heatW {
			continue
		}
		y, yEnd, dy := 0, heatH, 1
		if f.odd != (x&1 == 1) {
			y, yEnd, dy = heatH-1, -1, -1
		}
		for ; y != yEnd; y += dy {
			r := rand32()
			ty := y + fireJitter(r)
			if ty < 0 || ty >= heatH {
				continue
			}
			h[ty*heatW+tx] = fireCool1(h[y*heatW+x], uint8(r>>4&1)*cool[r>>2&3], r)
		}
	}
}

// fireJitter is a cell's sideways shift as its heat rises one row, from the
// low two random bits: -1, 0, 0 or +1. Symmetric, so the fire rises
// straight: the classic Doom fire's 1-k, k in 0..3, is +1..-2 — half a cell
// per row to one side — and was why this fire drifted left (owner
// 2026-11-13). Bits 4 (cool or not) and 2-3 (how much) are separate, so
// cooling doesn't lean either.
func fireJitter(r uint32) int { return int((r&3+1)>>1) - 1 }

// fireCool1 is the heat a cell passes on from v: fuel is a full,
// flickering source; flames cool by c.
func fireCool1(v, c uint8, r uint32) uint8 {
	if v == fireFuel {
		if r>>5&3 == 0 {
			return fireMax - 10
		}
		return fireMax
	}
	if v > c {
		return v - c
	}
	return 0
}

// markFuel paints the fuel's cells into the heat field (2-3 heat cells per
// fuel cell).
func (f *fire) markFuel() {
	for fy := range fireH {
		for fx := range fireW {
			if f.occ[fy*fireW+fx] == 0 {
				continue
			}
			for y := fireHY[fy]; y < fireHY[fy+1]; y++ {
				for x := fireHX[fx]; x < fireHX[fx+1]; x++ {
					f.heat[y*heatW+x] = fireFuel
				}
			}
		}
	}
}

func (f *fire) tick() { f.step() }

func (f *fire) fullView() ui.Widget {
	if f.fullOn {
		return f.full
	}
	return nil
}

// fullPress: UP/DOWN turn gravity (the fuel flows, the flames lean), A
// splashes fuel in, B back to the page, C leaves and changes page.
func (f *fire) fullPress(b Button) bool {
	switch b {
	case BtnUp:
		f.turn(1)
		f.log2("gravity ", gravityNames[f.dir])
	case BtnDown:
		f.turn(-1)
		f.log2("gravity ", gravityNames[f.dir])
	case BtnA:
		f.splash()
	case BtnB:
		f.fullOn = false
	case BtnC:
		f.fullOn = false
		return false
	}
	return true
}

func (f *fire) tileViews() (ui.Widget, ui.Widget) { return f.tiles[0], f.tiles[1] }
func (f *fire) showFull()                         { f.claim(); f.fullOn = true }

func (a *App) pageFire() *page {
	f := a.fire
	grav := widget.NewInteractiveLabelChoice(52, 11, gravityNames,
		widget.WithLabelChoiceIndex(&f.dir), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG),
		widget.WithLabelChoiceChange(func(i int, s string) { f.pick(i); a.log2("gravity ", s) }))
	splash := widget.NewToggle(44, 12, fontBody, colFG, "DROP", "DROP", colTrack, colTrack,
		func() bool { return false }, func(bool) { f.splash(); a.log("fire splash") })
	height := widget.NewInteractiveLabelChoice(52, 11, fireHeights,
		widget.WithLabelChoiceIndex(&f.height), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	full := widget.NewToggle(44, 12, fontBody, colFG, "SHOW", "SHOW", colTrack, colTrack,
		func() bool { return false }, func(bool) { f.showFull(); a.log("fire full screen") })
	// explicit height: findings F-33
	p := a.newPageWith("Fire", layout.VFlow(4, heatH),
		still(f.grid),
		newField("Gravity", 42, 104, grav),
		newField("Splash", 42, 104, splash),
		newField("Flames", 42, 104, height),
		newField("Full", 42, 104, full),
		still(a.text(104, func() string {
			return f.txt.reset().str("fuel ").int(f.count()).String()
		}, colMuted)),
		still(a.text(104, func() string { return f.txt2.reset().str("step ").int(int(f.stepUS)).str(" us").String() }, colMuted)),
	)
	p.demo = f
	return p
}

// layFuel lays the fuel as a thin layer on the floor gravity points at
// (lattice direction dir), inside the engine box's walls, as thick as the
// engine's own layer: the Fire tile's fuel while Water holds the shared
// engine on the Demos page. Without it the fuel stayed where it last lay
// while the flames followed the IMU, and a badge held sideways showed flames
// rising sideways from the old floor (owner, TG-5 #1).
func (f *fire) layFuel() {
	clear(f.occ[:])
	g := gravityVec[f.dir&7]
	wx, wy := (fireW+liquidW-1)/liquidW, (fireH+liquidH-1)/liquidH // the wall ring, in raster cells
	band := fireFuelRows
	if g[0] != 0 && g[1] != 0 {
		band = fireFuelRows * 3 / 2 // diagonal: the same depth across the corner
	}
	hmax := -1 << 30
	for y := wy; y < fireH-wy; y++ {
		for x := wx; x < fireW-wx; x++ {
			hmax = max(hmax, x*g[0]+y*g[1])
		}
	}
	for y := wy; y < fireH-wy; y++ {
		for x := wx; x < fireW-wx; x++ {
			if x*g[0]+y*g[1] > hmax-band {
				f.occ[y*fireW+x] = 1
			}
		}
	}
	f.laidDir = f.dir
}
