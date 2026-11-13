package app

import (
	"math"

	"github.com/itohio/EasyRobot/x/math/fluid/flip"
)

// fluid is the liquid engine Water and Fire's fuel run on: EasyRobot's
// particle liquid (x/math/fluid/flip, ER-061; owner 2026-11-13: the
// cellular automaton didn't behave like a liquid). The demos never touch
// the engine's insides: they set gravity, step it, read a raster of where
// the liquid is, and move liquid for splashes and pours (an engine keeps
// its amount: nothing is created).
type fluid interface {
	Step()
	// SetGravity sets the apparent gravity in screen axes (x right, y down,
	// m/s²: the specific force the badge feels, reversed): its direction and
	// its size (a shake pushes harder).
	SetGravity(gx, gy float32)
	// Raster fills dst (w·h) with how much liquid each cell of a w×h raster
	// over the box holds (0: none).
	Raster(dst []uint8, w, h int)
	// Reset lays all the liquid level on the floor (+y).
	Reset()
	// Count is the amount of liquid (particles).
	Count() int
	// SetCount sets the amount of liquid, up to the engine's capacity (no
	// allocation); Reset lays it afresh.
	SetCount(n int)
	// Move lifts up to n units of liquid off the surface (the most against
	// gravity) and drops them at (fx, fy), fractions of the box; it returns
	// how many moved.
	Move(n int, fx, fy float32) int
}

// The shared engine's box and particle count (Leader 2026-11-13): the
// largest r4 size that leaves ≥ 20 KB free once the automatons are gone
// (24×18 cells, the wall ring included, 400 particles: 12.3 KB on TinyGo,
// fpv-gate). Rastered to the 64×48 LED grids.
const (
	liquidW, liquidH = 24, 18
	liquidN          = 400
	// liquidG is |g| in the engine's units (Q16 cells per step²: the
	// handoff's 0.02 cell/step²); a shake may push up to twice that
	liquidG    = 1311
	liquidGMax = 2 * liquidG
	// liquidHold: a gravity change smaller than this (1/128 of |g|, 0.45°)
	// is sensor noise and is not passed on (easyrobot: noise above it rocks
	// a still pool's surface)
	liquidHold = liquidG / 128
	liquidMove = 16 // most units Move lifts at once
	// liquidTurn: the engine's gravity turns at most this far per step
	// (radians, 30°) towards the gravity set. Handed a sharp turn at once
	// (the badge turned over, 180°), r3 left the pool glued to the old floor
	// for good: its solved velocities settled under the rest bar (RestQ8),
	// below which nothing moves. Turns of up to 90° flow; a reversal so
	// arrives as a half-second sweep (reported to easyrobot)
	liquidTurn = math.Pi / 6
)

// flipFluid is EasyRobot's flip liquid as a fluid.
type flipFluid struct {
	l      *flip.Liquid
	gx, gy int32 // the gravity set (engine units); the engine's own (l.GX, l.GY) turns towards it (liquidTurn)
	picked [liquidMove]int
	half   [(waterW / 2) * (waterH / 2)]uint8 // Raster's half-resolution pass
}

func newFlipFluid() *flipFluid {
	l, err := flip.New(liquidW, liquidH, liquidN, flip.DefaultParams())
	if err != nil {
		panic("flip: " + err.Error()) // fixed, valid sizes
	}
	f := &flipFluid{l: l, gy: liquidG}
	l.GX, l.GY = f.gx, f.gy
	return f
}

func (f *flipFluid) Step() {
	f.l.GX, f.l.GY = turnToward(f.l.GX, f.l.GY, f.gx, f.gy)
	f.l.Step()
}

// turnToward is (ax, ay) turned at most liquidTurn towards (bx, by), its
// size moving with it; (bx, by) itself when it is that close.
func turnToward(ax, ay, bx, by int32) (int32, int32) {
	a := math.Atan2(float64(ay), float64(ax))
	b := math.Atan2(float64(by), float64(bx))
	d := math.Remainder(b-a, 2*math.Pi)
	if d <= liquidTurn && d >= -liquidTurn {
		return bx, by
	}
	r := (math.Hypot(float64(ax), float64(ay)) + math.Hypot(float64(bx), float64(by))) / 2
	a += math.Copysign(liquidTurn, d)
	s, c := math.Sincos(a)
	return int32(r * c), int32(r * s)
}

func (f *flipFluid) SetGravity(gx, gy float32) {
	const k = liquidG / gravity
	nx := int32(min(max(gx*k, -liquidGMax), liquidGMax))
	ny := int32(min(max(gy*k, -liquidGMax), liquidGMax))
	if abs32(nx-f.gx)+abs32(ny-f.gy) < liquidHold {
		return // a still badge: the pool stays exactly still
	}
	f.gx, f.gy = nx, ny // the engine's gravity turns there in Step
}

// Raster rasters at half the asked resolution and doubles it: 400 particles
// rastered straight to 64×48 (a third of an engine cell per raster cell)
// light one cell each, a speckle of dots with gaps, not a pool (flash 2 on
// the badge). Halved, a raster cell holds about two particles. w and h are
// even and at most 64×48 (both demos' rasters).
func (f *flipFluid) Raster(dst []uint8, w, h int) {
	hw, hh := w/2, h/2
	half := f.half[:hw*hh]
	f.l.Raster(half, hw, hh)
	for y := range hh {
		row := half[y*hw : (y+1)*hw]
		d0 := dst[2*y*w : (2*y+1)*w]
		d1 := dst[(2*y+1)*w : (2*y+2)*w]
		for x, v := range row {
			d0[2*x], d0[2*x+1] = v, v
			d1[2*x], d1[2*x+1] = v, v
		}
	}
}

func (f *flipFluid) Reset()     { f.l.Reset() }
func (f *flipFluid) Count() int { return len(f.l.X) }

// SetCount reslices the particles within New's capacity: the engine holds
// len(X) particles and sized everything else for the capacity.
func (f *flipFluid) SetCount(n int) {
	n = min(max(n, 1), cap(f.l.X))
	f.l.X, f.l.Y = f.l.X[:n], f.l.Y[:n]
	f.l.VX, f.l.VY = f.l.VX[:n], f.l.VY[:n]
}

func (f *flipFluid) Move(n int, fx, fy float32) int {
	n = min(n, liquidMove, len(f.l.X))
	tx := int32(fx * liquidW * flip.Cell)
	ty := int32(fy * liquidH * flip.Cell)
	for k := range n {
		best, bestH := -1, int64(math.MinInt64)
		for i := range f.l.X {
			if f.wasPicked(i, k) {
				continue
			}
			// height against gravity: the surface is where it is greatest
			h := -(int64(f.l.X[i])*int64(f.gx) + int64(f.l.Y[i])*int64(f.gy))
			if h > bestH {
				best, bestH = i, h
			}
		}
		f.picked[k] = best
		// a small fan around the target, so the drops don't sit on one point
		f.l.X[best] = uint16(min(max(tx+int32(k%4-2)*flip.Cell/4, 0), liquidW*flip.Cell-1))
		f.l.Y[best] = uint16(min(max(ty+int32(k/4-2)*flip.Cell/4, 0), liquidH*flip.Cell-1))
		f.l.VX[best], f.l.VY[best] = 0, 0
	}
	return n
}

func (f *flipFluid) wasPicked(i, k int) bool {
	for _, p := range f.picked[:k] {
		if p == i {
			return true
		}
	}
	return false
}

// liquidShare is the one engine shared by Water and Fire's fuel (Leader
// 2026-11-13): one liquid demo is active at a time; showing the other
// claims the engine and lays its liquid afresh. The inactive one doesn't
// step: its tile keeps its last frame. Each claims the amount it needs
// (owner 2026-11-13): Water a basin, Fire a thin layer of fuel.
type liquidShare struct {
	fl    fluid
	owner any
	n     int // the amount of liquid the owner claimed
}

// The amounts of liquid (particles; the engine holds liquidN).
const (
	waterN = liquidN // the Water page and full screen: a basin a third full
	fireN  = 32      // Fire's fuel: a thin layer on the floor (2 of 48 raster rows, about 4 %; the floor covered)
)

// claim makes d the engine's user with n units of liquid, re-seeding it
// if either changed; it reports whether it did.
func (s *liquidShare) claim(d any, n int) bool {
	if s.owner == d && s.n == n {
		return false
	}
	s.owner, s.n = d, n
	s.fl.SetCount(n)
	s.fl.Reset()
	return true
}

var gravityNames = []string{"down", "dn-left", "left", "up-left", "up", "up-right", "right", "dn-right"}

// gravity vector per direction (screen coordinates, y down)
var gravityVec = [8][2]int{{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}}

// gravityOf is the unit vector of a lattice direction (gravityVec index),
// for gravity set by the buttons, at |g| (m/s²).
func gravityOf(dir int) (float32, float32) {
	g := gravityVec[dir&7]
	k := float32(gravity)
	if g[0] != 0 && g[1] != 0 {
		k *= 0.70710678 // diagonals: still |g|
	}
	return float32(g[0]) * k, float32(g[1]) * k
}

// latticeSplit splits a gravity vector (screen axes) between the two
// lattice directions next to it: dir the nearer (index into gravityVec:
// down, down-left, left, … clockwise), alt the other, pAlt (0..128 of 256)
// the odds of alt — in proportion to the angle. Fire's flames lean along it;
// Water shades along dir.
func latticeSplit(gx, gy float32) (dir, alt int, pAlt uint32) {
	// angle from "down", clockwise: -atan2(gx, gy), in eighths of a turn
	u := math.Mod(-math.Atan2(float64(gx), float64(gy))/(math.Pi/4)+8, 8)
	base := math.Floor(u)
	frac := u - base
	if frac < 0.5 {
		return int(base) & 7, int(base+1) & 7, uint32(frac * 256)
	}
	return int(base+1) & 7, int(base) & 7, uint32((1 - frac) * 256)
}
