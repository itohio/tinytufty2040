package app

import (
	"runtime"
	"slices"
	"testing"
)

// centroid of the cells of a w-wide grid of size n where pick(i) holds, in
// fuel-cell units (scale: fuel cells per grid cell).
func centroid(w, n int, scale float64, pick func(i int) bool) (float64, float64, bool) {
	var sx, sy, k float64
	for i := range n {
		if pick(i) {
			sx, sy, k = sx+float64(i%w)*scale, sy+float64(i/w)*scale, k+1
		}
	}
	return sx / k, sy / k, k > 0
}

// Fire burns on the Demos page while Water holds the shared engine: the
// app builds Fire, then Water takes the engine, and the camera showed the
// Fire tile empty black (Leader, 2026-11-13). Its fuel only stops sloshing;
// the flames keep rising from it.
func TestFireBurnsWhileWaterHasTheEngine(t *testing.T) {
	share := newTestShare()
	f := newFire(share)
	newWater(share)
	if f.active() {
		t.Fatal("Water was built last: it should hold the engine")
	}
	for range 60 {
		f.tick()
	}
	flames := 0
	for _, h := range f.heat {
		if h > 0 && h != fireFuel {
			flames++
		}
	}
	if flames == 0 {
		t.Fatal("no flames after 60 frames while Water holds the engine")
	}
}

// Fire's fuel is a thin layer on the floor that burns and sloshes (owner
// 2026-11-13: "only a thin layer of fuel on the floor ... not 1/5 or 1/3 of
// the whole container"): settled, it lies in the bottom tenth of the box
// and covers most of the floor (the box's wall rows are never lit).
func TestFireFuelIsAThinLayer(t *testing.T) {
	f := newFire(newTestShare())
	for range 600 {
		f.step()
	}
	top, bottom, cols := fireH, -1, 0
	for x := range fireW {
		lit := false
		for y := range fireH {
			if f.occ[y*fireW+x] != 0 {
				top, bottom, lit = min(top, y), max(bottom, y), true
			}
		}
		if lit {
			cols++
		}
	}
	if depth := bottom - top + 1; depth*10 > fireH {
		t.Errorf("the fuel is %d of %d rows deep: want a thin layer (≤ 10 %%)", depth, fireH)
	}
	if cols*10 < fireW*7 {
		t.Errorf("the fuel covers %d of %d columns: want most of the floor", cols, fireW)
	}
}

// Water's tile on the Demos page runs the shared engine with fewer
// particles (owner: "subwindow in preview could run smaller resolution
// (less particles) to speed up the preview"); its page and full screen run
// the whole basin; switching between them allocates nothing.
func TestWaterPreviewRunsFewerParticles(t *testing.T) {
	a, frame := framedApp()
	share := a.water.share
	a.showPage(a.pageIndex("Demos"))
	frame()
	if share.owner != a.water || share.fl.Count() != waterPreviewN {
		t.Fatalf("on Demos: %d particles (Water owns it: %v), want the preview's %d", share.fl.Count(), share.owner == a.water, waterPreviewN)
	}
	a.showPage(a.pageIndex("Water"))
	frame()
	if share.fl.Count() != waterN {
		t.Fatalf("on the Water page: %d particles, want %d", share.fl.Count(), waterN)
	}
	a.showPage(a.pageIndex("Fire"))
	frame()
	if share.owner != a.fire || share.fl.Count() != fireN {
		t.Fatalf("on the Fire page: %d particles, want the fuel's %d", share.fl.Count(), fireN)
	}
	if runtime.Compiler != "gc" { // Go's type-assertion caches fill at random (go-soak-allocs)
		needMallocCount(t)
		switching := func() {
			a.showPage(a.pageIndex("Demos"))
			frame()
			a.showPage(a.pageIndex("Water"))
			frame()
		}
		switching()
		if n := testing.AllocsPerRun(10, switching); n != 0 {
			t.Fatalf("%.0f allocations switching Demos ↔ Water", n)
		}
	}
}

// The flames rise against gravity, whichever way it points: tilting the
// badge tilts the fire. Measured from the bulk of the fuel (its median
// along gravity): after a reversal flip leaves a few particles on the old
// floor (EasyRobot's open reversal item; 5 of 32 here), which is the
// engine's, not where the flames go.
func TestFireFlamesRiseAgainstGravity(t *testing.T) {
	for _, d := range []int{0, 2, 4, 6} {
		f := newFire(newTestShare())
		f.dir = d
		f.useDir()
		for range 300 { // a thin layer travels the whole box before it lies on the new floor
			f.step()
		}
		g := gravityVec[d]
		var along []float64 // each fuel cell's position along gravity
		for i, v := range f.occ {
			if v != 0 {
				along = append(along, float64(i%fireW*g[0]+i/fireW*g[1]))
			}
		}
		hx, hy, okH := centroid(heatW, heatW*heatH, float64(fireW)/heatW, func(i int) bool {
			return f.heat[i] > fireMax/2 && f.heat[i] <= fireMax // flames, not fuel marks
		})
		if len(along) == 0 || !okH {
			t.Fatalf("gravity %s: fuel cells %d, flames %v", gravityNames[d], len(along), okH)
		}
		slices.Sort(along)
		fuel := along[len(along)/2]
		if rel := hx*float64(g[0]) + hy*float64(g[1]) - fuel; rel > -2 {
			t.Errorf("gravity %s: flames are %.1f cells along gravity from the fuel, want above it (< -2)", gravityNames[d], rel)
		}
	}
}

// Flames rise straight against gravity: from a narrow source, the heat 20
// or more cells away is centred on it (owner 2026-11-13: "fire seems to be
// going left" — the classic Doom-fire shift of +1..-2 leaned 41 cells, and
// with a symmetric shift the fixed scan order still won every collision for
// one side: 13 cells). Both scan branches: gravity down and gravity left.
func TestFireRisesStraight(t *testing.T) {
	for _, d := range []int{0, 2} {
		f := newFire(newTestShare())
		f.height = 2
		f.dir = d
		f.useDir()
		g := gravityVec[d]
		var sum, n float64
		for s := range 400 {
			for k := -4; k <= 4; k++ { // a 9-cell source against the wall gravity points to
				x, y := heatW/2+k, heatH-1
				if g[0] != 0 {
					x, y = 0, heatH/2+k
				}
				f.heat[y*heatW+x] = fireFuel
			}
			f.spread()
			if s < 100 {
				continue
			}
			for i, v := range f.heat {
				x, y := i%heatW, i/heatW
				along, across := heatH-1-y, x-heatW/2
				if g[0] != 0 {
					along, across = x, y-heatH/2
				}
				if v > 0 && v <= fireMax && along >= 20 {
					sum += float64(v) * float64(across)
					n += float64(v)
				}
			}
		}
		if n == 0 {
			t.Fatalf("gravity %s: no flames 20 cells from the source", gravityNames[d])
		}
		if lean := sum / n; lean < -3 || lean > 3 {
			t.Errorf("gravity %s: flames lean %.1f cells sideways 20+ cells from the source, want within ±3", gravityNames[d], lean)
		}
	}
}

// A gravity picked on the page holds against the IMU while the page is
// shown (verify2 fire-thin @1225: on a badge with an IMU, the shared motion
// state put its own "down" back at every sample, so dn-left never took);
// showing the page again follows the badge.
func TestPickedGravityHoldsAgainstTheIMU(t *testing.T) {
	a, frame := framedApp() // the IMU reads an upright badge every frame
	a.showPage(a.pageIndex("Fire"))
	frame()
	a.fire.pick(1) // dn-left, as the page's Gravity choice does
	a.water.pick(2)
	for range 30 {
		frame()
	}
	if a.fire.dir != 1 || a.water.dir != 2 {
		t.Fatalf("after 1 s of IMU samples: fire %s, water %s; want the picks dn-left, left", gravityNames[a.fire.dir], gravityNames[a.water.dir])
	}
	a.showPage(a.pageIndex("Water"))
	a.showPage(a.pageIndex("Fire"))
	frame()
	if a.fire.dir == 1 {
		t.Fatal("shown again, Fire still holds the pick: want the badge's gravity")
	}
}

// The Fire tile on the Demos page burns the same way up as the Fire page,
// whichever way the badge is held (owner, TG-5 #1: the preview's orientation
// was wrong). On Demos, Water holds the shared engine, so Fire's fuel used to
// stay where it last lay while the flames followed the IMU: turned to the
// side, the flames rose sideways from fuel on the old floor. The fuel must
// lie on the floor gravity points at, and the flames rise from it.
func TestFireTileFuelLiesOnTheCurrentFloor(t *testing.T) {
	for _, d := range []int{0, 2, 4, 6} { // down, left, up, right
		share := newTestShare()
		f := newFire(share)
		newWater(share) // Water holds the engine, as on the Demos page
		f.tilt(gravityOf(d))
		for range 60 {
			f.step()
		}
		fx, fy, ok := centroid(fireW, fireW*fireH, 1, func(i int) bool { return f.occ[i] != 0 })
		if !ok {
			t.Fatalf("gravity %s: no fuel on the tile", gravityNames[d])
		}
		g := gravityVec[d]
		// the fuel's position along gravity, from the box's centre: near the
		// floor means well past the centre (a quarter of the box)
		along := (fx-fireW/2)*float64(g[0]) + (fy-fireH/2)*float64(g[1])
		half := float64(fireH) / 2
		if g[0] != 0 {
			half = float64(fireW) / 2
		}
		if along < half/2 {
			t.Errorf("gravity %s: the tile's fuel sits %.1f cells along gravity from the centre, want on the floor (> %.0f)", gravityNames[d], along, half/2)
		}
	}
}
