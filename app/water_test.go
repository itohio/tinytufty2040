package app

import (
	"math"
	"math/rand/v2"
	"testing"
)

func newTestShare() *liquidShare { return &liquidShare{fl: newFlipFluid()} }

// litCentre is the centre of the lit cells of a w-wide raster (cells).
func litCentre(cells []uint8, w int) (x, y float64, n int) {
	for i, v := range cells {
		if v != 0 {
			x += float64(i % w)
			y += float64(i / w)
			n++
		}
	}
	if n == 0 {
		return 0, 0, 0
	}
	return x / float64(n), y / float64(n), n
}

// A resting pool shows as a body of water, not a speckle of dots (flash 2
// on the badge: 400 particles rastered straight to 64×48 lit 400 cells
// with gaps between every one): below the surface of each column, at most
// a few cells inside the pool are dark.
func TestRestingPoolHasNoHoles(t *testing.T) {
	w := newWater(newTestShare())
	for range 1200 {
		w.step()
	}
	raw := make([]uint8, waterW*waterH)
	w.fl.Raster(raw, waterW, waterH)
	lit, holes := 0, 0
	for x := range waterW {
		top := -1
		for y := range waterH {
			if raw[y*waterW+x] != 0 {
				top = y
				break
			}
		}
		if top < 0 {
			continue
		}
		bottom := top
		for y := waterH - 1; y > top; y-- {
			if raw[y*waterW+x] != 0 {
				bottom = y
				break
			}
		}
		for y := top; y <= bottom; y++ {
			if raw[y*waterW+x] != 0 {
				lit++
			} else {
				holes++
			}
		}
	}
	if lit == 0 || holes*20 > lit {
		t.Fatalf("resting pool: %d lit cells, %d dark cells inside it (want ≤ 5%%)", lit, holes)
	}
}

// Every one of the 8 gravity directions moves the water: settled against
// the opposite side, its centre travels towards the new "down" (owner:
// water "not reacting to gravity change").
func TestWaterFollowsEveryGravityDirection(t *testing.T) {
	for d := range gravityVec {
		w := newWater(newTestShare())
		w.dir = (d + 4) % 8 // settle on the opposite side first
		w.useDir()
		for range 300 {
			w.step()
		}
		x0, y0, _ := litCentre(w.shade[:], waterW)
		w.dir = d
		w.useDir()
		for range 300 {
			w.step()
		}
		x1, y1, _ := litCentre(w.shade[:], waterW)
		g := gravityVec[d]
		if along := (x1-x0)*float64(g[0]) + (y1-y0)*float64(g[1]); along < 5 {
			t.Errorf("gravity %s: the water's centre moved %.1f cells along it, want towards it (> 5)", gravityNames[d], along)
		}
	}
}

// Gravity is a continuous vector (owner: "the imu is at a slight tilt and
// water does not move that way as it should"): tilted 15° to the right, the
// water's centre sits right of where it sits upright.
func TestWaterLeansWithASlightTilt(t *testing.T) {
	centre := func(deg float64) float64 {
		w := newWater(newTestShare())
		r := deg * math.Pi / 180
		w.setTilt(float32(gravity*math.Sin(r)), float32(gravity*math.Cos(r)))
		for range 400 {
			w.step()
		}
		x, _, _ := litCentre(w.shade[:], waterW)
		return x
	}
	if flat, tilted := centre(0), centre(15); tilted-flat < 1 {
		t.Errorf("tilted 15° right: the water's centre at %.1f, upright %.1f; want it further right", tilted, flat)
	}
}

// Owner: water and fire "jumping around while the badge is still". With the
// badge still (gravity steady but for sensor noise, σ 0.02 m/s² per axis as
// measured on the badge) the liquid comes to rest: after settling, the
// raster doesn't change. Upright, at the badge's usual lean, on a diagonal.
func TestStillBadgeLiquidComesToRest(t *testing.T) {
	r := rand.New(rand.NewPCG(1, 2))
	for _, deg := range []float64{0, 6, 45} {
		w := newWater(newTestShare())
		gx, gy := float32(gravity*math.Sin(deg*math.Pi/180)), float32(gravity*math.Cos(deg*math.Pi/180))
		noisy := func() {
			w.setTilt(gx+float32(r.NormFloat64()*0.02), gy+float32(r.NormFloat64()*0.02))
			w.step()
		}
		for range 1200 { // a tilted pool takes ~1000 steps to flow into place (r4, 24×18)
			noisy()
		}
		prev := w.shade
		moved := 0
		for range 200 {
			noisy()
			if w.shade != prev {
				moved++
				prev = w.shade
			}
		}
		if moved > 0 {
			t.Errorf("still badge at %v°: the water's raster changed in %d of 200 steps after settling", deg, moved)
		}
	}
}

// Pouring moves water, it adds none: the stream comes from the surface.
func TestWaterPourMovesNotAdds(t *testing.T) {
	w := newWater(newTestShare())
	n := w.count()
	w.pour = true
	for range 100 {
		w.step()
	}
	if w.count() != n {
		t.Fatalf("pouring changed the water from %d to %d units", n, w.count())
	}
	_, _, lit := litCentre(w.shade[:], waterW)
	if lit == 0 {
		t.Fatal("no water lit while pouring")
	}
}

// One engine for Water and Fire's fuel (2026-11-13): showing Fire
// takes it and lays the fuel afresh; Water then doesn't step (its tile keeps
// its last frame) until it takes the engine back.
func TestLiquidEngineIsShared(t *testing.T) {
	share := newTestShare()
	f := newFire(share)
	w := newWater(share)
	if !w.active() || f.active() {
		t.Fatal("at start Water should hold the engine")
	}
	w.turn(2) // gravity left: the water runs left
	for range 50 {
		w.step()
	}
	f.claim()
	if !f.active() || w.active() {
		t.Fatal("showing Fire should hand it the engine")
	}
	frozen := w.shade
	for range 50 {
		w.step()
		f.step()
	}
	if w.shade != frozen {
		t.Error("Water stepped while Fire held the engine")
	}
	w.claim()
	if !w.active() {
		t.Fatal("showing Water again should take the engine back")
	}
}
