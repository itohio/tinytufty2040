package app

import (
	"math"
	"testing"
	"time"

	"github.com/itohio/tinygui/examples/framebuffer"
)

// Gravity from the sensor is used at full precision: the liquid keeps the
// exact vector G and falls along the two lattice directions next to it in
// proportion to the angle; lying flat keeps the last vector; the boids' box
// turns by the exact roll. The logged direction snaps with hysteresis.
func TestTiltIsUsedPrecisely(t *testing.T) {
	a := testApp()
	w, f := a.water, a.fire
	ff := w.fl.(*flipFluid)
	const k = liquidG / gravity                  // engine units per m/s²
	at := func(deg float64) (float32, float32) { // badge turned clockwise by deg
		r := deg * math.Pi / 180
		return float32(9.8 * math.Sin(r)), float32(9.8 * math.Cos(r))
	}
	for _, c := range []struct {
		deg            float64
		dir, alt, pAlt int
	}{{0, 0, 1, 0}, {90, 6, 7, 0}, {-90, 2, 3, 0}, {180, 4, 5, 0}, {30, 7, 0, 85}, {-10, 0, 1, 56}, {5, 0, 7, 28}} {
		a.SetTilt(at(c.deg))
		r := c.deg * math.Pi / 180
		if dx, dy := float64(ff.gx)-9.8*k*math.Sin(r), float64(ff.gy)-9.8*k*math.Cos(r); math.Abs(dx) > 2 || math.Abs(dy) > 2 {
			t.Fatalf("turned %v°: the engine's G (%d, %d), want %.0f·(sin, cos)", c.deg, ff.gx, ff.gy, 9.8*k)
		}
		if w.dir != c.dir || f.pAlt > 0 && f.alt != c.alt || int(f.pAlt)-c.pAlt < -1 || int(f.pAlt)-c.pAlt > 1 {
			t.Fatalf("turned %v°: falls %s, flames %d/256 %s; want %s, %d/256 %s", c.deg,
				gravityNames[w.dir], f.pAlt, gravityNames[f.alt], gravityNames[c.dir], c.pAlt, gravityNames[c.alt])
		}
	}
	a.SetTilt(0.5, 0.5) // lying flat: keep the last vector
	if want := 9.8 * k * math.Sin(5*math.Pi/180); w.dir != 0 || math.Abs(float64(ff.gx)-want) > 2 {
		t.Fatalf("flat: the engine's G (%d, %d), want the 5° one kept (x %.0f)", ff.gx, ff.gy, want)
	}
	a.SetTilt(at(0))
	a.SetTilt(at(25)) // just past 22.5°: the logged direction holds (hysteresis)
	if a.tiltDir != 0 {
		t.Fatalf("25°: logged %s, want down", gravityNames[a.tiltDir])
	}
	a.SetTilt(at(35))
	if a.tiltDir != 7 {
		t.Fatalf("35°: logged %s, want dn-right", gravityNames[a.tiltDir])
	}
}

func testApp() *App {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	a := New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id}, time.Now())
	a.boot.on = false // the tests start after the gopher
	return a
}
