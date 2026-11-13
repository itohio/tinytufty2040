package app

import "testing"

// The light reading is steady (owner 2026-11-13: "light level is all
// whacky"): a lit room's noisy readings (±25 %, a 60000 spike now and then
// from flicker) read ~50 % on the log scale and the readout moves by at most
// a step; a real change (a hand over the sensor) shows within 8 samples
// (1.6 s at the 200 ms sample period).
func TestLightReadingIsSteady(t *testing.T) {
	var f lightFilter
	shown := 0
	noise := []uint16{1000, 1250, 800, 1100, 60000, 900, 1200, 760, 1050, 980}
	for i := range 30 { // settle
		shown = steadyPct(shown, f.add(noise[i%len(noise)]))
	}
	lo, hi := 101, -1
	for i := range 100 {
		shown = steadyPct(shown, f.add(noise[i%len(noise)]))
		lo, hi = min(lo, shown), max(hi, shown)
	}
	if lo < 45 || hi > 55 || hi-lo > 1 {
		t.Errorf("lit room: readout %d..%d %%, want ~50 and moving at most 1", lo, hi)
	}
	for range 8 { // a hand over the sensor
		shown = steadyPct(shown, f.add(70))
	}
	if shown > 10 {
		t.Errorf("covered: readout %d %% after 8 samples, want near 0", shown)
	}
}

// The log scale: 64 (dark) is 0 %, 16384 (sunlight) 100 %, each doubling
// 12.5 %; a lit room (raw ~1000) is about half.
func TestLightPercentIsLogarithmic(t *testing.T) {
	for _, c := range []struct {
		raw  float32
		want float32
	}{{0, 0}, {64, 0}, {128, 12.5}, {1024, 50}, {16384, 100}, {65535, 100}} {
		if got := lightPct(c.raw); got < c.want-0.01 || got > c.want+0.01 {
			t.Errorf("raw %v: %.2f %%, want %.1f", c.raw, got, c.want)
		}
	}
}
