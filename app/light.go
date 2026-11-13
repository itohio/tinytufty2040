package app

import "math"

// The light sensor (a phototransistor on an ADC) is noisy, picks up the
// backlight's PWM and room flicker, and is logarithmic in practice: raw
// ~1000 in a lit room, tens in the dark, tens of thousands in sunlight
// (measured on the badge). Shown linearly (raw·100/65535) it sat at 1-2 %
// and jumped with every sample (owner 2026-11-13: "light level is all
// whacky"). So: a median of the last samples (drops spikes), an EMA (calms
// the rest), a log scale, and a readout that moves only by whole steps.

// lightMedianN is the median window, in samples (one per sample period,
// 200 ms: one second).
const lightMedianN = 5

// lightFilter steadies the readings: the median of the last lightMedianN
// raw readings, put on the log scale (lightPct), then an exponential
// average there (a quarter of the way per sample: on the log scale a hand
// over the sensor and a lamp switched on settle equally fast). Fixed
// arrays: nothing allocates per sample.
type lightFilter struct {
	win  [lightMedianN]uint16
	n, i int
	med  uint16  // the median raw reading
	pct  float32 // the filtered light level, 0..100 %
}

// add takes a raw reading and returns the filtered light level (0..100 %).
func (f *lightFilter) add(raw uint16) float32 {
	f.win[f.i] = raw
	f.i = (f.i + 1) % lightMedianN
	if f.n < lightMedianN {
		f.n++
	}
	s := f.win // a copy to sort: the window keeps its order
	w := s[:f.n]
	for i := 1; i < len(w); i++ { // insertion sort: five values
		for j := i; j > 0 && w[j] < w[j-1]; j-- {
			w[j], w[j-1] = w[j-1], w[j]
		}
	}
	f.med = w[len(w)/2]
	p := lightPct(float32(f.med))
	if f.n == 1 {
		f.pct = p // the first reading: nothing to average with
	} else {
		f.pct += (p - f.pct) / 4
	}
	return f.pct
}

// Log scale: lightDark (raw) is 0 %, lightSun is 100 %, in equal steps per
// doubling (64 → 16384: 8 doublings, 12.5 % each; a lit room is ~50 %).
const (
	lightDark = 64
	lightSun  = 16384
)

// lightPct maps a filtered raw reading to 0..100 % on the log scale.
func lightPct(raw float32) float32 {
	if raw <= lightDark {
		return 0
	}
	p := float32(math.Log2(float64(raw)/lightDark) / math.Log2(lightSun/lightDark) * 100)
	return min(p, 100)
}

// steadyPct is the readout: it follows pct only when pct has moved a whole
// step or more away from what it shows, so it doesn't flicker between two
// neighbours.
func steadyPct(shown int, pct float32) int {
	if d := pct - float32(shown); d >= 1 || d <= -1 {
		return int(pct + 0.5)
	}
	return shown
}
