package app

import (
	"math"
	"strings"
	"testing"
)

// The Motion page shows the Kalman position through a low-pass at the
// chosen output bandwidth (20 Hz, or 10 Hz: `motion bw`): through a stroke the
// shown position trails the estimator's, more at 10 Hz, and once still it is
// the estimator's again; the estimator itself is untouched.
func TestMotionPageOutputBandwidth(t *testing.T) {
	lag := func(bw string) (maxLag, end float64) {
		a, tm := motionApp()
		a.Command("motion", "bw "+bw)
		a.Update(a.now)
		m := a.motion
		tr := &newMotionDemo(m).track
		drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke([3]float64{0, 0, 1}, 0.05, *tm+0.1, 0.3), func(int) {
			maxLag = max(maxLag, math.Abs(float64(m.kal.P[2]-tr.at()[2])))
		})
		return maxLag, math.Abs(float64(m.kal.P[2] - tr.at()[2]))
	}
	l20, e20 := lag("20")
	l10, e10 := lag("10")
	if !(l20 > 1e-4 && l10 > 1.5*l20) || e20 > 1e-4 || e10 > 1e-4 {
		t.Fatalf("a 5 cm stroke in 0.3 s: the shown position trailed by up to %.2f mm at 20 Hz, %.2f mm at 10 Hz (want some, about twice at 10 Hz); still, off by %.3f / %.3f mm (want 0)", l20*1000, l10*1000, e20*1000, e10*1000)
	}
	a, _ := motionApp()
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, s) }
	a.Command("motion", "bw 15")
	a.Update(a.now)
	if out := strings.Join(lines, "\n"); a.motion.bw != bw20 || !strings.Contains(out, "bw 20|10") {
		t.Fatalf("motion bw 15: %q, bandwidth %d; want refused, still 20 Hz", out, a.motion.bw)
	}
}
