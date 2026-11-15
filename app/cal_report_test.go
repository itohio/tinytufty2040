package app

import (
	"strings"
	"testing"
)

// The calibration fit can be read back exactly: `cal` prints the fit
// (model, in use, residual, offsets, scales, gyro bias) and every measured
// orientation's raw pose (the accelerometer's and gyro's means), and a new
// fit logs the same unasked, so a session leaves it in the console log.
// Every line fits the log line (logLineLen).
func TestCalCommandPrintsTheFitAndItsPoses(t *testing.T) {
	off := [3]float32{0.12, -0.25, 0.31}
	sc := [3]float32{1.02, 0.97, 1.01}
	gb := [3]float32{0.004, -0.002, 0.001}
	a, c, wait := orientApp(t)
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, strings.Clone(s)) } // a view of the log ring: copy it
	for f := calFace(0); f < 6; f++ {
		choose(t, a, f)
		measure(c, wait, f, off, sc, gb)
	}
	auto := strings.Join(lines, "\n")
	for _, want := range []string{"cal fit offset+scale in_use 0", "cal offset 0.1200 -0.2500 0.3100 m/s2", "cal scale 1.02000 0.97000 1.01000"} {
		if !strings.Contains(auto, want) {
			t.Fatalf("the six-orientation fit didn't log %q; logged:\n%s", want, auto)
		}
	}
	a.Press(BtnA, false) // use it
	wait(100e6)
	lines = lines[:0]
	a.Command("cal", "")
	a.Update(a.now)
	out := strings.Join(lines, "\n")
	for _, want := range []string{
		"cal fit offset+scale in_use 1",
		"cal offset 0.1200 -0.2500 0.3100 m/s2",
		"cal scale 1.02000 0.97000 1.01000",
		"cal gyro_bias 0.00400 -0.00200 0.00100 rad/s",
		"cal pose screen up acc 0.1200 -0.2500 10.2147",
		"cal pose screen up gyro 0.00400 -0.00200 0.00100",
		"cal pose opposite edge acc 10.1228 -0.2500 0.3100",
	} {
		if !strings.Contains(out, want) {
			t.Errorf("cal: no %q; printed:\n%s", want, out)
		}
	}
	for _, l := range lines {
		if len(l) > logLineLen {
			t.Errorf("cal line longer than the log line (%d > %d): %q", len(l), logLineLen, l)
		}
	}
}
