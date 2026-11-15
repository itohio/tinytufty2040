package app

import (
	"math"
	"testing"
)

// A badge standing upright on the desk, slid 5 cm and back at a hand's
// steady pace (1.5 s each way, no rotation), reads 5 cm at the far end and
// 0 back at the start, sideways (its x) and away from you (its −z) alike,
// on the Motion page's line (the shown Kalman position).
func TestASlideOnTheDeskReadsItsLength(t *testing.T) {
	x, _, z := badgeAxes(0)
	away := [3]float64{-z[0], -z[1], -z[2]}
	for _, c := range []struct {
		name string
		dir  [3]float64
	}{{"sideways", x}, {"away", away}} {
		a, tm := motionApp()
		m := a.motion
		t0 := *tm
		const d, T = 0.05, 1.5
		out, back := stroke(c.dir, d, t0+0.5, T), stroke(c.dir, -d, t0+0.5+T+1, T)
		var far float64 // along the slide, at the far end's pause
		drive(a, tm, int((T*2+3)*imuRate), func(float64) float64 { return 0 }, func(t float64) [3]float64 {
			o, b := out(t), back(t)
			return [3]float64{o[0] + b[0], o[1] + b[1], o[2] + b[2]}
		}, func(i int) {
			if float64(i)/imuRate < 0.5+T+0.9 {
				p := m.shown()
				far = float64(p[0])*c.dir[0] + float64(p[1])*c.dir[1] + float64(p[2])*c.dir[2]
			}
		})
		p := m.shown()
		end := math.Sqrt(float64(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]))
		if far < 0.04 || far > 0.06 || end > 0.01 {
			t.Errorf("%s: %.1f cm at the far end, %.1f cm from the start back home; want 5 then 0", c.name, far*100, end*100)
		}
	}
}
