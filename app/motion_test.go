package app

import (
	"github.com/itohio/EasyRobot/x/math/nav/calib"
	"math"
	"testing"
)

// A badge held still at a 30° roll with a biased gyro and a calibration in
// use that knows the bias: the attitude takes the bias out, so the filter's
// gravity drives the demos at the exact angle after 40 s (no drift; without
// the bias its yaw would turn 0.031 rad/s and the roll wander).
func TestIMUBiasAndGravity(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	w := a.water
	r := 30 * math.Pi / 180 // turned clockwise: up leans towards screen-left (-X)
	acc := [3]float32{float32(-gravity * math.Sin(r)), float32(gravity * math.Cos(r)), 0}
	bias := [3]float32{0.012, -0.02, 0.031}
	m := a.motion
	m.usePrior(&calib.Fit{Model: calib.OffsetScale, Scale: [3]float32{1, 1, 1}, GyroBias: bias})
	for range 600 { // 40 s at 15 Hz
		imuFrame(a, acc, bias, 1.0/15)
	}
	// 30° clockwise: the engine's G = 1311·(sin 30°, cos 30°) = (655, 1135)
	ff := w.fl.(*flipFluid)
	if dx, dy := ff.gx-655, ff.gy-1135; dx < -10 || dx > 10 || dy < -10 || dy > 10 {
		t.Fatalf("water's gravity (%d, %d), want (655, 1135): 30°", ff.gx, ff.gy)
	}
	// roll from the filter, within half a degree
	if d := math.Abs(float64(m.up[0]) + math.Sin(r)); d > 0.01 {
		t.Fatalf("filter's up %v, want x = -sin 30°", m.up)
	}
}

// The Motion page's angles are the badge's own (seen on the badge:
// upright it read roll −179°): upright, screen facing you, is roll 0,
// pitch 0, yaw 0 (from the last path reset); turned clockwise like a
// steering wheel is +roll; the top tipped towards you is +pitch.
func TestBadgeAnglesReadZeroUpright(t *testing.T) {
	deg := func(r float32) float64 { return float64(r) * 180 / math.Pi }
	for _, c := range []struct {
		name        string
		acc         [3]float32 // what the accelerometer reads at rest (up, badge axes)
		roll, pitch float64
	}{
		{"upright", [3]float32{0, gravity, 0}, 0, 0},
		{"turned 30° clockwise", [3]float32{float32(-gravity * math.Sin(math.Pi/6)), float32(gravity * math.Cos(math.Pi/6)), 0}, 30, 0},
		{"top 20° towards you", [3]float32{0, float32(gravity * math.Cos(math.Pi/9)), float32(-gravity * math.Sin(math.Pi/9))}, 0, 20},
	} {
		a := testApp()
		a.motion = newMotion()
		m := a.motion
		for range 600 {
			imuFrame(a, c.acc, [3]float32{}, 1.0/15)
		}
		m.resetPath()
		imuFrame(a, c.acc, [3]float32{}, 1.0/15)
		r, p, y := deg(m.rpy[0]), deg(m.rpy[1]), deg(m.rpy[2])
		if math.Abs(r-c.roll) > 1 || math.Abs(p-c.pitch) > 1 || math.Abs(y) > 1 {
			t.Errorf("%s: roll %.1f pitch %.1f yaw %.1f, want %.0f, %.0f, 0", c.name, r, p, y, c.roll, c.pitch)
		}
	}
}

// quaternion helpers for exact synthetic trajectories (w x y z, badge → world)
type q4 [4]float64

func qmul(a, b q4) q4 {
	return q4{a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3], a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
		a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1], a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]}
}
func qconj(a q4) q4 { return q4{a[0], -a[1], -a[2], -a[3]} }
func qrot(a q4, v [3]float64) [3]float64 {
	r := qmul(qmul(a, q4{0, v[0], v[1], v[2]}), qconj(a))
	return [3]float64{r[1], r[2], r[3]}
}
