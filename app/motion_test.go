package app

import (
	"math"
	"testing"
)

// A badge held still at a 30° roll with a biased gyro: the bias is learnt
// from the resting samples and the filter's gravity drives the demos at
// the exact angle (no drift from the bias).
func TestIMUBiasAndGravity(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	w := a.water
	r := 30 * math.Pi / 180 // turned clockwise: up leans towards screen-left (-X)
	acc := [3]float32{float32(-gravity * math.Sin(r)), float32(gravity * math.Cos(r)), 0}
	bias := [3]float32{0.012, -0.02, 0.031}
	for range 600 { // 40 s at 15 Hz
		imuFrame(a, acc, bias, 1.0/15)
	}
	m := a.motion
	for i := range bias {
		if d := m.bias[i] - bias[i]; d > 1e-3 || d < -1e-3 {
			t.Fatalf("gyro bias %v, want %v", m.bias, bias)
		}
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

// Shaking (large gyro rates) is not rest: the bias stays put.
func TestIMUBiasIgnoresMotion(t *testing.T) {
	m := newMotion()
	acc := [3]float32{0, gravity, 0}
	for range restFirst {
		m.learnBias(acc, [3]float32{0.01, 0, 0})
	}
	for range 200 {
		m.learnBias(acc, [3]float32{2, -3, 1})
	}
	if d := m.bias[0] - 0.01; d > 1e-3 || d < -1e-3 || m.bias[1] != 0 {
		t.Fatalf("bias %v moved with motion, want {0.01 0 0}", m.bias)
	}
}

// The Motion page's path follows translation (owner 2026-11-13: "it must
// track PATH … not one of the axes"): a badge held upright, pushed 1.5 m/s²
// to its right for 0.2 s and braked as long, ends ~6 cm to the right and
// stays there at rest; resting with sensor noise doesn't wander.
func TestMotionPathFollowsAPush(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	m := a.motion
	up := [3]float32{0, gravity, 0} // upright: +Y up the screen
	r := uint32(1)
	noisy := func(v [3]float32, amp float32) [3]float32 {
		for i := range v {
			r = r*1664525 + 1013904223
			v[i] += (float32(r>>8)/(1<<24) - 0.5) * 2 * amp
		}
		return v
	}
	for range 20 * imuRate { // 20 s at 100 Hz (the FIFO's rate): bias learnt, orientation settled
		imuFrame(a, noisy(up, 0.05), noisy([3]float32{}, 0.005), 1.0/imuRate)
	}
	m.resetPath()
	for range 10 * imuRate { // 10 s more at rest
		imuFrame(a, noisy(up, 0.05), noisy([3]float32{}, 0.005), 1.0/imuRate)
	}
	if d := math.Hypot(math.Hypot(float64(m.nav.P[0]), float64(m.nav.P[1])), float64(m.nav.P[2])); d > 0.005 {
		t.Fatalf("10 s at rest: the path moved %.1f cm, want < 0.5", d*100)
	}
	right := toWorld(m.ahrs.Output(), [3]float32{1, 0, 0})
	push := func(ax float32, n int) {
		for range n {
			imuFrame(a, [3]float32{ax, gravity, 0}, [3]float32{}, 1.0/imuRate)
		}
	}
	push(1.5, 20) // 0.2 s
	push(-1.5, 20)
	push(0, 2*imuRate) // 2 s at rest
	along := m.nav.P[0]*right[0] + m.nav.P[1]*right[1] + m.nav.P[2]*right[2]
	if along < 0.04 || along > 0.08 {
		t.Errorf("pushed right: moved %.1f cm along the badge's right, want ~6 (4..8); pos %v", along*100, m.nav.P)
	}
	if v := m.nav.V; v[0]*v[0]+v[1]*v[1]+v[2]*v[2] > 1e-6 {
		t.Errorf("at rest after the push: velocity %v, want zero (ZUPT)", v)
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
