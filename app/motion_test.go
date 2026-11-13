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

// Full-screen Pong: your paddle follows the badge up and down 1:1 on the
// glass (owner 2026-11-13: "move the board up or down and the paddle move up
// or down with the same speed and motion"): the paddle moves by the badge's
// height change × pongCellsPerM, holds at rest, and at an edge it stops
// and turns back with the badge at once.
func TestPongPaddleFollowsTheBadge(t *testing.T) {
	// 1:1 on the glass, from the panel, not from the code's constant: 320 px
	// across 40.8 mm, full-screen Pong draws 5 px per cell
	const cellsPerM = 320.0 / 40.8 / 5 * 1000
	a := testApp()
	a.motion = newMotion()
	a.pong.motion = a.motion
	m, p := a.motion, a.pong
	upright := [3]float32{0, gravity, 0}
	sample := func(ay float32) {
		imuFrame(a, [3]float32{0, gravity + ay, 0}, [3]float32{}, 1.0/15)
		p.tick()
	}
	for range 300 { // settle at rest
		imuFrame(a, upright, [3]float32{}, 1.0/15)
	}
	p.pad[0] = (pongH - pongPadLen) * pongFix // start at the bottom: room to rise 40 cells
	p.showFull()
	pad0, z0 := p.pad[0], m.nav.P[2]
	lift := func(ay float32, n int) { // accelerate n samples, brake as long, rest 1 s
		for range n {
			sample(ay)
		}
		for range n {
			sample(-ay)
		}
		for range 15 {
			sample(0)
		}
	}
	lift(0.8, 2) // up about a centimetre
	dz := m.nav.P[2] - z0
	moved := float32(pad0-p.pad[0]) / pongFix // cells up the screen
	if dz < 0.005 {
		t.Fatalf("the badge rose %.1f mm, want > 5", dz*1000)
	}
	if want := dz * cellsPerM; moved < want-0.5 || moved > want+0.5 {
		t.Fatalf("badge up %.1f mm: paddle up %.1f cells, want %.1f (1:1 on the glass)", dz*1000, moved, want)
	}
	held := p.pad[0]
	for range 30 {
		sample(0)
	}
	if p.pad[0] != held {
		t.Fatalf("at rest the paddle moved from %d to %d", held, p.pad[0])
	}
	// far down, past the bottom: the paddle stops there and the anchor
	// moves with the badge, so the brake's slight rise afterwards lifts it at
	// once, by exactly that rise
	bottom := int32((pongH - pongPadLen) * pongFix)
	lowest, reached := m.nav.P[2], false
	for _, ay := range []float32{-2, -2, -2, 2, 2, 2, 0, 0, 0, 0, 0} {
		sample(ay)
		lowest = min(lowest, m.nav.P[2])
		reached = reached || p.pad[0] == bottom
	}
	if !reached {
		t.Fatal("a big move down never brought the paddle to the bottom")
	}
	rise := m.nav.P[2] - lowest
	if want := float32(bottom-p.pad[0]) / pongFix; rise*cellsPerM < want-0.5 || rise*cellsPerM > want+0.5 {
		t.Fatalf("after the bottom the badge rose %.1f mm: paddle up %.1f cells from the bottom, want %.1f", rise*1000, want, rise*cellsPerM)
	}
	// paused, the paddle still goes with the badge (it is drawn while paused)
	p.fullPress(BtnA)
	pausedAt := p.pad[0]
	lift(0.8, 2)
	if p.pad[0] >= pausedAt {
		t.Fatalf("paused: the badge rose but the paddle stayed at %.1f cells", float32(p.pad[0])/pongFix)
	}
}

// The Motion page's angles are the badge's own (verify2 TG-motion-path:
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
