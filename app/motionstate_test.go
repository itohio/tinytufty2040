package app

import (
	"math"
	"math/rand/v2"
	"testing"

	ui "github.com/itohio/tinygui"
)

// imuRate is the synthetic traces' sample rate: the badge drains the
// sensor's FIFO at 100 Hz.
const imuRate = 100

// badgeAxes are the badge's axes in the world (x east, y away from the
// viewer, z up) when it stands upright facing the viewer, turned clockwise
// (as the viewer sees it) by roll: X its right, Y up its screen, Z out of
// the screen towards the viewer.
func badgeAxes(roll float64) (x, y, z [3]float64) {
	s, c := math.Sincos(roll)
	return [3]float64{c, 0, -s}, [3]float64{s, 0, c}, [3]float64{0, -1, 0}
}

func dot(a, b [3]float64) float64 { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }

// imuFrame feeds one sample and hands the motion state to the demos, as the
// firmware's drain and the Update after it do; here every sample is a frame
// (the demos' state at a frame is the same either way: the hand-off only
// stores the latest values).
func imuFrame(a *App, acc, gyro [3]float32, dt float32) {
	a.SetIMU(acc, gyro, dt)
	a.frameMotion()
}

// drive feeds n samples of a scripted motion: roll(t) (radians) and the
// world acceleration of the badge acc(t) (m/s²). The sensor reads the
// specific force f = R^T(a + g ẑ) in the badge's axes and the rate about its
// screen normal over the interval ending at the sample (a clockwise turn is
// −Z). each runs after every sample.
func drive(a *App, t0 *float64, n int, roll func(t float64) float64, acc func(t float64) [3]float64, each func(i int)) {
	const dt = 1.0 / imuRate
	for i := range n {
		t := *t0 + float64(i)*dt
		r := roll(t)
		rate := (roll(t) - roll(t-dt)) / dt // as a gyro reads it: the turn over the interval ending at the sample
		x, y, z := badgeAxes(r)
		w := acc(t)
		w[2] += gravity
		f := [3]float32{float32(dot(x, w)), float32(dot(y, w)), float32(dot(z, w))}
		imuFrame(a, f, [3]float32{0, 0, float32(-rate)}, dt)
		if each != nil {
			each(i)
		}
	}
	*t0 += float64(n) * dt
}

// smooth goes from 0 to 1 over [0, 1] with zero speed and acceleration at
// both ends (minimum jerk); smoothAcc is its second derivative.
func smooth(u float64) float64 {
	u = min(max(u, 0), 1)
	return u * u * u * (10 - 15*u + 6*u*u)
}

func smoothAcc(u float64) float64 {
	if u <= 0 || u >= 1 {
		return 0
	}
	return 60*u - 180*u*u + 120*u*u*u
}

func noAcc(float64) [3]float64 { return [3]float64{} }

// stroke is the world acceleration of a d-metre move along dir over T
// seconds starting at t1.
func stroke(dir [3]float64, d, t1, T float64) func(t float64) [3]float64 {
	return func(t float64) [3]float64 {
		k := d / (T * T) * smoothAcc((t-t1)/T)
		return [3]float64{dir[0] * k, dir[1] * k, dir[2] * k}
	}
}

func motionApp() (*App, *float64) {
	a := testApp()
	a.motion = newMotion()
	a.pong.motion = a.motion
	t := 0.0
	drive(a, &t, 3*imuRate, func(float64) float64 { return 0 }, noAcc, nil) // settle upright
	return a, &t
}

// A tilt alone doesn't move the badge (owner 2026-11-13: "tilting shows
// motion → gravity leaks into translation"): turning 45° and back over 3 s
// leaves the world position and the badge-frame displacement within 1 cm.
func TestTiltAloneDoesNotTranslate(t *testing.T) {
	a, tm := motionApp()
	m := a.motion
	t1 := *tm
	roll := func(t float64) float64 {
		u := t - t1
		switch {
		case u < 1:
			return math.Pi / 4 * smooth(u)
		case u < 2:
			return math.Pi / 4
		default:
			return math.Pi / 4 * (1 - smooth(u-2))
		}
	}
	worst := 0.0
	drive(a, tm, 4*imuRate, roll, noAcc, func(int) {
		p := m.nav.P
		worst = max(worst, math.Sqrt(float64(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])))
	})
	if worst > 0.01 {
		t.Errorf("turning 45° and back moved the badge up to %.1f cm in the world, want < 1", worst*100)
	}
	if d := m.disp; math.Abs(float64(d[0]))+math.Abs(float64(d[1])) > 0.01 {
		t.Errorf("turning 45° and back: badge-frame displacement %v m, want < 1 cm", d)
	}
}

// The badge-frame displacement follows the badge's own up axis: tilted
// 45°, a move along the badge's up axis moves it 1:1, a move straight up in
// the world by cos 45°.
func TestDisplacementFollowsTheBadgesOwnUpAxis(t *testing.T) {
	const d = 0.015 // m
	for _, c := range []struct {
		name string
		dir  func(x, y [3]float64) [3]float64
		want float64 // m
	}{
		{"along the badge's up axis", func(_, y [3]float64) [3]float64 { return y }, d},
		{"straight up in the world", func(_, _ [3]float64) [3]float64 { return [3]float64{0, 0, 1} }, d * math.Cos(math.Pi/4)},
	} {
		a, tm := motionApp()
		m := a.motion
		t1 := *tm
		tilt := func(t float64) float64 { return math.Pi / 4 * smooth(t-t1) }
		drive(a, tm, 3*imuRate, tilt, noAcc, nil) // turn 45° and rest
		held := func(float64) float64 { return math.Pi / 4 }
		y0 := m.disp[1]
		x, y, _ := badgeAxes(math.Pi / 4)
		// a brisk 0.5 s stroke: gentler than ~0.2 m/s² reads as rest to the strapdown
		drive(a, tm, 3*imuRate, held, stroke(c.dir(x, y), d, *tm, 0.5), nil)
		moved := float64(m.disp[1] - y0)
		if moved < c.want*0.85 || moved > c.want*1.15 {
			t.Errorf("%s %.1f cm: displacement up %.2f cm, want %.2f (±15 %%)", c.name, d*100, moved*100, c.want*100)
		}
	}
}

// The displacement moves in the badge's own frame: tilted 45°, a move along
// the badge's own right edge moves it 1:1; a move along the world's
// horizontal by cos 45°.
func TestDisplacementMovesInTheBadgesFrame(t *testing.T) {
	const d = 0.10
	for _, c := range []struct {
		name string
		dir  func(x [3]float64) [3]float64
		want float64 // m along the badge's right edge
	}{
		{"along the badge's right edge", func(x [3]float64) [3]float64 { return x }, d},
		{"along the world's horizontal", func([3]float64) [3]float64 { return [3]float64{1, 0, 0} }, d * math.Cos(math.Pi/4)},
	} {
		a, tm := motionApp()
		m := a.motion
		t1 := *tm
		drive(a, tm, 3*imuRate, func(t float64) float64 { return math.Pi / 4 * smooth(t-t1) }, noAcc, nil)
		x0 := m.disp[0]
		x, _, _ := badgeAxes(math.Pi / 4)
		drive(a, tm, 3*imuRate, func(float64) float64 { return math.Pi / 4 }, stroke(c.dir(x), d, *tm, 1), nil)
		moved := float64(m.disp[0] - x0)
		if moved < c.want*0.85 || moved > c.want*1.15 {
			t.Errorf("%s %.0f cm: the displacement moved %.1f cm, want %.1f (±15 %%)", c.name, d*100, moved*100, c.want*100)
		}
	}
}

// The Motion page shows the world path from the badge (owner: "rotate the
// badge and see the path also rotated so i can observe it from my latest
// integrated position point of view"): after a move 5 cm right, where it
// started is left of the view's centre (the badge, now); turned 90°
// clockwise in place, the same start is below the centre.
func TestPathViewTurnsWithTheBadge(t *testing.T) {
	a, tm := motionApp()
	m := a.motion
	tr := &pathTrack{m: m}
	v := &pathView{WidgetBase: ui.NewWidgetBase(ScreenW, ScreenH), t: tr}
	x, _, _ := badgeAxes(0)
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke(x, 0.05, *tm, 0.7), func(int) { tr.record() })
	start := fromMM(tr.pts[0])
	look := func() (float32, float32) {
		v.setCamera(v.back())
		v.ox, v.oy, v.f = ScreenW/2, ScreenH/2, pathFocal*ScreenH
		sx, sy, ok := v.project(start)
		if !ok {
			t.Fatal("the path's start is behind the camera")
		}
		return sx - float32(v.ox), sy - float32(v.oy)
	}
	if dx, dy := look(); dx > -20 || dy < -5 || dy > 5 {
		t.Fatalf("upright, 5 cm right of the start: the start drawn at (%+.0f, %+.0f) from the centre, want well left, level", dx, dy)
	}
	t1 := *tm
	drive(a, tm, 2*imuRate, func(t float64) float64 { return math.Pi / 2 * smooth(t-t1) }, noAcc, nil)
	if dx, dy := look(); dy < 20 || dx < -5 || dx > 5 {
		t.Fatalf("turned 90° clockwise: the start drawn at (%+.0f, %+.0f) from the centre, want well below it", dx, dy)
	}
}

// An accelerometer that reads a little short or long at rest still lets
// the filter settle (the badge reads 9.7 m/s² standing still, 0.06–0.16 m/s²
// short; strapdown r8 measured MPU6050 offsets of 0.3–0.5 m/s²).
// An attitude can only turn the reading, never change its size, so a settle
// test on the size never passed there: the path was never started, and the
// paddle, the displacement and the Motion page never moved.
func TestSettlesDespiteAnAccelerometerScaleError(t *testing.T) {
	for _, scale := range []float32{0.985, 1.015, 0.96, 1.04} {
		a := testApp()
		a.motion = newMotion()
		for range 3 * imuRate {
			imuFrame(a, [3]float32{0, gravity * scale, 0}, [3]float32{}, 1.0/imuRate)
		}
		if !a.motion.settled {
			t.Errorf("accelerometer ×%.3f at rest: not settled after 3 s", scale)
		}
	}
}

// A shake sloshes the water: shaken sideways at 1.5 Hz the water's centre
// moves at least a cell more than on a still badge.
func TestShakeSloshesWater(t *testing.T) {
	run := func(shake bool) (slosh float64) {
		rngState = 2463534242
		a, tm := motionApp()
		w := a.water
		for range 120 { // let the water settle level
			w.step()
		}
		cx := func() float64 {
			var s, n float64
			for i, v := range w.shade {
				if v != 0 {
					s += float64(i % waterW)
					n++
				}
			}
			return s / n
		}
		c0 := cx()
		x, _, _ := badgeAxes(0)
		acc := func(t float64) [3]float64 {
			if !shake {
				return [3]float64{}
			}
			k := 6 * math.Sin(2*math.Pi*1.5*t)
			return [3]float64{x[0] * k, x[1] * k, x[2] * k}
		}
		drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, acc, func(i int) {
			if i%3 == 2 { // a frame every 3 samples
				w.step()
				slosh = max(slosh, math.Abs(cx()-c0))
			}
		})
		return slosh
	}
	sloshStill, sloshShake := run(false), run(true)
	if sloshShake < sloshStill+1 {
		t.Errorf("shaken, the water's centre moved %.2f cells (still badge: %.2f), want at least a cell more", sloshShake, sloshStill)
	}
}

// The path keeps its history while the badge rests: after a 5 cm stroke
// right and one up, and 5 s still, it still starts where the badge started,
// about one point per millimetre moved (comparing metres with the stored
// millimetres, the rounding on two axes exceeded the step at rest: every
// frame stored the same point again and the ring forgot the path).
func TestPathKeepsItsHistoryAtRest(t *testing.T) {
	a, tm := motionApp()
	tr := &pathTrack{m: a.motion}
	x, y, _ := badgeAxes(0)
	each := func(i int) {
		if i%3 == 2 { // once a frame
			tr.record()
		}
	}
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke(x, 0.05, *tm, 0.7), each)
	drive(a, tm, 7*imuRate, func(float64) float64 { return 0 }, stroke(y, 0.05, *tm, 0.7), each)
	if tr.n > 115 {
		t.Errorf("10 cm of strokes and 5 s at rest stored %d points, want about 100", tr.n)
	}
	if s := fromMM(tr.pts[(tr.i-tr.n+pathPoints)%pathPoints]); math.Abs(float64(s[0]))+math.Abs(float64(s[2])) > 0.003 {
		t.Errorf("the path starts at %v, want where the badge started (0)", s)
	}
}

// A still badge keeps the paddle still (seen on the badge: on
// the old motion it drifted 26 px in 20 s): with the badge's sensor noise
// (accelerometer σ 0.02 m/s², gyro σ 0.003 rad/s, a gyro bias learnt at
// rest), paused or playing, the paddle stays within 1 px (full screen: 5 px
// a cell) over 30 s.
func TestStillBadgeKeepsThePaddleStill(t *testing.T) {
	for _, paused := range []bool{true, false} {
		a := testApp()
		a.motion = newMotion()
		a.pong.motion = a.motion
		p := a.pong
		r := rand.New(rand.NewPCG(3, 4))
		bias := [3]float32{0.004, -0.006, 0.002}
		sample := func() {
			acc := [3]float32{float32(r.NormFloat64() * 0.02), gravity + float32(r.NormFloat64()*0.02), float32(r.NormFloat64() * 0.02)}
			gyro := [3]float32{bias[0] + float32(r.NormFloat64()*0.003), bias[1] + float32(r.NormFloat64()*0.003), bias[2] + float32(r.NormFloat64()*0.003)}
			imuFrame(a, acc, gyro, 1.0/imuRate)
		}
		for range 5 * imuRate { // boot: the filter settles, the bias is learnt
			sample()
		}
		p.showFull()
		if paused {
			p.fullPress(BtnA)
		}
		lo, hi := p.pad[0], p.pad[0]
		for i := range 30 * imuRate {
			sample()
			if i%3 == 2 { // a frame every 3 samples
				p.tick()
				lo, hi = min(lo, p.pad[0]), max(hi, p.pad[0])
			}
		}
		const px = pongFix / 5 // one screen pixel in paddle units
		if hi-lo > px {
			t.Errorf("still badge, paused %v: the paddle wandered %.1f px in 30 s, want ≤ 1", paused, float64(hi-lo)/px)
		}
	}
}

// Right after boot, with the badge's own sensor errors (it reads 9.7 m/s²
// at rest; MPU6050 offsets 0.1–0.5 m/s² per axis, strapdown r8), a still
// badge keeps the paddle and the badge-frame displacement still too (seen on the
// badge: the paddle crept 7 px, then 5 px, per 30 s in the first minutes after
// boot while the Motion page read 0 cm: the strapdown's position crept by
// millimetres while it learnt its bias at rest, under the readout's 1 cm).
func TestStillBadgeRightAfterBootKeepsPaddleAndDisplacementStill(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	a.pong.motion = a.motion
	p, m := a.pong, a.motion
	r := rand.New(rand.NewPCG(5, 6))
	off := [3]float32{0.25, -0.35, 0.3} // gravity on +Y: |a| ≈ 9.46 + …
	gbias := [3]float32{0.01, -0.012, 0.006}
	sample := func() {
		acc := [3]float32{off[0] + float32(r.NormFloat64()*0.02), gravity*0.99 + off[1] + float32(r.NormFloat64()*0.02), off[2] + float32(r.NormFloat64()*0.02)}
		gyro := [3]float32{gbias[0] + float32(r.NormFloat64()*0.003), gbias[1] + float32(r.NormFloat64()*0.003), gbias[2] + float32(r.NormFloat64()*0.003)}
		imuFrame(a, acc, gyro, 1.0/imuRate)
	}
	for range 3 * imuRate { // boot: just long enough to settle
		sample()
	}
	if !a.motion.settled {
		t.Fatal("not settled 3 s after boot")
	}
	p.showFull()
	p.fullPress(BtnA) // paused: only the badge moves the paddle
	lo, hi := p.pad[0], p.pad[0]
	d0 := m.disp
	dmax := 0.0
	for i := range 60 * imuRate {
		sample()
		if i%3 == 2 {
			p.tick()
			lo, hi = min(lo, p.pad[0]), max(hi, p.pad[0])
			dmax = max(dmax, math.Abs(float64(m.disp[0]-d0[0])), math.Abs(float64(m.disp[1]-d0[1])))
		}
	}
	const px = pongFix / 5
	if hi-lo > px {
		t.Errorf("the paddle wandered %.1f px in the first 60 s after boot, want ≤ 1", float64(hi-lo)/px)
	}
	if dmax > 0.05 {
		t.Errorf("the badge-frame displacement wandered %.1f cm in the first 60 s after boot, want ≤ 5", dmax*100)
	}
}

// With the badge's own sensor noise the strapdown now and then leaves rest
// for a moment (seen on the badge: rest 97 %, the paddle moved 17 px, 2.2 mm,
// in 30 s on a still badge). At accelerometer noise 0.04 m/s² and gyro
// 0.005 rad/s the host shows the same: rest 96 %, 16 false starts in
// 120 s. The badge-frame displacement must stay still through all of them
// (Pong's paddle follows pitch: TestPongPitchStillHandKeepsThePaddle).
func TestNoisyStillBadgeKeepsTheDisplacementStill(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	m := a.motion
	r := rand.New(rand.NewPCG(7, 8))
	off := [3]float32{0.25, -0.35, 0.3}
	gb := [3]float32{0.01, -0.012, 0.006}
	sample := func() {
		n := func(s float64) float32 { return float32(r.NormFloat64() * s) }
		acc := [3]float32{off[0] + n(0.04), gravity*0.99 + off[1] + n(0.04), off[2] + n(0.04)}
		gyro := [3]float32{gb[0] + n(0.005), gb[1] + n(0.005), gb[2] + n(0.005)}
		imuFrame(a, acc, gyro, 1.0/imuRate)
	}
	for range 3 * imuRate {
		sample()
	}
	d0 := m.disp
	dmax := 0.0
	for i := range 120 * imuRate {
		sample()
		if i%3 == 2 {
			dmax = max(dmax, math.Abs(float64(m.disp[0]-d0[0])), math.Abs(float64(m.disp[1]-d0[1])))
		}
	}
	if dmax > 0.05 {
		t.Errorf("noisy still badge: the badge-frame displacement wandered %.1f cm in 120 s, want ≤ 5", dmax*100)
	}
}

// Every real movement registers, quick flicks too (owner: "in Pong
// a quick flick of 0.2–0.4 s is a real move"): up strokes of 3, 5 and 10 cm
// lasting 0.2, 0.3 and 0.4 s move the badge-frame displacement 1:1
// (±15 %).
func TestQuickStrokesRegister(t *testing.T) {
	up := [3]float64{0, 0, 1}
	for _, T := range []float64{0.2, 0.3, 0.4} {
		for _, d := range []float64{0.03, 0.05, 0.10} {
			a, tm := motionApp()
			before := a.motion.disp[1]
			drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke(up, d, *tm+0.2, T), nil)
			got := float64(a.motion.disp[1] - before)
			if got < d*0.85 || got > d*1.15 {
				t.Errorf("%.0f cm in %.1f s: displacement %.1f cm, want %.0f (1:1)", d*100, T, got*100, d*100)
			}
		}
	}
}

// The display follows a hand move as it happens (owner, TG-5 #2: "not
// responding (jumps around)"): moving the badge up 1 cm in 0.6 s, the
// badge-frame displacement starts within 0.25 s, never jumps more than
// 3.2 mm in a frame (25 px on the glass at 1:1), and ends 1:1. The
// false-start gate (an episode counted only after 1.5 mm and 0.45 s) held it
// still for half a second and then jumped it 10 mm at once.
func TestDisplayFollowsAHandMoveSmoothly(t *testing.T) {
	a, tm := motionApp()
	m := a.motion
	const start, frameS = 0.1, 3.0 / imuRate
	y0, prev := m.disp[1], m.disp[1]
	firstMove, frames, maxStep := -1.0, 0, 0.0
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke([3]float64{0, 0, 1}, 0.01, *tm+start, 0.6), func(i int) {
		if i%3 != 2 {
			return
		}
		frames++
		maxStep = max(maxStep, math.Abs(float64(m.disp[1]-prev)))
		prev = m.disp[1]
		if firstMove < 0 && math.Abs(float64(m.disp[1]-y0)) > 2.5e-6 { // a window unit at 1:1
			firstMove = float64(frames)*frameS - start
		}
	})
	if firstMove < 0 || firstMove > 0.25 {
		t.Errorf("the displacement first moved %.2f s into the move, want within 0.25 s", firstMove)
	}
	if maxStep > 0.0032 {
		t.Errorf("the displacement jumped %.1f mm in one frame, want ≤ 3.2 (a smooth follow)", maxStep*1000)
	}
	if got := float64(m.disp[1] - y0); got < 0.0085 || got > 0.0115 {
		t.Errorf("the displacement moved %.2f cm for 1 cm, want 1 (1:1)", got*100)
	}
}

// A short stroke that ends in a pause holds where it went (owner, TG-6 3b:
// the hybrid): 5 cm up in 0.5 s, then 10 s at rest, and the paddle's
// displacement stays at 5 cm. A long continuous motion instead goes back to
// where it began once the badge rests (it can't be integrated with this
// sensor: see TestOwnersSquaresShowInTheirPlanes).
func TestShortStrokeHoldsLongMotionRecentres(t *testing.T) {
	up := [3]float64{0, 0, 1}
	a, tm := motionApp()
	d0 := a.motion.disp[1]
	drive(a, tm, imuRate, func(float64) float64 { return 0 }, stroke(up, 0.05, *tm+0.2, 0.5), nil)
	end := a.motion.disp[1] - d0
	drive(a, tm, 10*imuRate, func(float64) float64 { return 0 }, noAcc, nil)
	if held := a.motion.disp[1] - d0; math.Abs(float64(held-end)) > 0.005 || math.Abs(float64(end)-0.05) > 0.0075 {
		t.Errorf("5 cm stroke, then 10 s at rest: %.1f cm at its end, %.1f cm after, want 5 and held", end*100, held*100)
	}

	// 6 s of 1 Hz up and down (4 cm, back to the start each period), read
	// with 0.05 m/s² of error, as a hand motion's tilt errors give: a plain
	// integration ends about 0.9 m away
	a, tm = motionApp()
	d0 = a.motion.disp[1]
	t0 := *tm
	shake := func(t float64) [3]float64 {
		if t < t0 || t > t0+6 {
			return [3]float64{}
		}
		w := 2 * math.Pi
		return [3]float64{0, 0, 0.04*w*w*math.Cos(w*(t-t0)) + 0.05}
	}
	drive(a, tm, 6*imuRate, func(float64) float64 { return 0 }, shake, nil)
	drive(a, tm, 8*imuRate, func(float64) float64 { return 0 }, noAcc, nil)
	if back := a.motion.disp[1] - d0; math.Abs(float64(back)) > 0.01 {
		t.Errorf("6 s of shaking, then 8 s at rest: %.1f cm from where it began, want back within 1 cm", back*100)
	}
}

// stillHand is a real still hand's tremor: a quiet stretch of the TG-6
// pickup capture (in a hand: it turns 0.029 rad/s RMS), its mean removed.
func stillHand(t *testing.T) (acc, gyr [][3]float32, rate float64) {
	s, rate := loadCapture(t, "testdata/tg6-pickup.log")
	w := s[3050:3510]
	var am, gm [3]float32
	for _, c := range w {
		for i := range 3 {
			am[i] += c.acc[i] / float32(len(w))
			gm[i] += c.gyr[i] / float32(len(w))
		}
	}
	for _, c := range w {
		var a, g [3]float32
		for i := range 3 {
			a[i], g[i] = c.acc[i]-am[i], c.gyr[i]-gm[i]
		}
		acc, gyr = append(acc, a), append(gyr, g)
	}
	return acc, gyr, rate
}

// A short stroke out of a still hand doesn't go back to where it began:
// held 1.3 s (the A3 hold showing), moved up 5 cm in 0.6 s, then held
// steady, the shown position stays above 4 cm (80 % of the stroke) for 3 s.
// A motion is timed from its onset: timed from the hold before it, the
// stroke counted as a long motion, whose position goes back to where it
// began. (In a trembling hand the strapdown's own error is larger, ±2 cm:
// the Kalman cutover's targets.)
func TestShortStrokeOutOfAHoldDoesNotGoBack(t *testing.T) {
	ta, tg, rate := stillHand(t)
	a, tm := motionApp()
	m := a.motion
	dt := 1 / rate
	const d, T, t1 = 0.05, 0.6, 1.3 // the stroke starts after 1.3 s of still hand
	push := stroke([3]float64{0, 0, 1}, d, *tm+t1, T)
	x, y, z := badgeAxes(0) // upright: the badge's up is the world's
	z0 := m.pos[2]
	held := false
	for i := 0; float64(i)*dt < t1+T+3; i++ {
		w := push(*tm + float64(i)*dt)
		w[2] += gravity
		el := float64(i) * dt
		var na, ng [3]float32
		if el < t1+T { // the hand trembles until the stroke ends, then is steady
			n := i % len(ta)
			na, ng = ta[n], tg[n]
		}
		imuFrame(a, [3]float32{float32(dot(x, w)) + na[0], float32(dot(y, w)) + na[1], float32(dot(z, w)) + na[2]}, ng, float32(dt))
		if el < t1 && el+dt >= t1 {
			held = m.holdFor > 0
		}
		if el >= t1+T {
			if up := float64(m.pos[2]-z0) * 100; up < 4 {
				t.Fatalf("%.2f s after the stroke ended: shown %.2f cm up, went back below 4 cm", el-t1-T, up)
			}
		}
	}
	if !held {
		t.Fatal("the stroke didn't start out of a hold: the test doesn't test it")
	}
}
