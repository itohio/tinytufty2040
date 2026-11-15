package app

import (
	"math"
	"math/rand/v2"
	"testing"
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

// The Motion page shows the world path from the badge (owner: "rotate the
// badge and see the path also rotated so i can observe it from my latest
// integrated position point of view"): after a move 5 cm right, where it
// started is left of the view's centre (the badge, now); turned 90°
// clockwise in place, the same start is below the centre.
func TestPathViewTurnsWithTheBadge(t *testing.T) {
	a, tm := motionApp()
	m := a.motion
	md := newMotionDemo(m)
	tr := &md.track // the camera sits at its position
	v := md.page
	x, _, _ := badgeAxes(0)
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke(x, 0.05, *tm, 0.7), func(int) { tr.record() })
	start := tr.pts[0]
	look := func() (int32, int32) {
		v.setCamera(v.back())
		v.ox, v.oy, v.f = ScreenW/2, ScreenH/2, int32(pathFocal*ScreenH)
		sx, sy, ok := v.project(start)
		if !ok {
			t.Fatal("the path's start is behind the camera")
		}
		return sx - int32(v.ox), sy - int32(v.oy)
	}
	if dx, dy := look(); dx > -20 || dy < -5 || dy > 5 {
		t.Fatalf("upright, 5 cm right of the start: the start drawn at (%+d, %+d) from the centre, want well left, level", dx, dy)
	}
	t1 := *tm
	drive(a, tm, 2*imuRate, func(t float64) float64 { return math.Pi / 2 * smooth(t-t1) }, noAcc, nil)
	if dx, dy := look(); dy < 20 || dx < -5 || dx > 5 {
		t.Fatalf("turned 90° clockwise: the start drawn at (%+d, %+d) from the centre, want well below it", dx, dy)
	}
}

// An accelerometer that reads a little short or long at rest still lets
// the filter settle (the badge reads 9.7 m/s² standing still, 0.06–0.16 m/s²
// short; strapdown r8 measured MPU6050 offsets of 0.3–0.5 m/s²).
// An attitude can only turn the reading, never change its size, so a settle
// test on the size never passed there: the path was never started, and the
// paddle, the boids' window and the Motion page never moved.
func TestSettlesDespiteAnAccelerometerScaleError(t *testing.T) {
	for _, scale := range []float32{0.985, 1.015, 0.96, 1.04} {
		a := testApp()
		a.motion = newMotion()
		for range 3 * imuRate {
			imuFrame(a, [3]float32{0, gravity * scale, 0}, [3]float32{}, 1.0/imuRate)
		}
		if !a.motion.att.settled {
			t.Errorf("accelerometer ×%.3f at rest: not settled after 3 s", scale)
		}
	}
}

// A shake sloshes the water (it feels the specific force in the screen's
// frame): shaken side to side at 1.5 Hz, its centre moves a cell or more
// beyond a still badge's.
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

// Every real movement registers, quick flicks too (owner: "in Pong
// a quick flick of 0.2–0.4 s is a real move"): up strokes of 3, 5 and 10 cm
// lasting 0.2, 0.3 and 0.4 s move the badge-frame displacement the paddle
// and the boids' window read 1:1 (±15 %).
func TestQuickStrokesRegister(t *testing.T) {
	up := [3]float64{0, 0, 1}
	for _, T := range []float64{0.2, 0.3, 0.4} {
		for _, d := range []float64{0.03, 0.05, 0.10} {
			a, tm := motionApp()
			before := a.motion.kal.P[2]
			drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke(up, d, *tm+0.2, T), nil)
			got := float64(a.motion.kal.P[2] - before)
			if got < d*0.85 || got > d*1.15 {
				t.Errorf("%.0f cm in %.1f s: displacement %.1f cm, want %.0f (1:1)", d*100, T, got*100, d*100)
			}
		}
	}
}

// A short stroke that ends in a pause holds where it went: 5 cm up in
// 0.5 s, then 10 s at rest, and the Kalman height stays at 5 cm.
func TestShortStrokeHoldsWhereItWent(t *testing.T) {
	up := [3]float64{0, 0, 1}
	a, tm := motionApp()
	d0 := a.motion.kal.P[2]
	drive(a, tm, imuRate, func(float64) float64 { return 0 }, stroke(up, 0.05, *tm+0.2, 0.5), nil)
	end := a.motion.kal.P[2] - d0
	drive(a, tm, 10*imuRate, func(float64) float64 { return 0 }, noAcc, nil)
	if held := a.motion.kal.P[2] - d0; math.Abs(float64(held-end)) > 0.005 || math.Abs(float64(end)-0.05) > 0.0075 {
		t.Errorf("5 cm stroke, then 10 s at rest: %.1f cm at its end, %.1f cm after, want 5 and held", end*100, held*100)
	}
}
