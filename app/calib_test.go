package app

import (
	"testing"
	"time"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
)

// scriptedCal is a calibrator whose phase the test sets.
type scriptedCal struct {
	ph     calPhase
	prog   float32
	res    calResult
	n      int // samples fed
	resets int
	ps     calib.Pose // the pose a done run gives (the guided calibration)
	mean   [6]float32 // the axes' windowed means (stats)
	spread float32    // their spread (0: 0.01, still)
}

func (c *scriptedCal) add(acc, gyro [3]float32, dt float32) { c.n++ }
func (c *scriptedCal) phase() calPhase                      { return c.ph }
func (c *scriptedCal) progress() float32                    { return c.prog }
func (c *scriptedCal) result() calResult                    { return c.res }
func (c *scriptedCal) reset()                               { c.resets++; c.ph, c.prog = calWaiting, 0 }
func (c *scriptedCal) stats(i int) (mean, spread float32) {
	if c.spread != 0 {
		return c.mean[i], c.spread
	}
	return c.mean[i], 0.01
}
func (c *scriptedCal) thresholds() (acc, gyro float32) { return 0.05, 0.01 }
func (c *scriptedCal) pose() (calib.Pose, bool)        { return c.ps, c.ph == calDone }

// The page walks the brief's five states (TG-3): the instruction with the
// ring at 0 %, waiting, the ring filling from the calibrator's progress,
// "moved" held long enough to read with the ring back at 0, and
// "calibrated", which stays; the result is handed over exactly once.
func TestCalibrationShowsEveryStage(t *testing.T) {
	c := &scriptedCal{}
	k := newCalibration(c)
	t0 := time.Unix(0, 0)
	at := func(ms int) time.Time { return t0.Add(time.Duration(ms) * time.Millisecond) }
	want := func(when string, stage calStage, pct float32) {
		t.Helper()
		if k.stage != stage || k.pct != pct {
			t.Fatalf("%s: stage %d at %.0f %%, want stage %d at %.0f %%", when, k.stage, k.pct, stage, pct)
		}
	}
	k.update(at(0))
	want("start", calStartup, 0)
	k.update(at(1900))
	want("1.9 s, still waiting", calStartup, 0)
	k.update(at(2100))
	want("2.1 s, still waiting", calShowWaiting, 0)
	c.ph, c.prog = calMeasuring, 0.42
	k.update(at(3000))
	want("measuring", calShowMeasuring, 42)
	c.ph = calMoved
	k.update(at(3100))
	want("moved", calShowMoved, 0)
	c.ph, c.prog = calMeasuring, 0.05 // it restarted at once
	k.update(at(4000))
	want("0.9 s after the move", calShowMoved, 0)
	k.update(at(4400))
	want("1.3 s after the move", calShowMeasuring, 5)
	c.ph = calDone
	if !k.update(at(5000)) {
		t.Fatal("done, but the result wasn't handed over")
	}
	want("done", calShowDone, 100)
	if k.update(at(60000)) {
		t.Fatal("the same result handed over twice")
	}
	want("a minute later", calShowDone, 100)
}

// With an IMU the app starts on the Calibration page, yet nothing waits
// for it (owner: "demo loading never waits"): C goes on to the next page at
// once, and every sample reaches both the calibrator and the motion state,
// which runs uncalibrated meanwhile.
func TestCalibrationPageDoesNotHoldUpTheDemos(t *testing.T) {
	a, frame := framedApp()
	c := &scriptedCal{}
	a.calib.c = c
	if a.page().name != "Calib" {
		t.Fatalf("starts on %q, want the Calibration page", a.page().name)
	}
	frame()
	if c.n == 0 || a.motion.samples == 0 {
		t.Fatalf("the calibrator got %d samples, the motion state %d: want both fed", c.n, a.motion.samples)
	}
	a.Press(BtnC, false)
	frame()
	if a.page().name == "Calib" {
		t.Fatal("C didn't leave the Calibration page")
	}
	n := c.n
	frame()
	if c.n == n {
		t.Fatal("calibration stopped when its page was left")
	}
}

// Without an IMU there is no Calibration page: the app starts on Demos.
func TestNoIMUNoCalibrationPage(t *testing.T) {
	a := testApp()
	if i := a.pageIndex("Calib"); i >= 0 || a.calib != nil {
		t.Fatalf("no IMU, yet a Calibration page at %d", i)
	}
	if a.page().name != "Demos" {
		t.Fatalf("no IMU: starts on %q, want Demos", a.page().name)
	}
}

// A completed calibration is shown, not applied: the shared motion state
// keeps its own gyro bias (nothing goes into it until strapdown r11,
// which seeding needs). A on the page offers the calibrations with the
// highlight on 3 faces: up and A pick "This pose", which
// starts a new single-pose calibration at once, with no edit mode.
func TestCalibrationIsShownNotAppliedAndARecalibrates(t *testing.T) {
	a, frame := framedApp()
	// a bias unlike the gyro framedApp feeds, which the motion state learns
	// at rest on its own
	res := calResult{gyroBias: [3]float32{0.3, 0.3, 0.3}}
	c := &scriptedCal{ph: calDone, res: res}
	a.calib.c = c
	frame()
	if a.calib.stage != calShowDone {
		t.Fatalf("stage %d, want done", a.calib.stage)
	}
	if a.motion.bias == res.gyroBias {
		t.Fatalf("the calibration's bias %v went into the motion state", res.gyroBias)
	}
	a.Press(BtnA, false) // the page's focus is the view itself
	frame()
	if a.overlay.Top() == nil || a.calDialog.Choice() != 1 {
		t.Fatalf("A on the Calibration page: the choice open %v, highlight %d (want open, on 3 faces)", a.overlay.Top() != nil, a.calDialog.Choice())
	}
	for round := 1; round <= 2; round++ {
		if round == 2 {
			c.ph = calDone
			frame()
			a.Press(BtnA, false)
		}
		a.Press(BtnUp, false)
		a.Press(BtnA, false)
		frame()
		if c.resets != round || a.calib.stage == calShowDone || a.overlay.Top() != nil {
			t.Fatalf("A, up, A (round %d): %d resets, stage %d, choice open %v (want a new calibration)", round, c.resets, a.calib.stage, a.overlay.Top() != nil)
		}
		if a.editing {
			t.Fatal("A on the Calibration page left an edit mode (footer \"EDIT ... B cancel\")")
		}
	}
}

// facePose is a still pose on face f of a sensor with offset off and gyro
// bias gb (m/s², rad/s), its noise well under the stillness thresholds.
func facePose(f calFace, off, gb [3]float32) calib.Pose {
	up := [6][3]float32{{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}}[f]
	var p calib.Pose
	for i := range 3 {
		p.Accel[i] = up[i]*gravity + off[i]
		p.Gyro[i] = gb[i]
		p.AccelNoise[i], p.GyroNoise[i] = 0.01, 0.0005
	}
	return p
}

// guidedApp opens the page's choice and starts a guided calibration
// (pick: 0 up from 3 faces is This pose, 1 three faces, 2 six faces).
func guidedApp(t *testing.T, faces int) (*App, *scriptedCal, func(time.Duration)) {
	a, frame := framedApp()
	c := &scriptedCal{ph: calWaiting}
	a.calib.c = c
	frame()
	a.Press(BtnA, false)
	if faces == 6 {
		a.Press(BtnDown, false)
	}
	a.Press(BtnA, false)
	frame()
	if a.calib.g.stage != gAsk {
		t.Fatalf("after picking %d faces: guided stage %d, want asking", faces, a.calib.g.stage)
	}
	wait := func(d time.Duration) {
		for e := time.Duration(0); e < d; e += 33 * time.Millisecond {
			frame()
		}
	}
	return a, c, wait
}

// lay finishes one run of the calibrator on face f (what the badge lying
// still on f gives) and lets the page take it.
func lay(c *scriptedCal, wait func(time.Duration), f calFace, off, gb [3]float32) {
	c.ps, c.ph = facePose(f, off, gb), calDone
	wait(40 * time.Millisecond)
	wait(1600 * time.Millisecond) // past "Face k done" and the other held messages (gHoldFor)
}

// Three faces (the default) fit the accelerometer's offsets and the
// gyro's bias; the page says which face it wants next and refuses a face
// already done, a done face drops off the list, and the edge opposite the
// UP/DOWN buttons counts for that edge (any order; the face comes from
// gravity).
func TestGuidedCalibrationThreeFaces(t *testing.T) {
	a, c, wait := guidedApp(t, 3)
	off := [3]float32{0.12, -0.25, 0.31} // 12, -25, 32 mg
	gb := [3]float32{0.004, -0.002, 0.001}
	g := &a.calib.g
	if s, _ := a.calib.line(); s != "Face 1 of 3: lay it flat, screen up" {
		t.Fatalf("first ask %q", s)
	}
	lay(c, wait, faceUp, off, gb)
	lay(c, wait, faceUp, off, gb) // the same face again
	if g.n != 1 {
		t.Fatalf("a face already done counted: %d faces", g.n)
	}
	if s, _ := a.calib.line(); s != "Face 2 of 3: lay it on the up/down-button edge" {
		t.Fatalf("after one face: %q", s)
	}
	lay(c, wait, faceOpposite, off, gb) // the other edge: it counts for the side
	lay(c, wait, faceArrows, off, gb)   // and then that side is done
	if g.n != 2 {
		t.Fatalf("after the opposite edge and the up/down edge: %d faces, want 2", g.n)
	}
	c.ps, c.ph = facePose(faceUpright, off, gb), calDone
	wait(100 * time.Millisecond)
	if g.stage != gResult {
		t.Fatalf("three faces: stage %d, want the result", g.stage)
	}
	for i := range 3 {
		if d := g.fit.Offset[i] - off[i]; d > 0.002 || d < -0.002 {
			t.Errorf("axis %d: offset %.4f, want %.4f", i, g.fit.Offset[i], off[i])
		}
		if d := g.fit.GyroBias[i] - gb[i]; d > 1e-5 || d < -1e-5 {
			t.Errorf("axis %d: gyro bias %.5f, want %.5f", i, g.fit.GyroBias[i], gb[i])
		}
	}
	if s, _ := a.calib.line(); s != "Calibrated on 3 faces (offsets only)" {
		t.Fatalf("result line %q", s)
	}
}

// A pose that wasn't still is refused (the solver's stillness thresholds):
// it isn't counted and the same face is asked for again.
func TestGuidedCalibrationRefusesAMovingPose(t *testing.T) {
	a, c, wait := guidedApp(t, 3)
	p := facePose(faceUp, [3]float32{}, [3]float32{})
	p.AccelNoise = [3]float32{0.5, 0.5, 0.5}
	c.ps, c.ph = p, calDone
	wait(100 * time.Millisecond)
	if g := &a.calib.g; g.stage != gNotStill || g.n != 0 {
		t.Fatalf("a moving pose: stage %d, %d faces (want not still, 0)", g.stage, g.n)
	}
	wait(1600 * time.Millisecond)
	if s, _ := a.calib.line(); s != "Face 1 of 3: lay it flat, screen up" {
		t.Fatalf("after the refusal: %q", s)
	}
}

// A on the result uses the fit: from then the samples the motion state gets
// are corrected (until power-off); B instead drops it and they stay raw.
func TestGuidedCalibrationUseCorrectsAndDiscardDoesNot(t *testing.T) {
	off := [3]float32{0.2, -0.1, 0.3}
	gb := [3]float32{0.01, 0, 0}
	raw := [3]float32{off[0], gravity + off[1], off[2]} // upright, still
	for _, use := range []bool{true, false} {
		a, c, wait := guidedApp(t, 3)
		for _, f := range []calFace{faceUp, faceArrows, faceUpright} {
			lay(c, wait, f, off, gb)
		}
		if a.calib.g.stage != gResult {
			t.Fatalf("stage %d, want the result", a.calib.g.stage)
		}
		if use {
			a.Press(BtnA, false)
		} else {
			a.Press(BtnB, false)
		}
		wait(100 * time.Millisecond)
		acc, gyro := a.calib.correct(raw, gb)
		corrected := acc[1] > gravity-0.01 && acc[1] < gravity+0.01 && acc[0] < 0.01 && acc[0] > -0.01 && gyro[0] < 1e-4
		if use && !corrected {
			t.Fatalf("A on the result: the sample %v / %v reaches the motion state as %v / %v, want the offsets and bias out", raw, gb, acc, gyro)
		}
		if !use && (corrected || acc != raw) {
			t.Fatalf("B on the result: the sample became %v, want it raw %v", acc, raw)
		}
		if !use && a.page().name != "Calib" {
			t.Fatalf("B on the result left the page for %s", a.page().name)
		}
	}
}

// Set down on another face after calibrating, the strips never read drift
// (owner, the Calibration page: "drift" after "Calibrated" on an edge, read
// as the badge not calibrating every axis). The turn itself reads noisy (a
// wide spread); once it lies still on the new face no axis shows drift,
// though the face changed before the last of the turn settled.
func TestCalibrationStripsDoNotReadATurnAsDrift(t *testing.T) {
	a, frame := framedApp()
	c := &scriptedCal{ph: calDone}
	c.mean = [6]float32{0, gravity, 0, 0.001, 0, 0} // upright
	a.calib.c = c
	for range 30 {
		frame()
	}
	// handled for a second: gravity swings to the up/down-button edge, the
	// spread wide; the face changes half way
	for i := range 30 {
		u := float32(i+1) / 30
		c.mean = [6]float32{-gravity * u, gravity * (1 - u), 0, 0.001, 0, 0}
		c.spread = 0.4
		frame()
	}
	c.spread = 0 // laid on its edge, still
	for f := range 300 {
		frame()
		for i := range 3 {
			if st := a.calib.state[i]; st == 1 {
				t.Fatalf("lying still on its edge, frame %d: axis %s reads drift", f, calAxisNames[i])
			}
		}
	}
}

// The page's frames allocate nothing in any stage (owner: no allocations
// after setup).
func TestCalibrationFramesDoNotAllocate(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	c := &scriptedCal{ph: calMeasuring, prog: 0.6}
	a.calib.c = c
	frame()
	for _, ph := range []calPhase{calWaiting, calMeasuring, calMoved, calDone} {
		c.ph = ph
		if n := testing.AllocsPerRun(30, frame); n != 0 {
			t.Errorf("phase %d: %.0f allocations per frame on the Calibration page", ph, n)
		}
	}
}
