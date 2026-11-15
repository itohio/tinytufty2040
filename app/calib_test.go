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
	ps     calib.Pose // the pose a done run gives (the orientation calibration)
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

// A completed still measurement sets the gyro's bias in the motion state
// (owner: "initially gyro is calibrated"; it seeds the motion state's own
// learning). A on the page offers the calibrations with the highlight on
// the orientations: up and A pick "This pose", which starts a new
// single-pose calibration at once, with no edit mode.
func TestCalibrationSetsTheGyroBiasAndARecalibrates(t *testing.T) {
	a, frame := framedApp()
	// a bias unlike the gyro framedApp feeds, which the motion state would
	// not learn on its own
	res := calResult{gyroBias: [3]float32{0.03, -0.02, 0.01}}
	c := &scriptedCal{ph: calDone, res: res}
	a.calib.c = c
	frame()
	if a.calib.stage != calShowDone {
		t.Fatalf("stage %d, want done", a.calib.stage)
	}
	if a.motion.bias != res.gyroBias || !a.calib.gyroSet {
		t.Fatalf("the motion state's gyro bias %v after the calibration's %v (set %v), want it seeded", a.motion.bias, res.gyroBias, a.calib.gyroSet)
	}
	a.Press(BtnA, false) // the page's focus is the view itself
	frame()
	if a.overlay.Top() == nil || a.calDialog.Choice() != 1 {
		t.Fatalf("A on the Calibration page: the choice open %v, highlight %d (want open, on the orientations)", a.overlay.Top() != nil, a.calDialog.Choice())
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

// faceUp is each orientation's gravity reaction, unit, in badge axes.
var faceUnit = [6][3]float32{{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}}

// facePose is a still pose in orientation f of a sensor with offset off,
// scale sc and gyro bias gb (m/s², ratio, rad/s), its noise well under the
// stillness thresholds.
func facePose(f calFace, off, sc, gb [3]float32) calib.Pose {
	var p calib.Pose
	for i := range 3 {
		p.Accel[i] = sc[i]*faceUnit[f][i]*gravity + off[i]
		p.Gyro[i] = gb[i]
		p.AccelNoise[i], p.GyroNoise[i] = 0.01, 0.0005
	}
	return p
}

var unitScale = [3]float32{1, 1, 1}

// orientApp opens the page's choice and starts the orientation calibration
// (the highlight's default), the badge lying flat, screen up.
func orientApp(t *testing.T) (*App, *scriptedCal, func(time.Duration)) {
	a, frame := framedApp()
	c := &scriptedCal{ph: calWaiting}
	c.mean = [6]float32{0, 0, gravity}
	a.calib.c = c
	frame()
	a.Press(BtnA, false)
	a.Press(BtnA, false)
	frame()
	if !a.calib.guiding() || a.calib.g.sel != faceUp {
		t.Fatalf("after picking the orientations: stage %d, highlight %d (want running, on screen up)", a.calib.g.stage, a.calib.g.sel)
	}
	wait := func(d time.Duration) {
		for e := time.Duration(0); e < d; e += 33 * time.Millisecond {
			frame()
		}
	}
	return a, c, wait
}

// lieIn lays the badge in orientation f (the strips' means), still.
func lieIn(c *scriptedCal, f calFace) {
	for i := range 3 {
		c.mean[i] = faceUnit[f][i] * gravity
	}
}

// choose moves the highlight to f with down.
func choose(t *testing.T, a *App, f calFace) {
	t.Helper()
	for range 6 {
		if a.calib.g.sel == f {
			return
		}
		a.Press(BtnDown, false)
	}
	t.Fatalf("down never reached orientation %d", f)
}

// measure lays the badge in f, lets a calibrator run finish there with the
// sensor's pose and waits past the held messages.
func measure(c *scriptedCal, wait func(time.Duration), f calFace, off, sc, gb [3]float32) {
	lieIn(c, f)
	wait(100 * time.Millisecond)
	c.ps, c.ph = facePose(f, off, sc, gb), calDone
	wait(40 * time.Millisecond)
	wait(1600 * time.Millisecond) // past "Done: ..." and the other held messages (gHoldFor)
}

// up/down pick the orientation; only the one picked is measured: placed
// otherwise the page asks for it, a run that ends in another orientation
// isn't counted, and once one is done the highlight moves to the next one
// not done.
func TestOrientationsUpDownPickAndOnlyThePickedIsTaken(t *testing.T) {
	a, c, wait := orientApp(t)
	g := &a.calib.g
	lieIn(c, faceUpright)
	wait(100 * time.Millisecond)
	if s, _ := a.calib.line(); s != "Lay it flat, screen up" {
		t.Fatalf("screen up picked, the badge upright: %q", s)
	}
	a.Press(BtnDown, false)
	if g.sel != faceDown {
		t.Fatalf("down: highlight %d, want screen down", g.sel)
	}
	a.Press(BtnUp, false)
	a.Press(BtnUp, false)
	if g.sel != faceOpposite {
		t.Fatalf("up twice from screen down: highlight %d, want the opposite edge (wrapping)", g.sel)
	}
	choose(t, a, faceDown)
	measure(c, wait, faceUp, [3]float32{}, unitScale, [3]float32{}) // it lies screen up instead
	if g.done != [6]bool{} {
		t.Fatalf("a run in screen up counted for screen down: %v", g.done)
	}
	measure(c, wait, faceDown, [3]float32{}, unitScale, [3]float32{})
	if !g.done[faceDown] || g.sel != faceUpright {
		t.Fatalf("screen down measured: done %v, highlight %d (want it done, the highlight on upright)", g.done, g.sel)
	}
}

// Three orientations on different axes fit the offsets; two axes aren't
// enough; all six (each axis both ways) fit the scales too.
func TestOrientationsFitOffsetsOnThreeAxesAndScalesOnSix(t *testing.T) {
	off := [3]float32{0.12, -0.25, 0.31}
	sc := [3]float32{1.02, 0.97, 1.01}
	gb := [3]float32{0.004, -0.002, 0.001}
	a, c, wait := orientApp(t)
	g := &a.calib.g
	for _, f := range []calFace{faceUp, faceDown, faceUpright} { // z both ways and y: two axes
		choose(t, a, f)
		measure(c, wait, f, off, unitScale, gb)
	}
	if g.fitSt != fitNone {
		t.Fatalf("two axes: fit state %d, want none", g.fitSt)
	}
	choose(t, a, faceArrows)
	measure(c, wait, faceArrows, off, unitScale, gb)
	if g.fitSt != fitShown || g.fit.Model != calib.OffsetOnly {
		t.Fatalf("three axes: fit state %d model %v, want an offsets fit shown", g.fitSt, g.fit.Model)
	}
	for i := range 3 {
		if d := g.fit.Offset[i] - off[i]; d > 0.002 || d < -0.002 {
			t.Errorf("axis %d: offset %.4f, want %.4f", i, g.fit.Offset[i], off[i])
		}
	}
	a2, c2, wait2 := orientApp(t)
	g2 := &a2.calib.g
	for f := calFace(0); f < 6; f++ {
		choose(t, a2, f)
		measure(c2, wait2, f, off, sc, gb)
	}
	if g2.fitSt != fitShown || g2.fit.Model != calib.OffsetScale {
		t.Fatalf("all six: fit state %d model %v, want offsets and scales", g2.fitSt, g2.fit.Model)
	}
	for i := range 3 {
		if d := g2.fit.Offset[i] - off[i]; d > 0.005 || d < -0.005 {
			t.Errorf("six, axis %d: offset %.4f, want %.4f", i, g2.fit.Offset[i], off[i])
		}
		if d := g2.fit.Scale[i] - sc[i]; d > 0.002 || d < -0.002 {
			t.Errorf("six, axis %d: scale %.4f, want %.4f", i, g2.fit.Scale[i], sc[i])
		}
	}
	if s, _ := a2.calib.line(); s != gAllText {
		t.Fatalf("all six: %q", s)
	}
}

// The table: for every orientation done, gravity as measured minus 1 g
// (raw: an axis's scale and offset show; fitted: near 0), and the spread
// across orientations (owner: "how much the gravity pointing down actually
// changes when i change the orientation").
func TestOrientationTableShowsGravitysDeviationRawAndFitted(t *testing.T) {
	off := [3]float32{0, 0, 0.1}
	sc := [3]float32{1.02, 0.97, 1.0}
	a, c, wait := orientApp(t)
	g := &a.calib.g
	for f := calFace(0); f < 6; f++ {
		choose(t, a, f)
		measure(c, wait, f, off, sc, [3]float32{})
	}
	// raw: x reads 1.02 g both ways, y 0.97 g, z 1 g ± the 0.1 m/s² offset
	want := [6]float32{0.1, -0.1, -0.03 * gravity, -0.03 * gravity, 0.02 * gravity, 0.02 * gravity}
	for f := calFace(0); f < 6; f++ {
		if d := g.gDev(f, false) - want[f]; d > 0.002 || d < -0.002 {
			t.Errorf("%s raw: %.4f m/s² from 1 g, want %.4f", calFaceShort[f], g.gDev(f, false), want[f])
		}
		if fv := g.gDev(f, true); fv > 0.005 || fv < -0.005 {
			t.Errorf("%s fitted: %.4f m/s² from 1 g, want about 0", calFaceShort[f], fv)
		}
	}
	// the spread the page shows: max − min of the raw deviations, 50 mg
	lo, hi := float32(0), float32(0)
	for f := calFace(0); f < 6; f++ {
		lo, hi = minmax(int(f), g.gDev(f, false), lo, hi)
	}
	if m := mg(hi - lo); m != 50 {
		t.Fatalf("raw spread %d mg, want 50 (1.02 g vs 0.97 g)", m)
	}
}

// A pose that wasn't still is refused (the solver's stillness thresholds):
// it isn't counted and the same orientation stays picked.
func TestOrientationsRefuseAMovingPose(t *testing.T) {
	a, c, wait := orientApp(t)
	p := facePose(faceUp, [3]float32{}, unitScale, [3]float32{})
	p.AccelNoise = [3]float32{0.5, 0.5, 0.5}
	c.ps, c.ph = p, calDone
	wait(100 * time.Millisecond)
	if g := &a.calib.g; g.stage != gNotStill || g.done[faceUp] || g.sel != faceUp {
		t.Fatalf("a moving pose: stage %d, done %v, highlight %d (want not still, nothing done, still screen up)", g.stage, g.done, g.sel)
	}
}

// A on a fit uses it: from then the accelerometer samples the motion state
// gets are corrected (across power-off only if saved); B instead drops it
// and they stay raw, and the page stays.
func TestOrientationFitUseCorrectsAndDiscardDoesNot(t *testing.T) {
	off := [3]float32{0.2, -0.1, 0.3}
	raw := [3]float32{off[0], gravity + off[1], off[2]} // upright, still
	for _, use := range []bool{true, false} {
		a, c, wait := orientApp(t)
		for _, f := range []calFace{faceUp, faceUpright, faceArrows} {
			choose(t, a, f)
			measure(c, wait, f, off, unitScale, [3]float32{})
		}
		if a.calib.g.fitSt != fitShown {
			t.Fatalf("fit state %d, want a fit shown", a.calib.g.fitSt)
		}
		if use {
			a.Press(BtnA, false)
		} else {
			a.Press(BtnB, false)
		}
		wait(100 * time.Millisecond)
		acc := a.calib.correct(raw)
		corrected := acc[1] > gravity-0.01 && acc[1] < gravity+0.01 && acc[0] < 0.01 && acc[0] > -0.01
		if use && !corrected {
			t.Fatalf("A on the fit: the sample %v reaches the motion state as %v, want the offsets out", raw, acc)
		}
		if !use && (corrected || acc != raw) {
			t.Fatalf("B on the fit: the sample became %v, want it raw %v", acc, raw)
		}
		if !use && a.page().name != "Calib" {
			t.Fatalf("B on the fit left the page for %s", a.page().name)
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

// The page's frames allocate nothing in any stage, the orientation
// calibration's table and fit included (owner: no allocations after
// setup).
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
	a, c, wait := orientApp(t)
	off, sc := [3]float32{0.1, -0.2, 0.3}, [3]float32{1.02, 0.97, 1.01}
	for f := calFace(0); f < 5; f++ {
		choose(t, a, f)
		measure(c, wait, f, off, sc, [3]float32{})
	}
	tick := func() { wait(33 * time.Millisecond) }
	lieIn(c, faceOpposite)
	for _, ph := range []calPhase{calWaiting, calMeasuring, calMoved} {
		c.ph, c.prog = ph, 0.6
		if n := testing.AllocsPerRun(30, tick); n != 0 {
			t.Errorf("orientations, phase %d (five done, a fit shown): %.0f allocations per frame", ph, n)
		}
	}
}

// After a pause longer than the pages' idle timeout, the Calibration page's
// first key still acts: up/down move the orientation highlight and A
// opens the choice. The page is one view with no focus to show, so the
// idle timeout clearing focus (as on list pages, where the first key only
// brings the focus back) made a key vanish (two dropped presses on the
// badge, each after a still pause). The containers time idleness by the
// wall clock: the test shortens the page's timeout and sleeps past it.
func TestCalibrationKeysActAfterAnIdlePause(t *testing.T) {
	idle := func(a *App, frame func()) {
		a.page().root.Timeout = 20 * time.Millisecond
		time.Sleep(50 * time.Millisecond)
		frame()
	}
	a, _, wait := orientApp(t)
	idle(a, func() { wait(33 * time.Millisecond) })
	a.Press(BtnDown, false)
	if a.calib.g.sel != faceDown {
		t.Fatalf("down after an idle pause: highlight %d, want screen down", a.calib.g.sel)
	}
	b, frame := framedApp()
	frame()
	idle(b, frame)
	b.Press(BtnA, false)
	if b.overlay.Top() == nil {
		t.Fatal("A after an idle pause on the Calibration page: the choice didn't open")
	}
}
