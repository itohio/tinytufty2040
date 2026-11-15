package app

import (
	"testing"
	"time"
)

// scriptedCal is a calibrator whose phase the test sets.
type scriptedCal struct {
	ph     calPhase
	prog   float32
	res    calResult
	n      int // samples fed
	resets int
}

func (c *scriptedCal) add(acc, gyro [3]float32, dt float32) { c.n++ }
func (c *scriptedCal) phase() calPhase                      { return c.ph }
func (c *scriptedCal) progress() float32                    { return c.prog }
func (c *scriptedCal) result() calResult                    { return c.res }
func (c *scriptedCal) reset()                               { c.resets++; c.ph, c.prog = calWaiting, 0 }
func (c *scriptedCal) stats(i int) (mean, spread float32)   { return 0, 0.01 }
func (c *scriptedCal) thresholds() (acc, gyro float32)      { return 0.05, 0.01 }

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
// which seeding needs). A on the page starts a new calibration at once, with
// no edit mode: every A recalibrates, and the footer stays on paging.
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
	if c.resets != 1 || a.calib.stage == calShowDone {
		t.Fatalf("A on the Calibration page: %d resets, stage %d (want a new calibration)", c.resets, a.calib.stage)
	}
	if a.editing {
		t.Fatal("A on the Calibration page left an edit mode (footer \"EDIT ... B cancel\")")
	}
	a.Press(BtnA, false)
	frame()
	if c.resets != 2 {
		t.Fatalf("a second A: %d resets, want 2", c.resets)
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
