package app

import (
	"testing"
	"time"

	"github.com/itohio/tinygui/examples/framebuffer"
)

// ledHW records the user LED as the app drives it.
type ledHW struct {
	testHW
	pct   int           // the last SetLED
	blink time.Duration // the hardware blink running (0: none)
}

func (h *ledHW) SetLED(p int)               { h.pct = p }
func (h *ledHW) BlinkLED(per time.Duration) { h.blink = per }

// state is what the LED shows now: "fast", "on", "off".
func (h *ledHW) state() string {
	switch {
	case h.blink > 0:
		return "fast"
	case h.pct > 0:
		return "on"
	}
	return "off"
}

// The LED cue on the orientation calibration: 10 Hz while the
// badge lies in the picked orientation but isn't still (or moved
// mid-measurement), 2 Hz while it is measured, on for 1 s when it's done,
// then off; leaving the page gives the LED back to its own setting.
func TestOrientationCalibrationDrivesTheLED(t *testing.T) {
	hw := &ledHW{}
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	now := time.Now()
	a := New(fb, hw, Config{DurationAdd: id, DurationSub: id, IMU: true}, now)
	c := &scriptedCal{ph: calWaiting}
	c.mean = [6]float32{0, gravity, 0} // upright: not the picked screen up
	a.calib.c = c
	frame := func() {
		now = now.Add(33 * time.Millisecond)
		a.SetIMU([3]float32{0, gravity, 0}, [3]float32{}, 0.033)
		a.Update(now)
	}
	wait := func(d time.Duration) {
		for e := time.Duration(0); e < d; e += 33 * time.Millisecond {
			frame()
		}
	}
	frame()
	a.Press(BtnA, false)
	a.Press(BtnA, false) // orientations
	wait(100 * time.Millisecond)
	if s := hw.state(); s != "off" {
		t.Fatalf("not lying in the picked orientation: LED %s, want off", s)
	}
	lieIn(c, faceUp) // in it, not still yet
	wait(100 * time.Millisecond)
	if hw.blink != 100*time.Millisecond {
		t.Fatalf("in the orientation, not still: LED blink %v, want 10 Hz (100 ms)", hw.blink)
	}
	c.ph, c.prog = calMeasuring, 0.3
	wait(100 * time.Millisecond)
	if hw.blink != 0 {
		t.Fatal("measuring: the fast blink still runs")
	}
	// 2 Hz: on for 250 ms, off for 250 ms
	ons, offs, flips, last := 0, 0, 0, ""
	for range 60 { // 2 s
		frame()
		s := hw.state()
		if s == "on" {
			ons++
		} else {
			offs++
		}
		if last != "" && s != last {
			flips++
		}
		last = s
	}
	if flips < 6 || flips > 10 || ons < 20 || offs < 20 {
		t.Fatalf("measuring 2 s: %d flips, %d frames on, %d off (want a 2 Hz blink: ~8 flips, half on)", flips, ons, offs)
	}
	c.ph = calMoved
	wait(100 * time.Millisecond)
	if hw.blink != 100*time.Millisecond {
		t.Fatalf("moved while measuring: LED blink %v, want 10 Hz", hw.blink)
	}
	c.ph = calWaiting
	wait(700 * time.Millisecond)
	c.ps, c.ph = facePose(faceUp, [3]float32{}, unitScale, [3]float32{}), calDone
	wait(66 * time.Millisecond)
	if s := hw.state(); s != "on" {
		t.Fatalf("screen up just done: LED %s, want on (solid)", s)
	}
	wait(700 * time.Millisecond)
	if s := hw.state(); s != "on" {
		t.Fatalf("0.8 s after done: LED %s, want still on", s)
	}
	wait(400 * time.Millisecond)
	if s := hw.state(); s != "off" {
		t.Fatalf("1.2 s after done: LED %s, want off until the next step", s)
	}
	a.ledOn, a.ledPct = true, 40 // the LED's own setting
	a.Press(BtnC, false)         // leave the page
	wait(100 * time.Millisecond)
	if hw.blink != 0 || hw.pct != 40 {
		t.Fatalf("left the page: blink %v, LED %d%%, want its own setting (40 %%)", hw.blink, hw.pct)
	}
}
