package app

import (
	"math"
	"testing"
	"time"
)

func TestPongPaddleReturnsBall(t *testing.T) {
	p := newPong()
	p.pad[0] = 20 * pongFix
	p.bx, p.by = (pongLeftX+2)*pongFix, 23*pongFix // just right of the left paddle, at its centre
	p.vx, p.vy = -pongMaxVX, 0                     // fast: crosses more than a cell per tick
	p.level = 0
	for range 3 {
		p.step()
	}
	if p.vx <= 0 {
		t.Fatalf("ball not returned: vx %d bx %d", p.vx, p.bx)
	}
	if p.score != [2]int{} {
		t.Fatalf("score changed on a return: %v", p.score)
	}
}

func TestPongMissScoresAndServesToLoser(t *testing.T) {
	p := newPong()
	p.pad[0] = 0
	p.player, p.target = true, 0 // left paddle held at the top
	p.bx, p.by = (pongLeftX+2)*pongFix, 40*pongFix
	p.vx, p.vy = -pongServeVX, 0
	for range 40 {
		p.step()
		if p.score[1] == 1 {
			break
		}
	}
	if p.score != [2]int{0, 1} {
		t.Fatalf("score %v, want right player 1", p.score)
	}
	if p.bx != pongW/2*pongFix || p.vx >= 0 {
		t.Fatalf("serve: bx %d vx %d, want centre and towards the left (the side that missed)", p.bx, p.vx)
	}
}

func TestPongBallStaysOnCourt(t *testing.T) {
	p := newPong()
	for range 5000 {
		p.step()
		if p.by < 0 || p.by > (pongH-1)*pongFix || p.bx < -pongMaxVX || p.bx >= pongW*pongFix+pongMaxVX {
			t.Fatalf("ball off court: %d,%d", p.bx, p.by)
		}
		for _, top := range p.pad {
			if top < 0 || top > (pongH-pongPadLen)*pongFix {
				t.Fatalf("paddle off court: %d", top)
			}
		}
	}
}

// Paused, UP/DOWN still move your paddle where it is drawn, at once, and
// resuming doesn't make it jump (seen on the badge: the paddle
// stayed put while paused and leapt 6 cells on resume).
func TestPongPausedPaddleMovesWithButtons(t *testing.T) {
	p := newPong()
	p.showFull()
	p.fullPress(BtnA) // pause
	before := p.pad[0]
	p.fullPress(BtnUp)
	p.tick()
	if got, want := p.pad[0], before-pongTap; got != want {
		t.Fatalf("paused, UP: paddle at %.1f cells, want %.1f (a tap's step up, drawn now)", float32(got)/pongFix, float32(want)/pongFix)
	}
	p.fullPress(BtnA) // resume
	moved := p.pad[0]
	p.tick()
	if d := p.pad[0] - moved; d != 0 {
		t.Fatalf("on resume the paddle moved %.1f cells by itself", float32(d)/pongFix)
	}
}

// The paddle's buttons (owner, TG-6 #4): a tap is a small step; held, the
// paddle glides at a set speed instead of jumping 6 cells at every
// auto-repeat. The badge's button sequence: the press, the held edge, after
// 500 ms a long press every 120 ms, the release.
func TestPaddleTapStepsAndHoldGlides(t *testing.T) {
	a := testApp()
	now := time.Now()
	frame := func() { now = now.Add(33 * time.Millisecond); a.Update(now) }
	a.showPage(a.pageIndex("Pong"))
	a.pong.showFull()
	frame()
	if a.full == nil {
		t.Fatal("Pong not full screen")
	}
	p := a.pong
	p.target = 20 * pongFix // mid-court: room both ways (with the IMU, the neutral pitch's centre)

	// a tap: down for 100 ms
	before := p.target
	a.Hold(BtnUp, true)
	a.Press(BtnUp, false)
	for range 3 {
		frame()
	}
	a.Hold(BtnUp, false)
	frame()
	if got := float32(before-p.target) / pongFix; got != 2 {
		t.Fatalf("a tap moved the paddle %.2f cells up, want 2", got)
	}

	// held for 1 s, with the badge's long repeats
	before = p.target
	a.Hold(BtnDown, true)
	a.Press(BtnDown, false)
	start := now
	last, worst := p.target, float32(0)
	repeats := []time.Duration{500 * time.Millisecond, 620 * time.Millisecond, 740 * time.Millisecond}
	for now.Sub(start) < time.Second {
		frame()
		for len(repeats) > 0 && now.Sub(start) >= repeats[0] {
			a.Press(BtnDown, true)
			repeats = repeats[1:]
		}
		worst = max(worst, float32(p.target-last)/pongFix)
		last = p.target
	}
	a.Hold(BtnDown, false)
	frame()
	moved := float32(p.target-before) / pongFix
	if worst > 1.5 {
		t.Errorf("held: the paddle moved %.2f cells in one 33 ms frame, want a glide (at most 1.5)", worst)
	}
	if moved < 15 {
		t.Errorf("held 1 s: the paddle moved %.1f cells down, want at least 15", moved)
	}
	stopped := p.target
	for range 5 {
		frame()
	}
	if p.target != stopped {
		t.Errorf("released: the paddle kept moving (%.1f cells)", float32(p.target-stopped)/pongFix)
	}
}

// pitchFrames holds the badge pitched deg degrees (positive: the top tipping
// towards you, the Motion page's pitch) for 3 s, still.
func pitchFrames(a *App, deg float32) {
	r := float64(deg) * math.Pi / 180
	acc := [3]float32{0, float32(gravity * math.Cos(r)), float32(-gravity * math.Sin(r))}
	for range 3 * imuRate {
		imuFrame(a, acc, [3]float32{}, 1.0/imuRate)
	}
	a.pong.tick()
}

// PONG-PITCH-1 (owner 2026-11-15: "the further i pitch the further the
// paddle moves ... 30 degrees ... the middle angle is locked when starting
// the game"): in PLAY your paddle follows the badge's pitch. The pitch PLAY
// starts at (here 25°) is the centre; the full travel is 30°, ±15° around
// it, clamped beyond; the top tipping towards you moves the paddle down.
func TestPongPaddleFollowsPitch(t *testing.T) {
	a := testApp()
	a.motion = newMotion()
	a.pong.motion = a.motion
	p, m := a.pong, a.motion
	pitchFrames(a, 25)
	if got := m.rpy[1] * 180 / math.Pi; got < 24 || got > 26 {
		t.Fatalf("held at 25°, the pitch reads %.1f° (the test's pose against the Motion page's pitch)", got)
	}
	p.showFull()
	p.tick()
	cells := func() float32 { return float32(p.pad[0]) / pongFix }
	if c := cells(); c < 19.9 || c > 20.1 {
		t.Fatalf("PLAY started at 25°: paddle at %.1f cells, want the centre, 20", c)
	}
	for _, c := range []struct{ deg, want float32 }{
		{-20, 0}, {-15, 0}, {-7.5, 10}, {0, 20}, {7.5, 30}, {15, 40}, {20, 40},
	} {
		pitchFrames(a, 25+c.deg)
		if got := cells(); got < c.want-0.5 || got > c.want+0.5 {
			t.Errorf("%+.1f° from where PLAY started: paddle at %.1f cells, want %.0f (0 top, 40 bottom)", c.deg, got, c.want)
		}
	}
}

// Held still in the owner's hand (hand model v2), the paddle doesn't
// jitter: within any second it moves less than a cell (0.75° of pitch). No
// smoothing: over a minute it follows the hand's real slow tilt (v2: 1.5-1.9°
// in 60 s, 2.0-2.6 cells of true pitch; the paddle 1.6-2.1), which is the
// pitch it's meant to follow; smoothing at 0.5 s would only take that to
// 1.2-1.4 cells for 500 ms of lag (PONG-PITCH-1 evidence).
func TestPongPitchStillHandKeepsThePaddle(t *testing.T) {
	for seed := uint64(1); seed <= 3; seed++ {
		r := newRig()
		s := newSensor(seed, 0.016, 0.0006, 0, 0)
		for range 10 * imuRate {
			a, g := s.sample([3]float64{})
			r.feed(a, g, 0.01)
		}
		accSD, gyroSD, tremA, tremG := noiseOf(hand)
		s.accSD, s.gyroSD, s.tremA, s.tremG = accSD, gyroSD, tremA, tremG
		for range 3 * imuRate {
			a, g := s.sample([3]float64{})
			r.feed(a, g, 0.01)
		}
		p := r.a.pong
		p.showFull()
		var last [20]int32 // the paddle over the last second (20 frames)
		n := 0
		for i := range 60 * imuRate {
			a, g := s.sample([3]float64{})
			r.feed(a, g, 0.01)
			if i%r.frameN != r.frameN-1 {
				continue
			}
			last[n%len(last)] = p.pad[0]
			if n++; n < len(last) {
				continue
			}
			lo, hi := last[0], last[0]
			for _, v := range last {
				lo, hi = min(lo, v), max(hi, v)
			}
			if d := float32(hi-lo) / pongFix; d >= 1 {
				t.Fatalf("seed %d, %.1f s: the paddle moved %.2f cells within a second, want < 1", seed, float32(i)/imuRate, d)
			}
		}
	}
}
