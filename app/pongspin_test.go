package app

import "testing"

// rally runs the court until the ball has bounced off the left paddle and
// then off a wall (top if up), with the left paddle moving at padV (1/256
// cell per tick; + is down) as the ball arrives; it returns the ball's
// horizontal speed just before and just after the wall bounce.
func rally(t *testing.T, padV int32, up bool) (before, after int32) {
	t.Helper()
	p := newPong()
	p.player = true
	p.vx, p.vy = -pongServeVX, 0
	p.bx, p.by = (pongLeftX+6)*pongFix, pongH/2*pongFix
	const face = (pongLeftX + 1) * pongFix
	ticks := (p.bx-face)/pongServeVX + 1                    // until the ball reaches the paddle's face
	p.pad[0] = p.by - pongPadLen/2*pongFix - (ticks-1)*padV // the paddle's centre meets it there
	hit := false
	for range 400 {
		p.target = p.pad[0] + padV // keep the paddle moving at padV
		vxPrev, vyPrev := p.vx, p.vy
		p.step()
		if !hit && vxPrev < 0 && p.vx > 0 {
			hit = true
			p.vy = 80 // towards a wall, reached before the far paddle
			if up {
				p.vy = -80
			}
			continue
		}
		if hit && (vyPrev < 0) != (p.vy < 0) && (vxPrev < 0) == (p.vx < 0) { // a wall bounce (a paddle turns vx)
			return vxPrev, p.vx
		}
	}
	t.Fatalf("no paddle hit then wall bounce (hit %v)", hit)
	return 0, 0
}

// A ball without spin bounces off a wall as it always did: its horizontal
// speed unchanged.
func TestPongWallBounceWithoutSpinIsUnchanged(t *testing.T) {
	for _, up := range []bool{true, false} {
		if b, a := rally(t, 0, up); a != b {
			t.Errorf("still paddle, wall (up %v): vx %d before, %d after; want unchanged", up, b, a)
		}
	}
}

// The paddle's movement at contact spins the ball, and the spin kicks it
// sideways at a wall, predictably by sign: a paddle moving up spins it so
// that the top wall slows it and the bottom wall speeds it up; moving down,
// the other way round.
func TestPongPaddleSpinDeflectsAtTheWall(t *testing.T) {
	for _, c := range []struct {
		padV   int32
		up     bool
		slower bool
	}{{-100, true, true}, {-100, false, false}, {100, true, false}, {100, false, true}} {
		b, a := rally(t, c.padV, c.up)
		if c.slower && a >= b || !c.slower && a <= b {
			t.Errorf("paddle %+d, wall (up %v): vx %d → %d, want slower %v", c.padV, c.up, b, a, c.slower)
		}
	}
}

// However much spin, a wall never stops or turns the ball back: it keeps
// heading for the far paddle at no less than pongMinVX.
func TestPongSpinNeverStallsTheBall(t *testing.T) {
	for _, padV := range []int32{-160, 160} {
		for _, up := range []bool{true, false} {
			if _, a := rally(t, padV, up); a < pongMinVX {
				t.Errorf("paddle %+d, wall (up %v): vx %d after, want ≥ %d", padV, up, a, pongMinVX)
			}
		}
	}
}
