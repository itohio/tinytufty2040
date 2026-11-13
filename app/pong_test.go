package app

import "testing"

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
// resuming doesn't make it jump (verify2 TG-pong-paddle: the paddle stayed
// put while paused and leapt 6 cells on resume).
func TestPongPausedPaddleMovesWithButtons(t *testing.T) {
	p := newPong()
	p.showFull()
	p.fullPress(BtnA) // pause
	before := p.pad[0]
	p.fullPress(BtnUp)
	p.tick()
	if got, want := p.pad[0], before-6*pongFix; got != want {
		t.Fatalf("paused, UP: paddle at %.1f cells, want %.1f (6 up, drawn now)", float32(got)/pongFix, float32(want)/pongFix)
	}
	p.fullPress(BtnA) // resume
	moved := p.pad[0]
	p.tick()
	if d := p.pad[0] - moved; d != 0 {
		t.Fatalf("on resume the paddle moved %.1f cells by itself", float32(d)/pongFix)
	}
}
