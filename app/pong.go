package app

import (
	ui "github.com/itohio/tinygui"
	"image/color"
	"time"

	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Pong on a 64×48 LED matrix. Positions are fixed point, 1/256 of a cell
// (no FPU on the M0+). In the page both paddles are CPU players; full
// screen the left paddle is yours: it follows the badge as you move it up
// and down (with an IMU), and UP/DOWN.
const (
	pongW, pongH = 64, 48
	pongFix      = 256
	pongPadLen   = 8
	pongLeftX    = 2 // paddle columns
	pongRightX   = pongW - 3
	pongWin      = 9
	pongTicks    = 2 // physics ticks per frame
	pongServeVX  = 128
	pongMaxVX    = 320
	pongPitch    = 3
	// your paddle follows the badge's displacement along its own up axis
	// (motion.disp[1]; owner 2026-11-13: "vertical position in the badge
	// frame of reference, not real world frame of reference") 1:1 on
	// the glass (owner 2026-11-13: "move the board up or down and the paddle
	// move up or down with the same speed and motion"): full screen a cell is
	// 5 px, 0.6375 mm on the 2.0" panel (320 px across 40.8 mm), so the
	// paddle's whole travel (40 cells) is 2.55 cm of badge motion
	pongCellsPerM = 1000 / 0.6375
)

var pongLevels = []string{"easy", "normal", "hard"}

// CPU paddle speed per level, 1/256 cell per tick
var pongCPUSpeed = [3]int32{44, 72, 112}

// palette indices
const (
	pongOff = iota
	pongNet
	pongScore
	pongPaddle
	pongTrail1
	pongTrail2
	pongTrail3
	pongBall
)

var pongPalette = []color.RGBA{
	{14, 14, 22, 255}, {40, 44, 70, 255}, {90, 120, 160, 255}, {235, 240, 255, 255},
	{70, 30, 0, 255}, {140, 70, 10, 255}, {210, 120, 20, 255}, {255, 210, 70, 255},
}

// 3×5 digits, rows top to bottom, 3 bits per row (bit 2 = left column)
var pongDigits = [10][5]uint8{
	{7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
	{7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7},
}

type pong struct {
	bx, by, vx, vy int32
	pad            [2]int32 // paddle tops (left, right)
	target         int32    // player's paddle target (top)
	score          [2]int
	ballHist       [3][2]int32 // previous exact ball positions (one per frame), newest first
	level          int
	player         bool    // left paddle follows target instead of the CPU
	motion         *motion // nil without an IMU
	anchorY        float32 // the badge's displacement along its up axis (m) where the paddle was at anchorPad
	anchorPad      int32
	paused         bool
	stepUS         int64
	grid, full     *pongView
	tiles          [2]*pongView
	fullOn         bool
	txt            textBuf
	log            func(string)
}

func newPong() *pong {
	p := &pong{level: 1}
	p.pad[0] = (pongH - pongPadLen) / 2 * pongFix
	p.pad[1] = p.pad[0]
	p.target = p.pad[0]
	p.serve(1)
	p.grid = newPongView(p, pongW*pongPitch, pongH*pongPitch)
	p.full = newPongView(p, ScreenW, ScreenH)
	p.tiles = [2]*pongView{newPongView(p, tileW, tileH), newPongView(p, 2*tileW, 2*tileH)}
	return p
}

// serve puts the ball in the centre moving towards side dir (-1 left, +1 right).
func (p *pong) serve(dir int32) {
	p.bx, p.by = pongW/2*pongFix, pongH/2*pongFix
	p.vx = dir * pongServeVX
	p.vy = int32(rand32()&63) + 24
	if rand32()&1 == 0 {
		p.vy = -p.vy
	}
	for i := range p.ballHist {
		p.ballHist[i] = [2]int32{p.bx, p.by}
	}
}

// step advances the physics one tick.
func (p *pong) step() {
	for side := range 2 {
		var speed, goal int32
		if side == 0 && p.player {
			speed, goal = 160, p.target
		} else {
			speed = pongCPUSpeed[p.level]
			goal = (pongH - pongPadLen) / 2 * pongFix // drift to the centre
			if (side == 0) == (p.vx < 0) {            // ball coming: follow it
				goal = p.by - pongPadLen/2*pongFix
			}
		}
		d := min(max(goal-p.pad[side], -speed), speed)
		p.pad[side] = min(max(p.pad[side]+d, 0), (pongH-pongPadLen)*pongFix)
	}

	prev := p.bx
	p.bx += p.vx
	p.by += p.vy
	const bottom = (pongH - 1) * pongFix
	if p.by < 0 {
		p.by, p.vy = -p.by, -p.vy
	} else if p.by > bottom {
		p.by, p.vy = 2*bottom-p.by, -p.vy
	}
	// a paddle is hit when the ball crosses into its column this tick (a
	// crossing test: at top speed the ball moves more than a cell per tick)
	const leftFace, rightFace = (pongLeftX + 1) * pongFix, pongRightX * pongFix
	switch {
	case p.vx < 0 && prev >= leftFace && p.bx < leftFace:
		p.hit(0, leftFace)
	case p.vx > 0 && prev < rightFace && p.bx >= rightFace:
		p.hit(1, rightFace-1)
	case p.bx < 0:
		p.point(1)
	case p.bx >= pongW*pongFix:
		p.point(0)
	}
}

// hit bounces the ball off paddle side if it is in reach, angling it by
// where it struck the paddle and speeding it up by 1/16.
func (p *pong) hit(side int, face int32) {
	off := p.by + pongFix/2 - p.pad[side] // ball centre from the paddle top
	if off < 0 || off >= pongPadLen*pongFix {
		return // missed: the ball flies on and scores
	}
	p.bx = face
	vx := -p.vx
	vx += vx >> 4
	p.vx = min(max(vx, -pongMaxVX), pongMaxVX)
	p.vy = (off - pongPadLen/2*pongFix) >> 3
}

// point scores for side (0 left, 1 right); after pongWin points both reset.
func (p *pong) point(side int) {
	p.score[side]++
	if p.score[side] > pongWin {
		p.score = [2]int{}
	}
	p.serve(int32(1 - 2*side)) // towards the side that missed
}

func (p *pong) tick() {
	p.follow() // paused too: your paddle stays with the badge
	if p.paused {
		return
	}
	t0 := time.Now()
	copy(p.ballHist[1:], p.ballHist[:len(p.ballHist)-1])
	p.ballHist[0] = [2]int32{p.bx, p.by}
	for range pongTicks {
		p.step()
	}
	p.stepUS = time.Since(t0).Microseconds()
}

// follow moves your paddle with the badge's displacement along its own up
// axis since the anchor, at
// once (same speed as the badge). At the top or bottom it stops and the
// anchor moves with the badge, so turning back moves the paddle at once
// (like a mouse at the screen's edge).
func (p *pong) follow() {
	m := p.motion
	if !p.player || m == nil {
		return
	}
	y := m.disp[1]
	pad := p.anchorPad - int32((y-p.anchorY)*pongCellsPerM*pongFix) // up the screen is -y
	if lim := int32((pongH - pongPadLen) * pongFix); pad < 0 || pad > lim {
		pad = min(max(pad, 0), lim)
		p.anchor(pad)
	}
	p.target, p.pad[0] = pad, pad
}

// anchor ties the paddle position pad to the badge's displacement now.
func (p *pong) anchor(pad int32) {
	p.anchorPad = pad
	if p.motion != nil {
		p.anchorY = p.motion.disp[1]
	}
}

func (p *pong) fullView() ui.Widget {
	if p.fullOn {
		return p.full
	}
	return nil
}

// fullPress: UP/DOWN move your paddle (6 cells per press or repeat), A
// pauses, B back to the page, C leaves and changes page.
func (p *pong) fullPress(b Button) bool {
	switch b {
	case BtnUp:
		p.target = max(p.target-6*pongFix, 0)
		p.anchor(p.target) // the badge carries on from here
		p.movePaused()
	case BtnDown:
		p.target = min(p.target+6*pongFix, (pongH-pongPadLen)*pongFix)
		p.anchor(p.target)
		p.movePaused()
	case BtnA:
		p.paused = !p.paused
	case BtnB:
		p.leave()
	case BtnC:
		p.leave()
		return false
	}
	return true
}

// movePaused puts your paddle at its target at once while the game is
// paused: no physics step runs to glide it there, and it would jump on
// resume (verify2 TG-pong-paddle).
func (p *pong) movePaused() {
	if p.paused {
		p.pad[0] = p.target
	}
}

func (p *pong) leave() {
	p.fullOn, p.player, p.paused = false, false, false
}

func (a *App) pagePong() *page {
	p := a.pong
	level := widget.NewInteractiveLabelChoice(52, 11, pongLevels,
		widget.WithLabelChoiceIndex(&p.level), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	play := widget.NewToggle(44, 12, fontBody, colFG, "PLAY", "PLAY", colTrack, colTrack,
		func() bool { return false }, func(bool) { p.showFull(); a.log("pong: you vs cpu") })
	// explicit height: findings F-33
	pg := a.newPageWith("Pong", layout.VFlow(4, pongH*pongPitch),
		still(p.grid),
		newField("CPU", 42, 104, level),
		newField("You", 42, 104, play),
		a.caption(104, pongHint(p.motion != nil)),
		a.caption(104, "A pause, B back"),
		still(a.text(104, func() string { return p.txt.reset().str("step ").int(int(p.stepUS)).str(" us").String() }, colMuted)),
	)
	pg.demo = p
	return pg
}

// pongHint is the page's line on how to move your paddle.
func pongHint(imu bool) string {
	if imu {
		return "move badge, UP/DN"
	}
	return "UP/DOWN: paddle"
}

func (p *pong) tileViews() (ui.Widget, ui.Widget) { return p.tiles[0], p.tiles[1] }

// showFull starts a game against the CPU on the whole screen.
func (p *pong) showFull() {
	p.fullOn, p.player, p.score = true, true, [2]int{}
	p.target = p.pad[0]
	p.anchor(p.pad[0])
}

func (p *pong) reset() { p.score = [2]int{}; p.serve(1) }
