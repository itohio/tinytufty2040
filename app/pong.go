package app

import (
	ui "github.com/itohio/tinygui"
	"image/color"
	"math"
	"time"

	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Pong on a 64×48 LED matrix. Positions are fixed point, 1/256 of a cell
// (no FPU on the M0+). In the page both paddles are CPU players; full
// screen the left paddle is yours: it follows the badge's pitch (with an
// IMU), and UP/DOWN.
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
	pongMinVX    = 64 // a wall's spin kick never slows the ball below this
	// spin: the ball's clockwise surface speed, in the units of its velocity
	// (1/256 cell per tick), kept ×pongSpinFix so it can fade slowly. A
	// paddle hit adds the paddle's movement that tick ×pongPadGrip/4 (the
	// ball's side drags with the paddle); a wall turns 1/pongWallGrip of it
	// into horizontal speed (friction acts on the spin only: a ball without
	// spin bounces as before) and loses that much; in flight it fades by
	// 1/pongSpinFade per tick.
	pongSpinFix  = 256
	pongPadGrip  = 2
	pongWallGrip = 2
	pongSpinFade = 256
	pongMaxSpin  = 160
	pongPitch    = 3
	// your paddle's full travel, bottom to top, is this much pitch around
	// the neutral latched when PLAY starts (owner 2026-11-15: "30 degrees of
	// movement", PONG-PITCH-1)
	pongTiltRange = 30 * math.Pi / 180
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
	padV           [2]int32 // each paddle's movement this tick (+ down)
	spin           int32    // the ball's clockwise surface speed ×pongSpinFix
	target         int32    // player's paddle target (top)
	score          [2]int
	ballHist       [3][2]int32 // previous exact ball positions (one per frame), newest first
	level          int
	player         bool    // left paddle follows target instead of the CPU
	motion         *motion // nil without an IMU
	neutral        float32 // the pitch (rad) for your paddle's centre: latched when PLAY starts
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
	p.spin = 0
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
		was := p.pad[side]
		p.pad[side] = min(max(p.pad[side]+d, 0), (pongH-pongPadLen)*pongFix)
		p.padV[side] = p.pad[side] - was
	}
	p.spin -= p.spin / pongSpinFade // toward 0 from either side

	prev := p.bx
	p.bx += p.vx
	p.by += p.vy
	const bottom = (pongH - 1) * pongFix
	if p.by < 0 {
		p.by, p.vy = -p.by, -p.vy
		p.wallSpin(-1) // the top wall meets the ball's top: it moves at +spin
	} else if p.by > bottom {
		p.by, p.vy = 2*bottom-p.by, -p.vy
		p.wallSpin(1) // the bottom meets it, moving at −spin
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
	// the paddle's movement drags the side it meets: a left paddle moving
	// down turns the ball anticlockwise, a right one clockwise
	v := p.padV[side]
	if side == 0 {
		v = -v
	}
	p.spin = min(max(p.spin+v*pongPadGrip*pongSpinFix/4, -pongMaxSpin*pongSpinFix), pongMaxSpin*pongSpinFix)
	p.vy += p.padV[side] / 4
}

// wallSpin is a wall's friction on the spin: the contact moves at
// side*spin (side −1 top, +1 bottom), and the wall pushes the ball against
// it, taking that from the spin; the ball keeps heading the way it was, at
// pongMinVX to pongMaxVX.
func (p *pong) wallSpin(side int32) {
	kick := p.spin / pongWallGrip
	p.spin -= kick
	vx := p.vx + side*kick/pongSpinFix
	if p.vx < 0 {
		p.vx = -min(max(-vx, pongMinVX), pongMaxVX)
		return
	}
	p.vx = min(max(vx, pongMinVX), pongMaxVX)
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
	p.follow() // paused too: your paddle stays with the badge's pitch
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

// follow puts your paddle where the badge's pitch says (owner 2026-11-15,
// PONG-PITCH-1: "the further i pitch the further the paddle moves"): the
// centre at the neutral pitch, the full travel over pongTiltRange, clamped
// beyond. The pitch is the attitude filter's (gravity-referenced: it
// doesn't drift), the Motion page's: positive tips the top towards you (the
// screen turns to the floor), which moves the paddle down.
func (p *pong) follow() {
	m := p.motion
	if !p.player || m == nil {
		return
	}
	const travel = (pongH - pongPadLen) * pongFix
	pad := int32(travel/2 + (m.rpy[1]-p.neutral)/pongTiltRange*travel)
	pad = min(max(pad, 0), travel)
	p.target, p.pad[0] = pad, pad
}

func (p *pong) fullView() ui.Widget {
	if p.fullOn {
		return p.full
	}
	return nil
}

// pongTap is a tap's step and pongGlide the held speed (owner, TG-6 #4: a
// tap is a small step, held the paddle glides at a set speed): the court's
// 40 free cells in a second.
const (
	pongTap   = 2 * pongFix
	pongGlide = 40 * pongFix // per second
)

// fullPress: UP/DOWN step your paddle (pongTap; held, it glides: glide), A
// pauses, B back to the page, C leaves and changes page.
func (p *pong) fullPress(b Button) bool {
	switch b {
	case BtnUp:
		p.moveBy(-pongTap)
	case BtnDown:
		p.moveBy(pongTap)
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

// holds: UP and DOWN glide the paddle while held (holder).
func (p *pong) holds(b Button) bool { return b == BtnUp || b == BtnDown }

// glide moves your paddle for dt of a held UP or DOWN at pongGlide.
func (p *pong) glide(b Button, dt time.Duration) {
	d := int32(pongGlide * dt.Seconds())
	if b == BtnUp {
		d = -d
	}
	p.moveBy(d)
}

// moveBy moves your paddle's target by d (cells·pongFix, down positive)
// within the court. With the badge's pitch the buttons move the neutral
// instead, by the pitch d is worth: the paddle steps or glides, and the
// pitch carries on from there.
func (p *pong) moveBy(d int32) {
	p.target = min(max(p.target+d, 0), (pongH-pongPadLen)*pongFix)
	if p.player && p.motion != nil {
		p.neutral -= float32(d) / ((pongH - pongPadLen) * pongFix) * pongTiltRange
	}
	p.movePaused()
}

// movePaused puts your paddle at its target at once while the game is
// paused: no physics step runs to glide it there, and it would jump on
// resume (seen on the badge).
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
		return "tilt badge, UP/DN"
	}
	return "UP/DOWN: paddle"
}

func (p *pong) tileViews() (ui.Widget, ui.Widget) { return p.tiles[0], p.tiles[1] }

// showFull starts a game against the CPU on the whole screen; the pitch
// now is your paddle's centre (owner: "the middle angle is locked when
// starting the game", as you look at the screen).
func (p *pong) showFull() {
	p.fullOn, p.player, p.score = true, true, [2]int{}
	p.target = p.pad[0]
	if p.motion != nil {
		p.neutral = p.motion.rpy[1]
	}
}

func (p *pong) reset() { p.score = [2]int{}; p.serve(1) }
