package app

import ui "github.com/itohio/tinygui"

// pongView draws Pong at full resolution (owner: "boids and paddles - full
// resolution!!!"): paddles, ball and its trail at their exact fixed-point
// positions scaled to pixels, the net, and the score in the 3×5 digits drawn
// as pixel blocks. Any size: the page, full screen and the menu tiles.
type pongView struct {
	ui.WidgetBase
	p *pong
}

func newPongView(p *pong, w, h uint16) *pongView {
	return &pongView{WidgetBase: ui.NewWidgetBase(w, h), p: p}
}

func (v *pongView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	w, h := int32(v.Width), int16(v.Height)
	p := v.p
	const span = pongW * pongFix // world units across; pixels = units·w/span
	cell := int16(w / pongW)     // pixels per cell
	fillRect(d, x0, y0, int16(w), h, pongPalette[pongOff])
	if v == p.full {
		drawSettling(d, p.motion, x0, y0, int16(w), h)
	}
	// net
	nx := x0 + int16(w/2) - cell/4
	for y := int16(0); y < h; y += 2 * cell {
		fillRect(d, nx, y0+y, max(cell/2, 1), cell, pongPalette[pongNet])
	}
	// score
	for side, sx := range [2]int16{x0 + int16(w/2) - 7*cell, x0 + int16(w/2) + 4*cell} {
		for r, bits := range pongDigits[p.score[side]] {
			for c := range 3 {
				if bits&(4>>c) != 0 {
					fillRect(d, sx+int16(c)*cell, y0+cell+int16(r)*cell, cell, cell, pongPalette[pongScore])
				}
			}
		}
	}
	// paddles
	for side, col := range [2]int32{pongLeftX, pongRightX} {
		px := x0 + int16(col*pongFix*w/span)
		py := y0 + int16(p.pad[side]*w/span)
		fillRect(d, px, py, cell, int16(pongPadLen*pongFix*w/span), pongPalette[pongPaddle])
	}
	// trail, then the ball
	for i := len(p.ballHist) - 1; i >= 0; i-- {
		t := p.ballHist[i]
		s := max(cell-int16(i), 2)
		fillRect(d, x0+int16(t[0]*w/span), y0+int16(t[1]*w/span), s, s, pongPalette[pongTrail3-uint8(i)])
	}
	bx, by := x0+int16(p.bx*w/span), y0+int16(p.by*w/span)
	if cell < 3 { // too small to round: a 2×2 ball
		fillRect(d, bx, by, 2, 2, pongPalette[pongBall])
		return
	}
	fillRect(d, bx+1, by, cell-2, cell, pongPalette[pongBall])
	fillRect(d, bx, by+1, cell, cell-2, pongPalette[pongBall])
}
