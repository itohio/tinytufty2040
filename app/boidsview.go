package app

import (
	"image/color"

	ui "github.com/itohio/tinygui"
)

// boidsView draws the flock as the badge sees it (boids.project): each boid
// at its projected position, a 3×3 head coloured by its on-screen heading
// with a nose pixel, and a fading trail of its last positions. Any size: the
// page, full screen and the menu tiles; the field across is boidsFOV
// whatever the size.
type boidsView struct {
	ui.WidgetBase
	b *boids
}

func newBoidsView(b *boids, w, h uint16) *boidsView {
	return &boidsView{WidgetBase: ui.NewWidgetBase(w, h), b: b}
}

// nose offset per heading octant (0 east, clockwise on screen)
var boidNose = [8][2]int16{{2, 0}, {2, 2}, {0, 2}, {-2, 2}, {-2, 0}, {-2, -2}, {0, -2}, {2, -2}}

var boidsViewBG = color.RGBA{8, 9, 14, 255}

// octant is the heading octant of a screen-space direction (dx right, dy
// down), 0 east, clockwise.
func octant(dx, dy float32) uint8 {
	ax, ay := dx, dy
	if ax < 0 {
		ax = -ax
	}
	if ay < 0 {
		ay = -ay
	}
	var o uint8
	switch {
	case ax > 2.414*ay: // within 22.5° of horizontal
		o = 0
	case ay > 2.414*ax:
		o = 2
	default:
		o = 1
	}
	switch {
	case dx >= 0 && dy >= 0:
		return o
	case dx < 0 && dy >= 0:
		return 4 - o
	case dx < 0:
		return 4 + o
	}
	return (8 - o) % 8
}

func (v *boidsView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	fillRect(d, x0, y0, int16(v.Width), int16(v.Height), boidsViewBG)
	b := v.b
	if v == b.full {
		drawSettling(d, b.motion, x0, y0, int16(v.Width), int16(v.Height))
	}
	// the frame's projection (boids.frame, once per tick): only scaling here
	cx, cy, s := int32(x0)+int32(v.Width)/2, int32(y0)+int32(v.Height)/2, int32(v.Width)/2
	lx, ly, hx0, hy0 := int32(x0), int32(y0), int32(x0)+int32(v.Width)-2, int32(y0)+int32(v.Height)-2 // the heads' room
	n := boidsN[b.count]
	for k := 1; k < boidsHist; k++ { // trails, oldest dimmest
		c := boidsPalette[boidsTrail+1-k*boidsTrail/boidsHist]
		for i := range n {
			t := b.trail[i][k-1]
			if t[0] == boidsOff {
				continue
			}
			if x, y := cx+int32(t[0])*s>>12, cy-int32(t[1])*s>>12; x >= lx && y >= ly && x < hx0 && y < hy0 {
				fillRect(d, int16(x), int16(y), 2, 2, c)
			}
		}
	}
	for i := range n {
		h := b.head[i]
		if h[0] == boidsOff {
			continue
		}
		x, y := cx+int32(h[0])*s>>12, cy-int32(h[1])*s>>12
		if x-1 < lx || y-1 < ly || x-1 >= hx0 || y-1 >= hy0 {
			continue // out of view: it's on its way back
		}
		o := b.oct[i]
		c := boidsPalette[boidsTrail+1+o]
		fillRect(d, int16(x)-1, int16(y)-1, 3, 3, c)
		fillRect(d, int16(x)+boidNose[o][0], int16(y)+boidNose[o][1], 1, 1, c)
	}
}
