package app

import (
	"image/color"
	"time"

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

// the full view's field-of-view readout: solid for boidsFOVHold after a
// change, then fading into the background over boidsFOVFade
const (
	boidsFOVHold = 700 * time.Millisecond
	boidsFOVFade = 800 * time.Millisecond
)

// fovReadout is the readout's colour since the last change; false once it
// has faded (or before any change).
func (b *boids) fovReadout(since time.Duration) (color.RGBA, bool) {
	if b.fovAt.IsZero() || since >= boidsFOVHold+boidsFOVFade {
		return color.RGBA{}, false
	}
	k := int32(256)
	if since > boidsFOVHold {
		k = int32(256 - (since-boidsFOVHold)*256/boidsFOVFade)
	}
	return color.RGBA{mix8(colFG.R, boidsViewBG.R, k), mix8(colFG.G, boidsViewBG.G, k), mix8(colFG.B, boidsViewBG.B, k), 255}, true
}

// mix8 is f·k + g·(1 − k), k in 256ths.
func mix8(f, g uint8, k int32) uint8 { return uint8((int32(f)*k + int32(g)*(256-k)) >> 8) }

func (v *boidsView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	fillRect(d, x0, y0, int16(v.Width), int16(v.Height), boidsViewBG)
	b := v.b
	if v == b.full {
		drawSettling(d, b.motion, x0, y0, int16(v.Width), int16(v.Height))
		if c, ok := b.fovReadout(time.Since(b.fovAt)); ok {
			l := boidsFOVLabels[b.fov]
			ui.DrawText(d, fontBody, x0+(int16(v.Width)-textWidth(fontBody, l))/2, y0+14, l, c)
		}
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
