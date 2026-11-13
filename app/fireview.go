package app

import (
	ui "github.com/itohio/tinygui"
)

// fireView draws the 160×120 heat field at any size in real pixels (owner:
// only the water is pixelated; the fire renders at proper resolution): the
// page at 1:1, the menu tiles sampled down, and full screen (320×240)
// interpolated, not 2×2 blocks. Interpolated pixels take the mean of their
// neighbours' heat (before the palette, so gradients stay smooth); fuel is
// never blended with flames, so the pool's edge stays sharp. Rows are built
// as RGB565 scanlines and copied; rows outside the band being drawn are
// skipped. A 320×240 heat field would need 76.8 KB the heap doesn't have.
type fireView struct {
	ui.WidgetBase
	f *fire
}

// shared by every fireView: drawing is single-threaded
var (
	fireLineA, fireLineB [2 * heatW]uint16
	firePal565           = func() (p [fireFuel + 1]uint16) {
		for i, c := range firePalette {
			p[i] = rgb565(c.R, c.G, c.B)
		}
		return
	}()
)

func newFireView(f *fire, w, h uint16) *fireView {
	return &fireView{WidgetBase: ui.NewWidgetBase(w, h), f: f}
}

// blend is the heat between a and b: their mean, unless either is fuel.
func blend(a, b uint8) uint8 {
	if a == fireFuel || b == fireFuel {
		return a // fuel keeps its edge: nearest, no blend
	}
	return (a + b) / 2
}

func (v *fireView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	bmp, ok := d.(ui.BitmapDisplayer)
	if !ok {
		return
	}
	w, h := int(v.Width), int(v.Height)
	r0, r1 := 0, h // screen rows of this widget in the band being drawn
	if c, ok := d.(ui.Clipper); ok {
		_, cy, _, ch := c.Clip()
		r0, r1 = max(0, int(cy-y0)), min(h, int(cy+ch-y0))
	}
	if w == 2*heatW && h == 2*heatH {
		v.drawDouble(bmp, x0, y0, r0, r1)
		return
	}
	// nearest heat cell, stepped in 16.16 fixed point (no division per pixel)
	sx, sy := heatW<<16/w, heatH<<16/h
	hf := &v.f.heat
	line := fireLineA[:w]
	for y := r0; y < r1; y++ {
		row := (y * sy >> 16) * heatW
		fx := 0
		for x := range line {
			line[x] = firePal565[hf[row+fx>>16]]
			fx += sx
		}
		_ = bmp.DrawRGBBitmap(x0, y0+int16(y), line, int16(w), 1)
	}
}

// drawDouble draws the field at twice its size: each heat row gives two
// screen rows, the odd pixels interpolated.
func (v *fireView) drawDouble(bmp ui.BitmapDisplayer, x0, y0 int16, r0, r1 int) {
	hf := &v.f.heat
	for y := r0 / 2; y < (r1+1)/2; y++ {
		next := min(y+1, heatH-1)
		for x := range heatW {
			nx := min(x+1, heatW-1)
			a, b := hf[y*heatW+x], hf[y*heatW+nx]
			c, e := hf[next*heatW+x], hf[next*heatW+nx]
			fireLineA[2*x] = firePal565[a]
			fireLineA[2*x+1] = firePal565[blend(a, b)]
			fireLineB[2*x] = firePal565[blend(a, c)]
			fireLineB[2*x+1] = firePal565[blend(blend(a, b), blend(c, e))]
		}
		if 2*y >= r0 {
			_ = bmp.DrawRGBBitmap(x0, y0+int16(2*y), fireLineA[:], 2*heatW, 1)
		}
		if 2*y+1 < r1 {
			_ = bmp.DrawRGBBitmap(x0, y0+int16(2*y+1), fireLineB[:], 2*heatW, 1)
		}
	}
}
