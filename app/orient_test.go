package app

import (
	"image/color"
	"math"
	"testing"
	"time"
)

// pixelLog records what a line sets: how many pixels and whether any fell
// outside the view.
type pixelLog struct {
	n       int
	r       viewRect
	outside int
}

func (p *pixelLog) Size() (int16, int16) { return ScreenW, ScreenH }
func (p *pixelLog) SetPixel(x, y int16, _ color.RGBA) {
	p.n++
	if x < p.r.x || y < p.r.y || x >= p.r.x+p.r.w || y >= p.r.y+p.r.h {
		p.outside++
	}
}
func (p *pixelLog) Display() error { return nil }

// The Motion page's lines finish and stay in the view whatever their ends
// (owner, TG-6: the page froze once the position ran away). A runaway
// position projects the path's points tens of thousands of pixels off the
// view; the int16 Bresenham over such a segment overflowed and never ended
// (line(100, 100 → −30000, 20000) set half a billion pixels in 2 s).
func TestPathLinesFinishAndStayInTheView(t *testing.T) {
	r := viewRect{10, 20, 200, 150}
	inf := float32(math.Inf(1))
	nan := float32(math.NaN())
	cases := []struct {
		name           string
		x0, y0, x1, y1 float32
		want           int // pixels, -1: up to the view's diagonal
	}{
		{"inside, horizontal", 20, 30, 59, 30, 40},
		{"one end far off", 100, 100, -30000, 20000, -1},
		{"both ends far off, across", -40000, 95, 40000, 95, 200},
		{"both ends far off, missing", -40000, -5000, 40000, -6000, 0},
		{"from inside to beyond int16", 100, 100, 1e7, 1e7, -1},
		{"infinite", 100, 100, inf, 50, 0},
		{"not a number", nan, 50, 100, 50, 0},
	}
	for _, c := range cases {
		p := &pixelLog{r: r}
		done := make(chan struct{})
		go func() {
			line(p, r, c.x0, c.y0, c.x1, c.y1, color.RGBA{})
			close(done)
		}()
		select {
		case <-done:
		case <-time.After(time.Second):
			t.Fatalf("%s: still drawing after 1 s (%d pixels)", c.name, p.n)
		}
		if p.outside != 0 {
			t.Errorf("%s: %d pixels outside the view", c.name, p.outside)
		}
		diag := int(math.Hypot(float64(r.w), float64(r.h))) + 2
		switch {
		case c.want >= 0 && p.n != c.want:
			t.Errorf("%s: %d pixels, want %d", c.name, p.n, c.want)
		case c.want < 0 && (p.n == 0 || p.n > diag):
			t.Errorf("%s: %d pixels, want some, at most the view's diagonal (%d)", c.name, p.n, diag)
		}
	}
}
