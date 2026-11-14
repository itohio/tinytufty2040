package app

import (
	"image/color"

	ui "github.com/itohio/tinygui"
)

// boidsView draws the flock at full resolution (owner: "boids and paddles -
// full resolution!!!"): each boid at its exact position (1/256 cell scaled
// to pixels), a 3×3 head coloured by its on-screen heading with a nose
// pixel, and a fading trail of its last positions. Any size: the page,
// full screen and the menu tiles.
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

func (v *boidsView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	w, h := int32(v.Width), int16(v.Height)
	fillRect(d, x0, y0, int16(w), h, boidsViewBG)
	b := v.b
	const span = boidsW * boidsFix // world units across the box; pixels = units·w/span on both axes
	n := boidsN[b.count]
	for k := 1; k < boidsHist; k++ { // trails, oldest dimmest
		c := boidsPalette[boidsTrail+1-k*boidsTrail/boidsHist]
		slot := (b.histI - 1 - k + 2*boidsHist) % boidsHist
		for i := range n {
			hc := b.histC[slot]
			bx, by := b.pose.toBox(int32(b.hist[i][slot][0])+hc[0], int32(b.hist[i][slot][1])+hc[1])
			if bx < 0 || by < 0 || bx >= span || by >= boidsH*boidsFix {
				continue
			}
			fillRect(d, x0+int16(bx*w/span), y0+int16(by*w/span), 2, 2, c)
		}
	}
	for i := range n {
		bx, by := b.pose.toBox(b.x[i], b.y[i])
		if bx < 0 || by < 0 || bx >= span || by >= boidsH*boidsFix {
			continue // left outside by the window, gliding back (walls)
		}
		o := octant(b.pose.vecToBox(b.vx[i], b.vy[i]))
		c := boidsPalette[boidsTrail+1+o]
		x, y := x0+int16(bx*w/span), y0+int16(by*w/span)
		fillRect(d, x-1, y-1, 3, 3, c)
		fillRect(d, x+boidNose[o][0], y+boidNose[o][1], 1, 1, c)
	}
}
