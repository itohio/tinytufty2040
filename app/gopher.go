package app

import (
	"image/color"
	"time"

	ui "github.com/itohio/tinygui"
)

// The boot animation: a pixel-art gopher (after the Go gopher by Renée
// French) blinks and waves over "TinyGo" for bootDur after power-up, while
// the firmware is already reading the sensor; any button skips it (the
// press does nothing else), so does a console command (which still runs).
// The gopher is not a bitmap: gopherAt works out each of its 32×32 cells
// from a few ellipses, drawn 4 px a cell.
const (
	bootDur     = 2 * time.Second
	gopherCells = 32
	gopherPx    = 4 // screen px per cell
)

// the gopher's colours (gopherAt's indices; 0 is the background, not drawn)
var gopherPalette = [...]color.RGBA{
	1: {24, 56, 66, 255},    // outline
	2: {106, 215, 229, 255}, // body
	3: {250, 250, 250, 255}, // eyes, teeth
	4: {16, 16, 16, 255},    // pupils, eyelids
	5: {238, 206, 160, 255}, // snout, hands, feet
	6: {70, 46, 38, 255},    // nose
}

type bootAnim struct {
	on bool
	t0 time.Time
}

// bootStep ends the animation once bootDur has passed.
func (a *App) bootStep(now time.Time) {
	if a.boot.on && now.Sub(a.boot.t0) >= bootDur {
		a.boot.on = false
	}
}

// bootSkip ends the animation; true if it was on.
func (a *App) bootSkip() bool {
	on := a.boot.on
	a.boot.on = false
	return on
}

// drawBoot draws the animation's frame at a.now: the gopher blinks at
// 0.7 s and waves from 0.9 s to 1.8 s.
func (a *App) drawBoot() {
	t := a.now.Sub(a.boot.t0)
	blink := t >= 700*time.Millisecond && t < 850*time.Millisecond
	wave := t >= 900*time.Millisecond && t < 1800*time.Millisecond && (t-900*time.Millisecond)/(200*time.Millisecond)%2 == 0
	const size = gopherCells * gopherPx
	x0, y0 := int16((ScreenW-size)/2), int16(20)
	for y := range int32(gopherCells) {
		for x := range int32(gopherCells) {
			if c := gopherAt(x, y, blink, wave); c != 0 {
				fillRect(a.d, x0+int16(x)*gopherPx, y0+int16(y)*gopherPx, gopherPx, gopherPx, gopherPalette[c])
			}
		}
	}
	const title = "TinyGo"
	ui.DrawText(a.d, fontTitle, (ScreenW-textWidth(fontTitle, title))/2, y0+size+36, title, colFG)
}

// inEllipse reports whether the doubled point (x2, y2) lies in the ellipse
// centred on (cx, cy) with radii rx, ry (all doubled: cell centres fall on
// odd numbers).
func inEllipse(x2, y2, cx, cy, rx, ry int32) bool {
	dx, dy := x2-cx, y2-cy
	return dx*dx*ry*ry+dy*dy*rx*rx <= rx*rx*ry*ry
}

// gopherAt is the gopher's colour index at cell (x, y), 0 for the
// background: eyes, nose, teeth and snout over the body, its outline, then
// the ears, arms and feet behind it.
func gopherAt(x, y int32, blink, wave bool) uint8 {
	X, Y := 2*x+1, 2*y+1
	for _, cx := range [2]int32{22, 42} { // the eyes
		if inEllipse(X, Y, cx, 20, 8, 8) {
			if blink {
				if Y >= 20 && Y <= 23 {
					return 4 // the closed lid
				}
				return 2
			}
			if inEllipse(X, Y, cx+3, 22, 3, 3) {
				return 4
			}
			return 3
		}
		if inEllipse(X, Y, cx, 20, 10, 10) {
			return 1
		}
	}
	switch {
	case inEllipse(X, Y, 32, 32, 4, 3):
		return 6 // the nose
	case (x == 15 || x == 16) && (y == 21 || y == 22):
		return 3 // the teeth
	case inEllipse(X, Y, 32, 36, 9, 6):
		return 5 // the snout
	case inEllipse(X, Y, 32, 32, 22, 24):
		return 2 // the body
	case inEllipse(X, Y, 32, 32, 24, 26):
		return 1
	}
	rightArm := [2]int32{54, 42}
	if wave {
		rightArm = [2]int32{56, 20}
	}
	for _, e := range [...][4]int32{
		{14, 8, 6, 6}, {50, 8, 6, 6}, // ears
		{10, 42, 4, 6}, {rightArm[0], rightArm[1], 4, 6}, // arms
		{20, 57, 7, 3}, {44, 57, 7, 3}, // feet
	} {
		if inEllipse(X, Y, e[0], e[1], e[2], e[3]) {
			if e[1] == 8 {
				return 2 // the ears are the body's colour
			}
			return 5
		}
		if inEllipse(X, Y, e[0], e[1], e[2]+2, e[3]+2) {
			return 1
		}
	}
	return 0
}
