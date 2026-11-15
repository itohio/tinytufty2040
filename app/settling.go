package app

import (
	ui "github.com/itohio/tinygui"
	"tinygo.org/x/drivers"
)

// settlingText is shown while the motion state hasn't started (MATH-1
// A1): until it has, the paddle and the boids' window don't move, and a
// frozen paddle must not be silent. In the footer and on the full-screen
// motion views; ASCII only (the font).
const settlingText = "settling - hold still"

// settling reports whether there is a motion state that hasn't started.
func (m *motion) settling() bool { return m != nil && !m.settled }

// drawSettling writes settlingText centred near the bottom of a full-screen
// view (x0, y0, w × h) while m settles. A constant string: nothing allocated.
func drawSettling(d drivers.Displayer, m *motion, x0, y0, w, h int16) {
	if !m.settling() {
		return
	}
	ui.DrawText(d, fontBody, x0+(w-textWidth(fontBody, settlingText))/2, y0+h-10, settlingText, colWarn)
}
