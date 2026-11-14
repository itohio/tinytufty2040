package app

import (
	"time"

	ui "github.com/itohio/tinygui"
)

// demo is a page that animates: the App ticks it once per frame while its
// page is shown, and draws its full-screen grid, when it has one, instead of
// the UI.
type demo interface {
	tick()
	// fullView is what to show on the whole screen, or nil. Called from
	// Update, so the demo may repoint the grid's cells here.
	fullView() ui.Widget
	// fullPress handles a button while full screen; false lets the App
	// handle it as usual (C changes the page).
	fullPress(b Button) bool
}

// holder is a full-screen demo that acts on a button for as long as it is
// held (Pong's paddle, owner TG-6 #4: a tap steps, held it glides). The App
// keeps the button's auto-repeats from it and calls glide every frame the
// button has been held longer than holdGlideAfter.
type holder interface {
	holds(b Button) bool
	glide(b Button, dt time.Duration)
}

// holdGlideAfter: a press shorter than this is a tap.
const holdGlideAfter = 250 * time.Millisecond
