package app

import ui "github.com/itohio/tinygui"

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
