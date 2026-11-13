package app

import (
	"time"

	"github.com/itohio/tinygui/examples/framebuffer"
)

func testApp() *App {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	return New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id}, time.Now())
}
