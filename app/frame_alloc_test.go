package app

import (
	"runtime"
	"strings"
	"testing"
	"time"

	"github.com/itohio/tinygui/examples/framebuffer"
)

type testHW struct{}

func (testHW) Light() uint16            { return 1000 }
func (testHW) Battery() (float32, bool) { return 3.9, false }
func (testHW) SetLED(int)               {}
func (testHW) SetBacklight(int)         {}

// framedApp is the app with an IMU (every page, the Motion page too) and a
// frame as the badge runs it: sensor sample, Update, three 80-row bands.
func framedApp() (*App, func()) {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	now := time.Now()
	a := New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id, IMU: true}, now)
	frame := func() {
		now = now.Add(33 * time.Millisecond)
		a.SetIMU([3]float32{-1.5, 9.4, 1.5}, [3]float32{0.01, -0.02, 0.005}, 0.033)
		a.Update(now)
		for b := range 3 {
			fb.SetY0(int16(b * 80))
			a.Draw()
		}
	}
	return a, frame
}

// Every page draws its frames (three 80-row bands, as on the Tufty) without
// allocating: per-frame garbage fills the badge's heap and the collections
// that follow cost frames (findings F-34).
func TestPagesDrawFramesWithoutAllocating(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	for i := range a.pages.Count() {
		a.showPage(i)
		for range 40 {
			frame()
		}
		if n := testing.AllocsPerRun(30, frame); n != 0 {
			t.Errorf("page %s: %.0f allocations per frame", a.pages.Name(i), n)
		}
	}
}

// menuRound is one round of tools/soak: buttons only, as a person uses the
// badge. u/d/a/b/c are presses, f a few frames. On Demos every tile opens
// full screen and back, the speed dropdown and the options dialog open and
// close; then C walks every page doing its usual interactions (edits, list
// scrolling, each demo's Full view) and wraps back to Demos.
var menuRound = []struct{ page, steps string }{
	{"Calib", "u u a u a f"}, // A, up to "This pose", A: a calibration started afresh
	{"Demos", "u u u u u u a f b d a f b d a f b d a f b d a f b d a b b d d d a"},
	{"Dash", "d d u u"},
	{"Gauge", "u u u a u a"},
	{"Ctrl", "u u u u u u d a u a"},
	{"Image", "u u u a u a d a u a"},
	{"Flow", "d d"},
	{"List", "u u u a d d d d b"},
	{"Log", "u u"},
	{"Life", "u u u u d d a f b"},
	{"Water", "u u u u u u d d d a f b"},
	{"Fire", "u u u u u u d d d a f b"},
	{"Pong", "u u u d a f b"},
	{"Boids", "u u u u u u d a d d d a f b"}, // Gather, then Full
	{"Motion", "u u a f b"},
}

func playRound(t *testing.T, a *App, frame func()) {
	t.Helper()
	keys := map[string]Button{"u": BtnUp, "d": BtnDown, "a": BtnA, "b": BtnB, "c": BtnC}
	for i, pg := range menuRound {
		if got := a.page().name; got != pg.page {
			t.Fatalf("round step %d: on page %s, want %s", i, got, pg.page)
		}
		for _, k := range strings.Fields(pg.steps + " c") {
			if k == "f" {
				for range 3 {
					frame()
				}
				continue
			}
			a.Press(keys[k], false)
			frame()
		}
	}
}

// Owner hard rule (TG-2 #12): no allocations after setup. Every page is
// built in setup (Pages.BuildAll) and the dialogs too, so a whole menu round
// — every page with its usual interactions, the Demos tiles full screen, the
// speed dropdown, the options dialog — allocates nothing: exactly 0 under
// TinyGo (`tinygo test -gc=leaking -run MenuRound ./app`), fewer than one per
// press on Go (its runtime grows type-assertion caches at random).
func TestMenuRoundAllocatesNothing(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	if n := a.pages.Built(); n != a.pages.Count() {
		t.Fatalf("%d of %d pages built in setup", n, a.pages.Count())
	}
	playRound(t, a, frame) // first draws size their buffers
	// the round split into presses before measuring (splitting allocates)
	keys := map[string]Button{"u": BtnUp, "d": BtnDown, "a": BtnA, "b": BtnB, "c": BtnC}
	var presses []string
	for _, pg := range menuRound {
		presses = append(presses, strings.Fields(pg.steps+" c")...)
	}
	round := func() {
		for _, k := range presses {
			if k == "f" {
				for range 3 {
					frame()
				}
				continue
			}
			a.Press(keys[k], false)
			frame()
		}
	}
	var ms runtime.MemStats
	runtime.ReadMemStats(&ms)
	m0 := ms.Mallocs
	for range 3 {
		round()
	}
	runtime.ReadMemStats(&ms)
	n := ms.Mallocs - m0
	if a.page().name != "Calib" {
		t.Fatalf("the rounds ended on %s, want Calib", a.page().name)
	}
	if runtime.Compiler != "gc" && n != 0 || n/uint64(3*len(presses)) != 0 {
		t.Fatalf("%d allocations in 3 menu rounds (%d presses)", n, 3*len(presses))
	}
}

// Moving the focus on a page (navigator, its observer, the log line it
// writes) allocates nothing: a press doesn't feed the GC.
func TestMovingFocusDoesNotAllocate(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	a.showPage(a.pageIndex("Ctrl")) // a column of fields
	frame()
	if n := testing.AllocsPerRun(30, func() {
		a.Press(BtnDown, false)
		a.Press(BtnDown, false)
		a.Press(BtnUp, false)
		a.Press(BtnUp, false)
	}); n != 0 {
		t.Fatalf("%.0f allocations moving the focus 4 times", n)
	}
}

// Scrolling a list (focus moves that scroll the ScrollChoice into view)
// allocates nothing either.
func TestScrollingAListDoesNotAllocate(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	a.showPage(a.pageIndex("List"))
	a.Press(BtnA, false) // into the list
	frame()
	if n := testing.AllocsPerRun(10, func() {
		for range 12 {
			a.Press(BtnDown, false)
		}
		for range 12 {
			a.Press(BtnUp, false)
		}
	}); n != 0 {
		t.Fatalf("%.0f allocations scrolling the list down and up", n)
	}
}
