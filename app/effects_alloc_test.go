package app

import (
	"runtime"
	"testing"
)

func mallocCount() uint64 {
	var ms runtime.MemStats
	runtime.ReadMemStats(&ms)
	return ms.Mallocs
}

var countProbe []*[16]byte

// needMallocCount skips an allocation test on a runtime that counts no
// allocations: TinyGo's default host GC (Boehm) reports Mallocs = 0, which
// would pass every check (review R-TG-1). Run TinyGo with `tinygo test
// -gc=leaking`, which counts each one, as the badge's GC does.
func needMallocCount(t *testing.T) {
	t.Helper()
	m0 := mallocCount()
	countProbe = append(countProbe, new([16]byte))
	if mallocCount() == m0 {
		t.Skip("this runtime counts no allocations (TinyGo's host Boehm GC): run tinygo test -gc=leaking")
	}
}

// The demo effects (Life, water, fire, Pong, boids, Motion; their
// pages and full-screen views) run without allocating after setup: owner,
// "the tinygui should be ABLE to run without GC at all … all the demo
// effects should run without allocation too". Exact under TinyGo
// (`tinygo test -gc=leaking -run Effects ./app`, and the default GC); on Go
// fewer than one per frame (Go's runtime grows its type-assertion caches
// at random).
func TestEffectsRunWithoutAllocating(t *testing.T) {
	needMallocCount(t)
	a, frame := framedApp()
	check := func(name string) {
		for range 40 { // warm up: first draws, lazily sized buffers
			frame()
		}
		m0 := mallocCount()
		for range 30 {
			frame()
		}
		n := mallocCount() - m0
		if runtime.Compiler != "gc" && n != 0 || n/30 != 0 {
			t.Errorf("%s: %d allocations in 30 frames", name, n)
		}
	}
	fulls := map[string]func(){
		"Life": func() { a.life.fullOn = true }, "Water": func() { a.water.fullOn = true },
		"Fire": func() { a.fire.fullOn = true }, "Pong": func() { a.pong.showFull() },
		"Boids": func() { a.boids.fullOn = true }, "Motion": func() { a.motionDemo.fullOn = true },
	}
	for i := range a.pages.Count() {
		name := a.pages.Name(i)
		on, ok := fulls[name]
		if !ok && name != "Demos" {
			continue // not an effect page
		}
		a.showPage(i)
		check(name)
		if ok {
			on()
			check(name + " full screen")
			a.Press(BtnB, false)
		}
	}
	a.showPage(a.pageIndex("Demos")) // the Demos menu running every effect in its tile, one full screen
	a.Press(BtnA, false)
	check("Demos tile full screen")
}
