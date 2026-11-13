package app

import (
	"runtime"
	"time"
)

// Health monitoring (owner: "make it robust, monitor heap and ram
// consumption ... ui freezes for a split second"). The firmware reports each
// frame's time; `frames` prints a histogram since the last call, `mem` the
// live heap after a collection, `memshow 1` puts heap use in the footer.

// frame-time buckets, upper bounds in ms (the last one is open)
var frameBuckets = [...]int{40, 60, 80, 120, 200, 500}

type health struct {
	hist       [len(frameBuckets) + 1]int
	maxUS      int64
	frames     int
	gc0        uint32
	memShow    bool
	heapUsedKB int // refreshed once a second for the footer (counts garbage)
	heapSysKB  int
	ms         runtime.MemStats
}

// ReportFrame records the time one frame took (update + draw + push).
func (a *App) ReportFrame(d time.Duration) {
	h := &a.health
	us := d.Microseconds()
	ms := int(us / 1000)
	i := 0
	for i < len(frameBuckets) && ms >= frameBuckets[i] {
		i++
	}
	h.hist[i]++
	h.frames++
	h.maxUS = max(h.maxUS, us)
}

// sampleHeap refreshes the footer's heap figures (once a second, from Update).
func (a *App) sampleHeap() {
	h := &a.health
	runtime.ReadMemStats(&h.ms)
	h.heapUsedKB = int(h.ms.HeapAlloc / 1024)
	h.heapSysKB = int(h.ms.HeapSys / 1024)
}

// framesReport logs the frame-time histogram and GCs since the last call
// (on short lines: log lines are at most 48 bytes), then starts a new period.
func (a *App) framesReport() {
	h := &a.health
	runtime.ReadMemStats(&h.ms)
	a.log(a.txtStat.reset().str("frames ").int(h.frames).str(" max_ms ").int(int(h.maxUS / 1000)).
		str(" gc ").int(int(h.ms.NumGC - h.gc0)).String())
	t := a.txtStat.reset().str("ms")
	lo := 0
	for i, n := range h.hist {
		if i == 3 {
			a.log(t.String())
			t = a.txtStat.reset().str("ms")
		}
		t.str(" ").int(lo)
		if i < len(frameBuckets) {
			lo = frameBuckets[i]
			t.str("-").int(lo)
		} else {
			t.str("+")
		}
		t.str(":").int(n)
	}
	a.log(t.String())
	a.health = health{gc0: h.ms.NumGC, memShow: h.memShow, heapUsedKB: h.heapUsedKB, heapSysKB: h.heapSysKB}
}

// memReport collects garbage and logs the live heap and the free room after
// the collection (the headroom that matters: heap_alloc between collections
// counts garbage the next collection frees).
func (a *App) memReport() {
	h := &a.health
	runtime.GC()
	runtime.ReadMemStats(&h.ms)
	live, sys := int(h.ms.HeapAlloc/1024), int(h.ms.HeapSys/1024)
	a.log(a.txtStat.reset().str("mem live_kb ").int(live).str(" free_kb ").int(sys - live).
		str(" heap_kb ").int(sys).str(" gc ").int(int(h.ms.NumGC)).String())
	// every allocation since boot: idle growth means something outside the
	// frame path allocates (the frame allocs line counts only the frame)
	a.log(a.txtStat.reset().str("mallocs ").int(int(h.ms.Mallocs)).str(" frees ").int(int(h.ms.Frees)).String())
}
