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
