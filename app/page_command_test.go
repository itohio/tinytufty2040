package app

import (
	"strings"
	"testing"
)

// The console's page takes an index or a page's name (any case); anything
// else leaves the page where it is and says what it takes. It used to read
// a bare `page` or a name as page 0, so a misread command jumped there
// silently.
func TestPageCommandTakesAnIndexOrAName(t *testing.T) {
	a := testApp()
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, s) }
	run := func(arg string) string {
		lines = lines[:0]
		a.Command("page", arg)
		a.Update(a.now)
		return strings.Join(lines, "\n")
	}
	pong := a.pageIndex("Pong")
	run("4")
	if a.cur != 4 {
		t.Fatalf("page 4: on page %d", a.cur)
	}
	run("pong")
	if a.cur != pong {
		t.Fatalf("page pong: on page %d (%s), want %d (Pong)", a.cur, a.page().name, pong)
	}
	for _, arg := range []string{"", "Nowhere", "99"} {
		out := run(arg)
		if a.cur != pong {
			t.Fatalf("page %q: moved to page %d (%s), want it to stay on Pong", arg, a.cur, a.page().name)
		}
		if !strings.Contains(out, "page: a number 0-") {
			t.Fatalf("page %q: said %q, want what page takes", arg, out)
		}
	}
}
