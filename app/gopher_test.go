package app

import (
	"testing"
	"time"

	"github.com/itohio/tinygui/examples/framebuffer"
)

func bootApp() (*App, time.Time) {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	now := time.Now()
	return New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id}, now), now
}

// At power-up the gopher plays and then the first page shows, within the
// budget: on at the first frame, still on at 1.9 s, gone by 2.1 s.
func TestTheGopherPlaysThenTheFirstPageShows(t *testing.T) {
	a, now := bootApp()
	a.Update(now)
	if !a.boot.on {
		t.Fatal("no gopher at power-up")
	}
	for ms := 0; ms <= 2100; ms += 50 {
		a.Update(now.Add(time.Duration(ms) * time.Millisecond))
		a.Draw()
		if on := a.boot.on; ms <= 1900 && !on || ms >= 2100 && on {
			t.Fatalf("at %d ms the gopher is on: %v (want on to 2 s, then the first page)", ms, on)
		}
	}
	if a.cur != 0 {
		t.Fatalf("after the gopher the page is %d, want the first (0)", a.cur)
	}
}

// Any button skips the gopher and does nothing else; a console command
// skips it and still runs.
func TestAButtonOrACommandSkipsTheGopher(t *testing.T) {
	a, now := bootApp()
	a.Update(now)
	a.Press(BtnC, false) // C: would change the page
	a.Update(now.Add(100 * time.Millisecond))
	if a.boot.on || a.cur != 0 {
		t.Fatalf("after C: gopher on %v, page %d; want skipped, still the first page", a.boot.on, a.cur)
	}
	b, now := bootApp()
	b.Update(now)
	b.Command("page", "2")
	b.Update(now.Add(100 * time.Millisecond))
	if b.boot.on || b.cur != 2 {
		t.Fatalf("after `page 2`: gopher on %v, page %d; want skipped and page 2", b.boot.on, b.cur)
	}
}

// The gopher's frames draw without allocating.
func TestTheGopherDrawsWithoutAllocating(t *testing.T) {
	a, now := bootApp()
	a.Update(now.Add(time.Second)) // waving
	if n := testing.AllocsPerRun(5, a.Draw); n != 0 {
		t.Fatalf("a gopher frame allocates %.0f times, want 0", n)
	}
}
