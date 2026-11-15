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
	if name := a.page().name; name != "Boids" {
		t.Fatalf("after the gopher the page is %q, want Boids", name)
	}
}

// Any button skips the gopher and does nothing else; a console command
// skips it and still runs.
func TestAButtonOrACommandSkipsTheGopher(t *testing.T) {
	a, now := bootApp()
	a.Update(now)
	start := a.cur
	a.Press(BtnC, false) // C: would change the page
	a.Update(now.Add(100 * time.Millisecond))
	if a.boot.on || a.cur != start {
		t.Fatalf("after C: gopher on %v, page %q; want skipped, still Boids", a.boot.on, a.page().name)
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

// After the gopher the app opens on Boids, and the Calibration page
// measures only while it is open: 10 s of a still badge on Boids feed the
// calibrator nothing (no measurement, no gyro bias set); opened, it is fed.
func TestTheAppOpensOnBoidsAndCalibratesOnlyWhenAsked(t *testing.T) {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	now := time.Now()
	a := New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id, IMU: true}, now)
	c := &scriptedCal{}
	a.calib.c = c
	for i := range 10 * 30 {
		a.SetIMU([3]float32{0, gravity, 0}, [3]float32{}, 1.0/30)
		a.Update(now.Add(time.Duration(i) * time.Second / 30))
	}
	if name := a.page().name; name != "Boids" {
		t.Fatalf("after the gopher the app is on %q, want Boids", name)
	}
	if c.n != 0 || a.calib.gyroSet {
		t.Fatalf("on Boids the calibrator got %d samples (gyro bias set: %v), want none", c.n, a.calib.gyroSet)
	}
	a.showPage(a.calibPage)
	a.SetIMU([3]float32{0, gravity, 0}, [3]float32{}, 1.0/30)
	if c.n == 0 {
		t.Fatal("the Calibration page opened, yet the calibrator got no sample")
	}
}
