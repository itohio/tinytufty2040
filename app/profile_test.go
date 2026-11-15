package app

import (
	"strings"
	"testing"
	"time"
)

// The first frame after boot has no previous frame: it adds no period, so
// the first stats report after a boot reads the real start-to-start time
// (the badge printed period_us and other_us as garbage in every first report
// after a reboot: the board timed the first frame from the zero time, and
// the 32-bit int cut the huge average).
func TestProfileFirstFrameAfterBootAddsNoPeriod(t *testing.T) {
	a := testApp()
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, s) }
	t0 := time.Date(2026, 11, 15, 9, 53, 0, 0, time.UTC)
	ms := time.Millisecond
	for i := range 10 {
		a.ReportProfile(t0.Add(time.Duration(i)*50*ms), 6*ms, 2*ms, 1, 2*ms, 30*ms, 10*ms)
	}
	a.profileReport()
	var period, other string
	for _, l := range lines {
		if f := strings.Fields(l); len(f) > 2 && f[0] == "prof" {
			switch f[1] {
			case "period_us":
				period = f[2]
			case "draw_us":
				other = f[6]
			}
		}
	}
	if period != "50000" || other != "2000" {
		t.Fatalf("the first report after boot: period_us %s, other_us %s; want 50000 and 2000 (50 ms frames, 48 ms of parts)", period, other)
	}
}
