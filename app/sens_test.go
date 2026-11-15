package app

import (
	"strings"
	"testing"
)

// moveUp drives a d-metre stroke up the badge's own up axis (upright: world
// up) over 1 s, then 1 s at rest; it returns how far the boids' window moved
// up the screen, in px.
func moveUp(a *App, tm *float64, d float64) (windowPx float64) {
	b := a.boids
	y0 := b.pose.baseY
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke([3]float64{0, 0, 1}, d, *tm+0.1, 1), nil)
	return float64(y0-b.pose.baseY) / boidsFix * fullCellPx
}

// The motion sensitivity moves the boids' window: by default a 10 cm move
// is 10 px; at 2.5 mm per px, 40 px; at 1:1 the window moves with the badge
// on the glass, 1 cm = 78 px (320 px across 40.8 mm).
func TestMotionSensitivityMapsTheBadgesMoveToPixels(t *testing.T) {
	for _, c := range []struct {
		arg  string  // the console's choice
		d    float64 // m moved
		want float64 // px
	}{{sensArgs[sensDefault], 0.10, 10}, {"2.5", 0.10, 40}, {"1:1", 0.01, 0.01 / glassMM * 1000}} {
		i := sensArg(c.arg)
		if i < 0 {
			t.Errorf("no %s choice", c.arg)
			continue
		}
		a, tm := motionApp()
		a.setSens(i)
		win := moveUp(a, tm, c.d)
		if win < c.want*0.85 || win > c.want*1.15 {
			t.Errorf("sens %s: a %.0f cm move took the boids' window %.1f px, want %.0f (±15 %%)", c.arg, c.d*100, win, c.want)
		}
	}
}

// Changing the sensitivity doesn't move the window: only what follows moves
// at the new scale (5 cm more at 2.5 mm per px: 20 px).
func TestChangingTheSensitivityMovesNothing(t *testing.T) {
	a, tm := motionApp()
	b := a.boids
	moveUp(a, tm, 0.10)
	base := b.pose.baseY
	a.setSens(sensArg("2.5"))
	drive(a, tm, imuRate/2, func(float64) float64 { return 0 }, noAcc, nil)
	if b.pose.baseY != base {
		t.Fatalf("the change moved the window %.1f px, want 0", float64(base-b.pose.baseY)/boidsFix*fullCellPx)
	}
	if winPx := moveUp(a, tm, 0.05); winPx < 17 || winPx > 23 {
		t.Errorf("5 cm more at 2.5 mm per px: the window %.1f px, want 20 (±15 %%)", winPx)
	}
}

// The console sets it by its value in mm per px; anything else is refused
// with the choices, and the setting stays.
func TestSensCommand(t *testing.T) {
	a := testApp()
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, s) }
	run := func(arg string) string {
		lines = lines[:0]
		a.Command("sens", arg)
		a.Update(a.now)
		return strings.Join(lines, "\n")
	}
	five := sensArg("5")
	if out := run("5"); a.sens != five || !strings.Contains(out, "sens: 1 px = 5 mm") {
		t.Fatalf("sens 5: choice %d, said %q; want 5 mm per px", a.sens, out)
	}
	if out := run("3"); a.sens != five || !strings.Contains(out, "one of 1:1 2.5 5 10 20 40") {
		t.Fatalf("sens 3: choice %d, said %q; want refused, still 5 mm", a.sens, out)
	}
	if out := run(""); !strings.Contains(out, "sens: 1 px = 5 mm") {
		t.Fatalf("sens: said %q, want the current 5", out)
	}
}
