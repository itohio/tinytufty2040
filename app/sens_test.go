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

// The motion sensitivity moves the boids' window (owner 2026-11-14, TG-6
// #6: he picked the badge up and the flock vanished; 1:1 on the glass a 10
// cm move was 785 px): by default a 10 cm move is 10 px; at 2.5 mm per px,
// 40 px.
func TestMotionSensitivityMapsTheBadgesMoveToPixels(t *testing.T) {
	for _, c := range []struct {
		choice int
		want   float64 // px for 10 cm
	}{{sensDefault, 10}, {0, 40}} {
		a, tm := motionApp()
		if c.choice != sensDefault {
			a.setSens(c.choice)
		}
		win := moveUp(a, tm, 0.10)
		if name := sensLabels[c.choice] + " per px"; win < c.want*0.85 || win > c.want*1.15 {
			t.Errorf("%s: a 10 cm move took the boids' window %.1f px, want %.0f (±15 %%)", name, win, c.want)
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
	a.setSens(0)
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
	if out := run("5"); a.sens != 1 || !strings.Contains(out, "sens: 1 px = 5 mm") {
		t.Fatalf("sens 5: choice %d, said %q; want 5 mm per px", a.sens, out)
	}
	if out := run("3"); a.sens != 1 || !strings.Contains(out, "one of 2.5 5 10 20 40 (1 px = that many mm)") {
		t.Fatalf("sens 3: choice %d, said %q; want refused, still 5 mm", a.sens, out)
	}
	if out := run(""); !strings.Contains(out, "sens: 1 px = 5 mm") {
		t.Fatalf("sens: said %q, want the current 5", out)
	}
}
