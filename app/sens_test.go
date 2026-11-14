package app

import (
	"strings"
	"testing"
)

// moveUp drives a d-metre stroke up the badge's own up axis (upright: world
// up) over 1 s, then 1 s at rest, ticking Pong once a frame; it returns how
// far your paddle and the boids' window moved up the screen, in px.
func moveUp(a *App, tm *float64, d float64) (paddlePx, windowPx float64) {
	p, b := a.pong, a.boids
	pad0, y0 := p.pad[0], b.pose.baseY
	drive(a, tm, 2*imuRate, func(float64) float64 { return 0 }, stroke([3]float64{0, 0, 1}, d, *tm+0.1, 1), func(i int) {
		if i%3 == 2 {
			p.tick()
		}
	})
	return float64(pad0-p.pad[0]) / pongFix * fullCellPx, float64(y0-b.pose.baseY) / boidsFix * fullCellPx
}

// One shared motion sensitivity moves Pong's paddle and the boids' window
// (owner 2026-11-14, TG-6 #6: he picked the badge up and the flock
// vanished; 1:1 on the glass a 10 cm move was 785 px): by default a 10 cm
// move is 10 px; at 2.5 mm per px, 40 px.
func TestMotionSensitivityMapsTheBadgesMoveToPixels(t *testing.T) {
	for _, c := range []struct {
		choice int
		want   float64 // px for 10 cm
	}{{sensDefault, 10}, {0, 40}} {
		a, tm := motionApp()
		if c.choice != sensDefault {
			a.setSens(c.choice)
		}
		a.pong.showFull()
		a.pong.fullPress(BtnA) // paused: only the badge moves the paddle
		pad, win := moveUp(a, tm, 0.10)
		name := sensLabels[c.choice] + " per px"
		if pad < c.want*0.85 || pad > c.want*1.15 {
			t.Errorf("%s: a 10 cm move took the paddle %.1f px, want %.0f (±15 %%)", name, pad, c.want)
		}
		if win < c.want*0.85 || win > c.want*1.15 {
			t.Errorf("%s: a 10 cm move took the boids' window %.1f px, want %.0f (±15 %%)", name, win, c.want)
		}
	}
}

// Changing the sensitivity moves neither the paddle nor the window: only
// what follows moves at the new scale (5 cm more at 2.5 mm per px: 20 px).
func TestChangingTheSensitivityMovesNothing(t *testing.T) {
	a, tm := motionApp()
	p, b := a.pong, a.boids
	p.showFull()
	p.fullPress(BtnA)
	moveUp(a, tm, 0.10)
	pad, base := p.pad[0], b.pose.baseY
	a.setSens(0)
	drive(a, tm, imuRate/2, func(float64) float64 { return 0 }, noAcc, func(i int) {
		if i%3 == 2 {
			p.tick()
		}
	})
	if p.pad[0] != pad || b.pose.baseY != base {
		t.Fatalf("the change moved the paddle %.1f px and the window %.1f px, want 0",
			float64(pad-p.pad[0])/pongFix*fullCellPx, float64(base-b.pose.baseY)/boidsFix*fullCellPx)
	}
	padPx, winPx := moveUp(a, tm, 0.05)
	if padPx < 17 || padPx > 23 || winPx < 17 || winPx > 23 {
		t.Errorf("5 cm more at 2.5 mm per px: the paddle %.1f px, the window %.1f px, want 20 (±15 %%)", padPx, winPx)
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
