package app

import "math"

// tilter is a demo that follows the badge's tilt from a motion sensor:
// gravity in screen axes (x right, y down, m/s²), at full precision.
type tilter interface{ tilt(gx, gy float32) }

var tiltLog = func() (l [8]string) {
	for i, n := range gravityNames {
		l[i] = "tilt: gravity " + n
	}
	return
}()

// minTilt is the gravity (m/s²) in the screen's plane below which the badge
// counts as lying flat: the demos keep the last direction.
const minTilt = 3

// SetTilt feeds gravity from a motion sensor, in screen axes (x right, y
// down, m/s²: the direction things fall), used at full precision: the
// liquids take it as a continuous vector, the boids' box its exact roll.
// Call once per frame before Update. Without a sensor the buttons set gravity.
func (a *App) SetTilt(gx, gy float32) {
	if gx*gx+gy*gy < minTilt*minTilt {
		return // flat: in-plane gravity too small to tell a direction
	}
	a.tiltX, a.tiltY, a.tilted = gx, gy, true
	if d := nearestGravity(gx, gy, a.tiltDir); d != a.tiltDir {
		a.tiltDir = d
		a.log(tiltLog[d]) // on the console and the Log page: roughly where it points
	}
	for _, t := range a.tilters {
		t.tilt(gx, gy)
	}
}

// nearestGravity is the direction of gravityVec closest to (gx, gy); the
// current one wins unless another is clearly closer (by 10 %).
func nearestGravity(gx, gy float32, cur int) int {
	best, bestScore := cur, gravityScore(cur, gx, gy)*1.1
	for i := range gravityVec {
		if s := gravityScore(i, gx, gy); s > bestScore {
			best, bestScore = i, s
		}
	}
	return best
}

// gravityScore is how well direction i matches (gx, gy). A plain function,
// not a closure over gx, gy: TinyGo put that closure on the heap each frame.
func gravityScore(i int, gx, gy float32) float32 {
	v := gravityVec[i]
	s := gx*float32(v[0]) + gy*float32(v[1])
	if v[0] != 0 && v[1] != 0 {
		s *= 0.70710678 // diagonals have length √2
	}
	return s
}

func (w *water) tilt(gx, gy float32) {
	if !w.pinned {
		w.setTilt(gx, gy)
	}
}

func (f *fire) tilt(gx, gy float32) {
	if !f.pinned {
		f.setTilt(gx, gy)
	}
}

// tilt turns the boids' box by the badge's roll (the walls move, the flock
// keeps its world heading): exactly, from the shortest way round.
func (b *boids) tilt(gx, gy float32) {
	roll := float32(math.Atan2(float64(gx), float64(gy)))
	d := roll - b.pose.angle
	d -= 2 * math.Pi * float32(math.Round(float64(d/(2*math.Pi))))
	b.pose.setAngle(b.pose.angle + d)
}
