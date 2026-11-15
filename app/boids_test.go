package app

import (
	"math"
	"testing"
)

// The badge is the camera: turned 30° to the right about its own up axis
// (the gyro reading it, the flock's own motion frozen), a boid fixed where
// the view pointed shows left of centre at the projected 30°: x = −tan 30°
// in half widths at a 90° field, level.
func TestTurningTheBadgeTurnsTheView(t *testing.T) {
	a, tm := motionApp()
	b := a.boids
	b.motion = a.motion
	b.fov = 5 // 90°
	b.camera()
	b.set(0, floatQ(b.look), b.vel(0)) // a boid straight ahead
	const turn, T, dt = 30 * math.Pi / 180, 1.0, 1.0 / imuRate
	// turn at the rate whose integral is exactly 30°: smooth(u) over T
	for i := 1; float64(i)*dt <= T+0.5; i++ {
		t0 := float64(i-1) * dt / T
		t1 := float64(i) * dt / T
		rate := turn * (smooth(t1) - smooth(t0)) / dt
		imuFrame(a, [3]float32{0, gravity, 0}, [3]float32{0, float32(-rate), 0}, dt) // −Y turns the view to the right
		*tm += dt
	}
	b.camera()
	xq, yq, ok := b.project(wide(b.p[0]))
	x, y := float64(xq)/boidsScrQ, float64(yq)/boidsScrQ
	want := -math.Tan(turn)
	if !ok || math.Abs(x-want) > 0.03 || math.Abs(y) > 0.03 {
		t.Fatalf("turned 30° right: the boid ahead before shows at (%.3f, %.3f) (in view %v), want (%.3f, 0): left by the projected 30°", x, y, ok, want)
	}
}

// A boid outside the view comes back to be seen: placed 90° off the view's
// direction and flying away, it is in view within 10 s.
func TestABoidOutsideTheViewComesBack(t *testing.T) {
	b := newBoids()
	b.count = 0                                                // 32
	b.set(0, [3]float32{1, 0, 0}, [3]float32{0, boidsVMax, 0}) // 90° to the right of the view (it looks along −Y), flying away: toward +Y
	for i := range 10 * 30 {
		b.tick()
		if b.inCone(wide(b.p[0])) {
			if i == 0 {
				t.Fatal("the boid started in view")
			}
			return
		}
	}
	x, y, ok := b.project(wide(b.p[0]))
	t.Fatalf("after 10 s the boid is still out of view: at (%.2f, %.2f) half widths (ahead %v)", float64(x)/boidsScrQ, float64(y)/boidsScrQ, ok)
}

// The flock stays on the sphere: after 20 s with 128 boids every position is
// a unit vector, every velocity tangent to it and within the speed limits.
func TestBoidsStayOnTheSphere(t *testing.T) {
	b := newBoids()
	b.count = 2 // 128
	for range 20 * 30 {
		b.tick()
	}
	for i := range boidsN[b.count] {
		p, v := b.pos(i), b.vel(i)
		n := math.Sqrt(float64(dot3(p, p)))
		sp := math.Sqrt(float64(dot3(v, v)))
		if math.Abs(n-1) > 2e-4 || math.Abs(float64(dot3(v, p))) > 2e-4 || sp < boidsVMin*0.99 || sp > boidsVMax*1.01 {
			t.Fatalf("boid %d: |p| %.6f, v·p %.2g, speed %.3f rad/s (want 1, 0, %.2f–%.2f; Q14 rounding: 2e-4)", i, n, dot3(v, p), sp, boidsVMin, boidsVMax)
		}
	}
}

// A boid's head is coloured by its heading on screen, in octants: east 0,
// then clockwise (screen y grows down).
func TestOctantFollowsHeading(t *testing.T) {
	for _, c := range []struct {
		dx, dy float32
		want   uint8
	}{{1, 0, 0}, {1, 1, 1}, {0, 1, 2}, {-1, 1, 3}, {-1, 0, 4}, {-1, -1, 5}, {0, -1, 6}, {1, -1, 7}} {
		if got := octant(c.dx, c.dy); got != c.want {
			t.Errorf("octant(%v, %v) = %d, want %d", c.dx, c.dy, got, c.want)
		}
	}
}

// In the full view UP and DOWN change the field of view in 10° steps between
// 40° and 150° (110° at first), and its readout shows at once, fades, and is
// gone 1.5 s after the last change.
func TestFullViewChangesTheFieldOfView(t *testing.T) {
	b := newBoids()
	if boidsFOVs[b.fov] != 110 {
		t.Fatalf("the field of view starts at %v°, want 110°", boidsFOVs[b.fov])
	}
	if _, ok := b.fovReadout(0); ok {
		t.Fatal("a readout before any change")
	}
	for range 10 {
		b.fullPress(BtnUp)
	}
	if boidsFOVs[b.fov] != 150 {
		t.Fatalf("ten UPs: %v°, want 150° (the widest)", boidsFOVs[b.fov])
	}
	for range 20 {
		b.fullPress(BtnDown)
	}
	if boidsFOVs[b.fov] != 40 {
		t.Fatalf("twenty DOWNs: %v°, want 40° (the narrowest)", boidsFOVs[b.fov])
	}
	b.fullPress(BtnUp)
	if boidsFOVs[b.fov] != 50 {
		t.Fatalf("UP from 40°: %v°, want 50°", boidsFOVs[b.fov])
	}
	if c, ok := b.fovReadout(0); !ok || c != colFG {
		t.Fatalf("just changed: readout %v (shown %v), want the text colour", c, ok)
	}
	if c, ok := b.fovReadout(boidsFOVHold + boidsFOVFade/2); !ok || c == colFG || c == boidsViewBG {
		t.Fatalf("half faded: readout %v (shown %v), want between the text and the background", c, ok)
	}
	if _, ok := b.fovReadout(boidsFOVHold + boidsFOVFade); ok {
		t.Fatal("1.5 s after the change the readout still shows")
	}
}

// Scatter sends the flock off the screen and it comes back to be seen: at a
// 60°, 110° and 150° view, gathered and settled, then scattered, at least
// 80 % of 64 and of 128 boids are off the screen at some moment while
// Scatter lasts, and at least 95 % are back in the view's cone within the
// same time again and 1.5 s to turn round. (Leaving the cone isn't enough:
// it spans the screen's height, so a boid leaving it sideways is still on
// the screen's rim.)
func TestScatterLeavesTheViewAndComesBack(t *testing.T) {
	for _, fov := range []int{2, 7, 11} { // 60°, 110°, 150°
		for _, c := range []int{1, 2} { // 64, 128
			b := newBoids()
			b.count, b.fov = c, fov
			b.gather()
			for range 60 {
				b.tick()
			}
			b.scatter()
			free := b.free
			n := boidsN[c]
			maxOut, backAt := 0, -1
			for i := 1; i <= 2*free+45; i++ {
				b.tick()
				off, out := 0, 0
				for j := range n {
					if !onScreen(b, wide(b.p[j])) {
						off++
					}
					if !b.inCone(wide(b.p[j])) {
						out++
					}
				}
				if i <= free {
					maxOut = max(maxOut, off)
				} else if backAt < 0 && out*20 <= n {
					backAt = i - free
				}
			}
			if maxOut*5 < n*4 || backAt < 0 {
				t.Errorf("view %v°, %d boids: at most %d%% off the screen while Scatter lasts (%.1f s; want ≥ 80), back (≥ 95%% in) %.1f s after (want within %.1f s; < 0: never)",
					boidsFOVs[fov], n, maxOut*100/n, float64(free)/30, float64(backAt)/30, float64(free+45)/30)
			}
		}
	}
}

// A boid with no neighbours and no pull flies a great circle: at the
// slowest speed, from 12 headings, after 6 s it is on average less than 0.5°
// off the great circle it started on (the step's rounding wanders about
// 0.2°). Integer rounding that always went one way bent every path: 1.0°.
func TestALoneBoidFliesAGreatCircle(t *testing.T) {
	ko, bs := kOutQ, bandSafety
	kOutQ, bandSafety = 0, 0 // no seek, no border band: free flight
	defer func() { kOutQ, bandSafety = ko, bs }()
	p0 := unit3([3]float32{0.3, -0.9, 0.2})
	e1 := unit3(tangent([3]float32{0, 0, 1}, p0))
	e2 := cross3(p0, e1)
	var off float64
	for k := range 12 {
		b := newBoids()
		b.count = 0
		for i := 1; i < boidsMax; i++ { // the others on the far side: no neighbours within 6 s
			b.set(i, unit3([3]float32{-1, float32(i%7) * 0.01, float32(i%5) * 0.01}), [3]float32{})
		}
		s, c := math.Sincos(float64(k) * math.Pi / 6)
		var v [3]float32
		for a := range 3 {
			v[a] = (float32(c)*e1[a] + float32(s)*e2[a]) * boidsVMin
		}
		b.set(0, p0, v)
		n0 := unit3(cross3(b.pos(0), b.vel(0)))
		for range 6 * 30 {
			b.step()
		}
		off += math.Abs(float64(dot3(b.pos(0), n0))) * 180 / math.Pi / 12
	}
	if off > 0.5 {
		t.Errorf("a lone boid ends on average %.2f° off its great circle after 6 s, want < 0.5", off)
	}
}

func cross3(a, b [3]float32) [3]float32 {
	return [3]float32{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}

// Inside the view the flock keeps off its border: over 60 s, from four
// gathers, boids in the view's cone spend under 3 % of their time in the
// outer 10 % of its angle.
func TestBoidsKeepOffTheBorderInsideTheView(t *testing.T) {
	for seed := uint32(1); seed <= 4; seed++ {
		rngState = 2463534242 + seed*7919
		b := newBoids()
		b.gather()
		r := math.Acos(float64(b.cosView) / boidsQ)
		edge, seen := 0, 0
		for k := range 60 * 30 {
			b.tick()
			if k < 150 { // the gather's first 5 s
				continue
			}
			for i := range boidsN[b.count] {
				p := wide(b.p[i])
				if !b.inCone(p) {
					continue
				}
				seen++
				if math.Acos(math.Min(1, float64(dotQ(p, b.look))/boidsQ)) > 0.9*r {
					edge++
				}
			}
		}
		if share := float64(edge) * 100 / float64(seen); share >= 3 {
			t.Errorf("gather %d: boids in the view spent %.1f%% of their time in the outer 10%% of its angle, want < 3", seed, share)
		}
	}
}

// The distance between two boids is their angle on the sphere, whatever
// their place and the direction between them: pairs 5° apart at 200 random
// places and bearings measure the same to 0.01°, as do pairs at 2° and 8°.
func TestDistanceIsTheAngleOnTheSphere(t *testing.T) {
	rngState = 99
	for _, deg := range []float64{2, 5, 8} {
		lo, hi := math.Inf(1), math.Inf(-1)
		for range 200 {
			p := unit3([3]float32{randIn(), randIn(), randIn()})
			d := unit3(tangent([3]float32{randIn(), randIn(), randIn()}, p))
			s, c := math.Sincos(deg * math.Pi / 180)
			var q [3]float32
			for a := range 3 {
				q[a] = float32(c)*p[a] + float32(s)*d[a]
			}
			got := math.Acos(1-float64(pairGap(toQ14(p), toQ14(q)))/(boidsQ*boidsQ)) * 180 / math.Pi
			lo, hi = math.Min(lo, got), math.Max(hi, got)
		}
		if lo < deg-0.01 || hi > deg+0.01 {
			t.Errorf("pairs %.0f° apart measured %.3f–%.3f°, want %.0f° everywhere (±0.01)", deg, lo, hi, deg)
		}
	}
}

// A lone boid flies the same in every direction: from 24 headings 15° apart,
// with no neighbours, no seek and no border band, after 2 s its speed is the
// same to 1 % and it is within 0.3° of its great circle (a per-axis
// magnitude, clamp or length shortcut would make some headings slower or
// bend them).
func TestALoneBoidFliesTheSameInEveryDirection(t *testing.T) {
	ko, bs := kOutQ, bandSafety
	kOutQ, bandSafety = 0, 0
	defer func() { kOutQ, bandSafety = ko, bs }()
	p0 := unit3([3]float32{0.3, -0.9, 0.2})
	e1 := unit3(tangent([3]float32{0, 0, 1}, p0))
	e2 := cross3(p0, e1)
	for _, sp := range []float32{boidsVMin, boidsVMax} {
		lo, hi, off := math.Inf(1), 0.0, 0.0
		for k := range 24 {
			b := newBoids()
			b.count = 0
			for i := 1; i < boidsMax; i++ { // the others on the far side: no neighbours within 2 s
				b.set(i, unit3([3]float32{-1, float32(i%7) * 0.01, float32(i%5) * 0.01}), [3]float32{})
			}
			s, c := math.Sincos(float64(k) * math.Pi / 12)
			var v [3]float32
			for a := range 3 {
				v[a] = (float32(c)*e1[a] + float32(s)*e2[a]) * sp
			}
			b.set(0, p0, v)
			n0 := unit3(cross3(b.pos(0), b.vel(0)))
			for range 2 * 30 {
				b.step()
			}
			vv := b.vel(0)
			spd := math.Sqrt(float64(dot3(vv, vv)))
			lo, hi = math.Min(lo, spd), math.Max(hi, spd)
			off = math.Max(off, math.Abs(float64(dot3(b.pos(0), n0)))*180/math.Pi)
		}
		if (hi-lo)/float64(sp) > 0.01 || off > 0.3 {
			t.Errorf("at %.2f rad/s: speeds %.4f–%.4f over the headings (want within 1 %%), up to %.2f° off the great circle (want ≤ 0.3)", sp, lo, hi, off)
		}
	}
}

// The flock's wandering has no four-fold shape: over 120 s from six gathers,
// the flock centre reaches as far along the screen's axes as along its
// diagonals, within 15 % (a diamond with its corners on the axes: up to
// 1.41; a square with its corners on the diagonals: down to 0.71).
func TestTheFlockWandersWithoutADiamond(t *testing.T) {
	var ax, dg float64
	for seed := uint32(1); seed <= 6; seed++ {
		rngState = 2463534242 + seed*7919
		b := newBoids()
		b.gather()
		var reach [8]float64
		for k := range 120 * 30 {
			b.tick()
			if k < 300 {
				continue
			}
			var cx, cy float64
			m := 0
			for i := range boidsN[b.count] {
				if x, y, ok := b.project(wide(b.p[i])); ok {
					cx += float64(x) / boidsScrQ
					cy += float64(y) / boidsScrQ
					m++
				}
			}
			cx, cy = cx/float64(m), cy/float64(m)
			s := (int(math.Floor(math.Atan2(cy, cx)/(math.Pi/4)+0.5)) + 8) % 8
			reach[s] = math.Max(reach[s], math.Hypot(cx, cy))
		}
		for s := range 8 {
			if s%2 == 0 {
				ax += reach[s]
			} else {
				dg += reach[s]
			}
		}
	}
	if r := ax / dg; r < 0.85 || r > 1.15 {
		t.Errorf("the flock centre reaches %.2f× as far along the axes as along the diagonals, want 0.85–1.15 (no diamond, no square)", r)
	}
}

// onScreen reports whether p is drawn on the screen: in front, and inside
// its rectangle in the projection.
func onScreen(b *boids, p [3]int32) bool {
	x, y, ok := b.project(p)
	return ok && x > -boidsScrQ && x < boidsScrQ && y > -boidsScrQ*boidsGridH/boidsGridW && y < boidsScrQ*boidsGridH/boidsGridW
}

// B cancels an edit back to the value before it: on the Boids page, A on
// the count starts editing it, Up changes it (it applies at once), and B
// puts the count from before the edit back, in the flock and on the page.
func TestCancellingAnEditRestoresTheValueBeforeIt(t *testing.T) {
	a, frame := framedApp()
	a.showPage(a.pageIndex("Boids"))
	frame()
	b := a.boids
	before := b.count
	a.page().nav.Focus(0) // the count
	a.Press(BtnA, false)
	frame()
	if !a.editing {
		t.Fatal("A on the count didn't start editing it")
	}
	a.Press(BtnUp, false)
	frame()
	if b.count == before {
		t.Fatal("Up while editing didn't change the count")
	}
	a.Press(BtnB, false)
	frame()
	if a.editing {
		t.Fatal("B didn't end the edit")
	}
	if b.count != before {
		t.Errorf("after B the count is %d boids, want %d (before the edit)", boidsN[b.count], boidsN[before])
	}
}
