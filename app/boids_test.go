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
	b.fov = 2 // 90°
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
		if b.inView(wide(b.p[0])) {
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
