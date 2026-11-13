package app

import (
	"math"
	"testing"
)

// Over many steps with the largest flock, while the box turns, spins and
// shakes and the flock scatters, every boid stays inside the box and flies
// between the speed limits.
func TestBoidsStayInTheBoxWithinSpeed(t *testing.T) {
	b := newBoids()
	b.count = len(boidsN) - 1
	b.pose.spin = true
	for s := range 3000 {
		switch s % 500 {
		case 100:
			b.pose.target += 3 * boxTurnStep
		case 250:
			b.scatter()
		case 300:
			b.pose.shake()
		}
		b.tick()
		for i := range boidsN[b.count] {
			bx, by := b.pose.toBox(b.x[i], b.y[i])
			if bx < -2 || by < -2 || bx > boidsW*boidsFix+1 || by > boidsH*boidsFix+1 {
				t.Fatalf("step %d boid %d outside the box: %d,%d", s, i, bx, by)
			}
			ax, ay := abs32(b.vx[i]), abs32(b.vy[i])
			mag := max(ax, ay) + min(ax, ay)>>2 + min(ax, ay)>>3
			// ±3 for integer rounding of the scaled and rotated components
			if mag > boidsVMax+3 || mag < boidsVMin-3 {
				t.Fatalf("step %d boid %d speed %d outside [%d, %d]", s, i, mag, boidsVMin, boidsVMax)
			}
		}
	}
}

// Turning the box doesn't turn the flock: at any box angle, a boid no wall
// reaches keeps its world position and velocity, so on screen it appears to
// turn the other way.
func TestBoidsKeepTheirWorldHeadingWhenTheBoxTurns(t *testing.T) {
	b := newBoids()
	for angle := range 256 {
		b.pose.setAngle(float32(angle) * 2 * math.Pi / 256)
		b.x[0], b.y[0] = b.pose.cx+3*boidsFix, b.pose.cy-2*boidsFix
		b.vx[0], b.vy[0] = boidsVMax, 0
		b.walls(0)
		if b.x[0] != b.pose.cx+3*boidsFix || b.y[0] != b.pose.cy-2*boidsFix || b.vx[0] != boidsVMax || b.vy[0] != 0 {
			t.Fatalf("angle %d: a boid in the middle was changed to %d,%d v %d,%d", angle, b.x[0], b.y[0], b.vx[0], b.vy[0])
		}
	}
	b.pose.setAngle(math.Pi / 2) // a quarter turn: east in the world is vertical on screen
	if vx, vy := b.pose.vecToBox(boidsVMax, 0); abs32(vx) > 1 || abs32(vy) < boidsVMax-1 {
		t.Fatalf("after a quarter turn the boid should fly vertically on screen, got %d,%d", vx, vy)
	}
}

// A wall that moves pushes the boids it reaches: shaking the box drags its
// walls through boids resting near one; they end up inside, moved along.
func TestBoidsArePushedByAMovingWall(t *testing.T) {
	b := newBoids()
	b.count = 0
	for i := range boidsMax { // all near the left wall, flying up and down
		b.x[i], b.y[i] = boidsMargin+256, int32(i%32)*boidsFix+boidsFix
		b.vx[i], b.vy[i] = 0, boidsVMin
	}
	b.pose.shakeDX, b.pose.shakeDY, b.pose.shakeT = boxShake, 0, 1 // throw the box right
	x0 := b.x[0]
	for range boxShakeLen / 2 {
		b.pose.advance()
		b.step()
	}
	for i := range boidsN[b.count] {
		if bx, _ := b.pose.toBox(b.x[i], b.y[i]); bx < 0 {
			t.Fatalf("boid %d left behind outside the wall: box x %d", i, bx)
		}
	}
	if b.x[0]-x0 < boxShake/2 {
		t.Fatalf("the wall moved %d but pushed the boid only %d", boxShake, b.x[0]-x0)
	}
}

func TestOctantFollowsHeading(t *testing.T) {
	for _, c := range []struct {
		vx, vy int32
		want   uint8
	}{{100, 0, 0}, {70, 70, 1}, {0, 100, 2}, {-70, 70, 3}, {-100, 0, 4}, {-70, -70, 5}, {0, -100, 6}, {70, -70, 7}, {100, -30, 0}, {30, 100, 2}} {
		if got := octant(c.vx, c.vy); got != c.want {
			t.Errorf("octant(%d,%d) = %d, want %d", c.vx, c.vy, got, c.want)
		}
	}
}

// The flock feels the badge's own acceleration (owner 2026-11-13: boids
// feel the specific force): thrown, it drifts the other way, as if left
// behind; a still badge leaves it alone.
func TestBoidsFeelTheBadgesAcceleration(t *testing.T) {
	mean := func(felt float32) float64 {
		rngState = 2463534242
		b := newBoids()
		b.feel(felt, 0)
		var s float64
		for range 10 {
			b.tick()
		}
		n := boidsN[b.count]
		for i := range n {
			s += float64(b.vx[i])
		}
		return s / float64(n)
	}
	still, felt := mean(0), mean(4)
	if felt-still < 10 {
		t.Fatalf("4 m/s² felt to the right: the flock's mean vx %+.0f, still badge %+.0f; want it pushed right", felt, still)
	}
}
