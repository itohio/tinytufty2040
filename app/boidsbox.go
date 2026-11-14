package app

import "math"

// The boids' world and the screen's box (owner: "the frame is the window
// into boids world ... by moving the walls around we don't move the boids
// since they are as if flying in the free space"). The flock lives in world
// coordinates (1/256 cell); the screen is a boidsW×boidsH box with a centre
// and an angle in that world. Turning or moving the box sweeps its walls
// through the flock; boids away from the walls keep their world heading.
// Buttons turn and shake the box now; an IMU's orientation will drive the
// same pose.

const (
	boxTurnStep = math.Pi / 8   // a button press turns the box 22.5°
	boxTurnRate = math.Pi / 128 // the box sweeps at most this far per tick
	boxShake    = 10 * 256      // how far a shake throws the box (world units)
	boxShakeLen = 20            // ticks of a shake (out and back)
	q14         = 1 << 14
)

// boxPose is where the screen's box sits in the boids' world: a centre and
// an angle in radians (clockwise as seen by the viewer). The angle is kept
// at full precision (it follows the IMU's roll); its sine and cosine are
// computed once when it changes, in Q14 for the per-boid transforms.
type boxPose struct {
	angle, target float32 // current and wanted angle (radians, unwrapped)
	sin, cos      int32   // of angle, Q14
	spin          bool
	baseX, baseY  int32 // centre without the shake
	cx, cy        int32 // centre now
	shakeT        int   // ticks into a shake, 0 when still
	shakeDX       int32
	shakeDY       int32
}

func newBoxPose() boxPose {
	c := boxPose{baseX: boidsW * boidsFix / 2, baseY: boidsH * boidsFix / 2, cos: q14}
	c.cx, c.cy = c.baseX, c.baseY
	return c
}

// setAngle turns the box to a (radians) at once.
func (p *boxPose) setAngle(a float32) {
	p.angle, p.target = a, a
	p.trig()
}

func (p *boxPose) trig() {
	s, c := math.Sincos(float64(p.angle))
	p.sin, p.cos = int32(math.Round(s*q14)), int32(math.Round(c*q14))
}

func (p *boxPose) sincos() (int32, int32) { return p.sin, p.cos }

// advance moves the box one tick towards its target angle and along a shake.
func (p *boxPose) advance() {
	if p.spin {
		p.target += boxTurnRate / 2
	}
	if d := p.target - p.angle; d != 0 {
		p.angle += min(max(d, -boxTurnRate), boxTurnRate)
		p.trig()
	}
	p.cx, p.cy = p.baseX, p.baseY
	if p.shakeT > 0 {
		// a triangle: out to the full throw at the middle, back at the end
		k := int32(boxShakeLen/2 - abs(p.shakeT-boxShakeLen/2))
		p.cx += p.shakeDX * k / (boxShakeLen / 2)
		p.cy += p.shakeDY * k / (boxShakeLen / 2)
		if p.shakeT++; p.shakeT > boxShakeLen {
			p.shakeT = 0
		}
	}
}

// shake throws the box in a random direction and back.
func (p *boxPose) shake() {
	s, c := math.Sincos(float64(rand32()%628) / 100)
	p.shakeDX, p.shakeDY = int32(c*boxShake), int32(s*boxShake)
	p.shakeT = 1
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// toBox maps a world point into box coordinates (0..boidsW*boidsFix across).
// Division, not >>: the shift rounds negative values away from zero and
// would bias the flock (F-35's cousin in the boids, see step). In int64: a
// boid the window left behind may be any distance away, and the Q14
// products wrapped int32 past 512 cells (TG-6 #5a).
func (p *boxPose) toBox(x, y int32) (int32, int32) {
	s, c := int64(p.sin), int64(p.cos)
	dx, dy := int64(x)-int64(p.cx), int64(y)-int64(p.cy)
	return int32((c*dx+s*dy)/q14) + boidsW*boidsFix/2, int32((-s*dx+c*dy)/q14) + boidsH*boidsFix/2
}

// toWorld maps box coordinates back to the world.
func (p *boxPose) toWorld(bx, by int32) (int32, int32) {
	s, c := int64(p.sin), int64(p.cos)
	dx, dy := int64(bx)-boidsW*boidsFix/2, int64(by)-boidsH*boidsFix/2
	return int32((c*dx-s*dy)/q14) + p.cx, int32((s*dx+c*dy)/q14) + p.cy
}

// vecToBox / vecToWorld rotate a vector between the frames (int64, as
// toBox: a glide back can be long).
func (p *boxPose) vecToBox(vx, vy int32) (int32, int32) {
	s, c := int64(p.sin), int64(p.cos)
	return int32((c*int64(vx) + s*int64(vy)) / q14), int32((-s*int64(vx) + c*int64(vy)) / q14)
}

func (p *boxPose) vecToWorld(vx, vy int32) (int32, int32) {
	s, c := int64(p.sin), int64(p.cos)
	return int32((c*int64(vx) - s*int64(vy)) / q14), int32((s*int64(vx) + c*int64(vy)) / q14)
}
