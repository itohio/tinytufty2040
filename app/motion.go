package app

import (
	"math"

	"github.com/itohio/EasyRobot/x/math/filter/ahrs"
	"github.com/itohio/EasyRobot/x/math/mat"
	"github.com/itohio/EasyRobot/x/math/nav/strapdown"
	"github.com/itohio/EasyRobot/x/math/vec"
)

// Motion from the IMU, at full precision (owner: "imu is PRECISE and should
// be used precisely"). Samples arrive in the badge's axes (+X right, +Y up
// the screen, +Z out of it; the firmware maps the chip's mounting, which
// faces away from the screen) and SI units. Here: the gyro
// bias is learnt while the badge rests, EasyRobot's Mahony filter fuses
// gyro and accelerometer into an orientation, and the demos' gravity comes
// from the filter's estimate (not the raw accelerometer, so shaking the
// badge doesn't throw the liquid around).

const (
	restGyro  = 0.05 // rad/s: below this on every axis (after the bias) …
	restAccel = 0.6  // … and |a| within this of g (m/s²), the badge is at rest
	restFirst = 30   // rest samples averaged for the first bias estimate
	biasRate  = 0.02 // then the bias follows the resting gyro this fast
	gravity   = 9.80665
	// the filter has settled once the world acceleration (gravity off, by
	// its attitude) is below settleAccel at rest
	settleAccel = 0.1 // m/s² across gravity (0.6°), on the low-passed force: under the strapdown's 0.2 rest threshold
	// the specific force the demos feel is low-passed over forceTau (s):
	// sensor noise off, a shake (a few Hz) kept
	forceTau = 0.03
	// while the badge accelerates (world acceleration, gravity off, above
	// accGate) or turns (faster than turnGate) the filter runs on the gyro
	// alone: its accelerometer correction takes a push for a tilt, and
	// during a turn it pulls the attitude a sample behind (it compares the
	// new reading with the last attitude: 0.4° at 1 rad/s, a few cm of path
	// per turn). At most gateMax seconds in a row, so a drifted attitude
	// still corrects
	accGate  = 0.15 // m/s²: a gentle 10 cm stroke peaks at 0.6
	turnGate = 0.2  // rad/s
	gateMax  = 2    // s
	// the filter has settled when the attitude points along the
	// accelerometer (within settleAccel across gravity: its direction, not
	// its size) for settleTime in a row at rest: the strapdown learns
	// its bias then (0.3 m/s² left it 0.09 off; under 0.2 it re-learns at
	// later rests)
	settleTime = 0.5 // s
	// the filter's gain: high until it settles (it starts flat and has to
	// turn to the badge's pose: 0.1 s instead of 0.5 s per e-fold), then the
	// steady gain
	kpSettle = 10
	kp       = 2
)

type motion struct {
	ahrs     *ahrs.MahonyAHRS
	in       mat.Matrix3x3 // accel, gyro, mag rows; passed by pointer (no boxing)
	bias     [3]float32
	biasN    int  // rest samples in the first estimate (restFirst: done)
	resting  bool // the last sample was at rest
	up       [3]float32
	samples  int
	rpy      [3]float32 // roll, pitch, yaw (rad), badge-relative: see badgeEuler
	yaw0     float32    // the yaw at the last path reset: yaw is shown from there
	lastGyro [3]float32 // after the bias
	// the path: EasyRobot's strapdown brick (x/math/nav/strapdown) on the
	// filter's attitude: nav.P is the position (m, world, z up), nav.Still
	// whether the badge rests (velocity held at zero)
	nav strapdown.Nav
	// the shared motion state every demo reads (owner 2026-11-13: one
	// motion state, computed once per IMU sample): the attitude (ahrs),
	// the world position (nav.P), and, in the badge's own frame (owner:
	// "vertical position in the badge frame of reference ... same for water
	// and fire"), the displacement (each world step turned into the badge's
	// axes as it is now, summed) and the specific force (what the
	// accelerometer reads: gravity plus the badge's own acceleration)
	disp      [3]float32 // m, badge axes (+X right, +Y up the screen, +Z out of it)
	force     [3]float32 // m/s², badge axes, low-passed (forceTau)
	lastP     [3]float32 // nav.P at the previous sample
	pend      [3]float32 // this moving episode's steps, until it passes dispMin and dispMinT
	pendT     float32    // this moving episode's length so far (s)
	falseN    int        // episodes dropped short of the gate since the last motion report
	movesN    int        // … and counted
	live      bool       // this moving episode passed dispMin: steps go straight to disp
	gatedFor  float32    // s in a row the filter ran on the gyro alone
	settleFor float32    // s in a row the attitude has explained the accelerometer at rest
	settled   bool       // the filter has agreed with the accelerometer at rest once
	sampleN   int        // samples since the last motion report (console `motion`)
	restN     int        // … of which the strapdown called rest
	txt       [5]textBuf // the Motion page's lines
}

func newMotion() *motion {
	nav, err := strapdown.New(strapdown.DefaultConfig())
	if err != nil {
		panic("strapdown: " + err.Error()) // DefaultConfig is valid
	}
	// no integral term (KI 0): learnBias removes the gyro's bias, and an
	// integral wound up while the filter turns from flat to the badge's pose
	// kept turning it while the path pauses the accelerometer (gyroOnly)
	return &motion{ahrs: ahrs.NewMahony(ahrs.WithKP(kpSettle), ahrs.WithKI(0), ahrs.WithMagnetometer(false)),
		up: [3]float32{0, 1, 0}, nav: nav, force: [3]float32{0, gravity, 0}}
}

// SetIMU feeds one IMU sample (accelerometer m/s², gyro rad/s, sensor =
// badge axes) taken dt seconds after the previous one: every sample (the
// firmware drains the sensor's FIFO, 100 Hz), before Update. It updates the
// shared motion state and hands it to the demos (applyMotion).
func (a *App) SetIMU(acc, gyro [3]float32, dt float32) {
	m := a.motion
	if m == nil {
		return
	}
	m.samples++
	m.learnBias(acc, gyro)
	for i := range gyro {
		gyro[i] -= m.bias[i]
	}
	m.lastGyro = gyro
	in := acc
	if m.gyroOnly(acc, gyro, dt) {
		in = [3]float32{} // no accelerometer: the filter's correction pauses
	}
	m.in = mat.Matrix3x3{in, gyro, {}}
	m.ahrs.GainP = kp
	if !m.settled {
		m.ahrs.GainP = kpSettle
	}
	m.ahrs.Update(dt, &m.in)
	q := m.ahrs.Output()
	q0, q1, q2, q3 := q[0], q[1], q[2], q[3]
	// world up in the badge's frame (what a resting accelerometer measures)
	m.up = [3]float32{2 * (q1*q3 - q0*q2), 2 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3}
	m.rpy[0], m.rpy[1], m.rpy[2] = badgeEuler(q)
	m.rpy[2] = wrapPi(m.rpy[2] - m.yaw0)
	if m.settled {
		// only once the filter has turned to the badge's pose: fed from
		// power-on, the strapdown learnt its bias while the filter still
		// turned from flat to upright (0.27 m/s² off), and a bias error above
		// its rest threshold kept it from ever seeing a rest to re-learn at
		// (a tilt then moved the path metres)
		m.nav.Update(dt, [4]float32(q), acc, gyro)
	}
	m.sampleN++
	if m.nav.Still {
		m.restN++
	}
	// the world step since the last sample, turned into the badge's axes
	d := [3]float32{m.nav.P[0] - m.lastP[0], m.nav.P[1] - m.lastP[1], m.nav.P[2] - m.lastP[2]}
	m.lastP = m.nav.P
	m.addStep(toBadge(q, d), m.nav.Still, dt)
	k := dt / (forceTau + dt)
	for i := range acc {
		m.force[i] += (acc[i] - m.force[i]) * k
	}
	a.applyMotion()
}

// applyMotion hands the shared motion state to the demos: no demo keeps
// its own integration.
//   - Water and Fire's fuel feel the specific force in the screen's frame:
//     things fall along it, so a tilt and a shake both move them.
//   - The boids' box (the window into their world) turns with the attitude
//     and moves with the displacement in the badge's own frame; the boids
//     feel the badge's own acceleration (a shake stirs them).
//   - Pong's paddle follows the displacement along the badge's own up axis
//     (pong.follow reads disp).
//   - The Motion page's path is the world position, seen from the badge
//     (pathTrack reads nav.P; pathView turns the camera with the attitude).
func (a *App) applyMotion() {
	m := a.motion
	// screen x right = +X, y down = −Y; things fall against the specific force
	gx, gy := -m.force[0], m.force[1]
	a.SetTilt(gx, gy)
	if b := a.boids; b != nil {
		// the box's roll from the attitude (not the force: a shake doesn't
		// turn the window)
		if ux, uy := -m.up[0], m.up[1]; ux*ux+uy*uy > (minTilt/gravity)*(minTilt/gravity) {
			b.turnTo(float32(math.Atan2(float64(ux), float64(uy))))
		}
		// the window moves in the badge's own frame, as the paddle (owner
		// 2026-11-13: "boids window - also badge frame of reference")
		b.moveTo(m.disp[0], m.disp[1])
		// the badge's own acceleration in the screen's frame: the force
		// minus the gravity the attitude expects; nothing until the
		// attitude has settled (at power-on it starts flat, and the
		// difference pushed the flock with 9 m/s²)
		if m.settled {
			b.feel(gx+m.up[0]*gravity, gy-m.up[1]*gravity)
		} else {
			b.feel(0, 0)
		}
	}
}

// toBadge turns a world vector into the badge's frame (the inverse of
// toWorld).
func toBadge(q vec.Quaternion, w [3]float32) [3]float32 {
	q0, q1, q2, q3 := q[0], q[1], q[2], q[3]
	return [3]float32{
		(1-2*(q2*q2+q3*q3))*w[0] + 2*(q1*q2+q0*q3)*w[1] + 2*(q1*q3-q0*q2)*w[2],
		2*(q1*q2-q0*q3)*w[0] + (1-2*(q1*q1+q3*q3))*w[1] + 2*(q2*q3+q0*q1)*w[2],
		2*(q1*q3+q0*q2)*w[0] + 2*(q2*q3-q0*q1)*w[1] + (1-2*(q1*q1+q2*q2))*w[2],
	}
}

// toWorld turns a badge-frame vector (+X right, +Y up the screen, +Z out of
// it) into the world (z up) by the filter's quaternion q.
func toWorld(q vec.Quaternion, b [3]float32) [3]float32 {
	q0, q1, q2, q3 := q[0], q[1], q[2], q[3]
	return [3]float32{
		(1-2*(q2*q2+q3*q3))*b[0] + 2*(q1*q2-q0*q3)*b[1] + 2*(q1*q3+q0*q2)*b[2],
		2*(q1*q2+q0*q3)*b[0] + (1-2*(q1*q1+q3*q3))*b[1] + 2*(q2*q3-q0*q1)*b[2],
		2*(q1*q3-q0*q2)*b[0] + 2*(q2*q3+q0*q1)*b[1] + (1-2*(q1*q1+q2*q2))*b[2],
	}
}

// gyroOnly reports whether the filter should skip its accelerometer
// correction for this sample: the badge accelerates beyond gravity (by the
// filter's attitude so far) or turns; for at most gateMax s in a row. Until
// the filter has settled it is never true (and it notes when it settles).
func (m *motion) gyroOnly(acc, gyro [3]float32, dt float32) bool {
	aw := toWorld(m.ahrs.Output(), acc)
	aw[2] -= gravity
	lin2 := aw[0]*aw[0] + aw[1]*aw[1] + aw[2]*aw[2]
	if !m.settled { // the filter starts flat and turns to the badge's pose: the accelerometer leads
		// settled when the attitude points along the measured force: its
		// direction only. An attitude can turn the reading but never change
		// its size, and an uncalibrated MPU6050 reads 0.1–0.5 m/s² off g at
		// rest (the badge: 9.7), so a test on the size never passed there
		// and the path never started
		fw := toWorld(m.ahrs.Output(), m.force) // low-passed: sensor noise doesn't break the run
		f2 := fw[0]*fw[0] + fw[1]*fw[1] + fw[2]*fw[2]
		const tilt = settleAccel / gravity // the largest angle (rad) left between them
		if m.biasN >= restFirst && m.resting && fw[2] > 0 && fw[0]*fw[0]+fw[1]*fw[1] < tilt*tilt*f2 {
			m.settleFor += dt
		} else {
			m.settleFor = 0
		}
		if m.settleFor >= settleTime {
			m.settled = true
			m.resetPath() // the path starts here
		}
		return false
	}
	w2 := gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]
	if (lin2 <= accGate*accGate && w2 <= turnGate*turnGate) || m.gatedFor >= gateMax {
		m.gatedFor = 0
		return false
	}
	m.gatedFor += dt
	return true
}

// resetPath puts the path's origin where the badge is now (the learnt
// accelerometer bias is kept) and makes the badge's heading now yaw 0.
func (m *motion) resetPath() {
	m.nav.Reset()
	m.lastP = [3]float32{}
	_, _, m.yaw0 = badgeEuler(m.ahrs.Output())
}

// badgeEuler is the badge's attitude as the owner reads it (verify2
// TG-motion-path: upright, the filter's Euler angles read roll −179°): 0, 0,
// 0 when the badge stands upright with its screen facing you. The body axes
// are forward = into the screen (badge −Z, the way you look), right = the
// badge's right (+X), down = down the screen (−Y); the angles are the usual
// yaw-pitch-roll of that body in a world with z down: roll is a turn like a
// steering wheel (right side down positive), pitch the top tipping towards
// you (positive: the screen turns to the floor) or away, yaw the heading (no magnetometer: it has no
// absolute zero and drifts; the page shows it from the last path reset).
func badgeEuler(q vec.Quaternion) (roll, pitch, yaw float32) {
	fwd := toWorld(q, [3]float32{0, 0, -1})
	right := toWorld(q, [3]float32{1, 0, 0})
	down := toWorld(q, [3]float32{0, -1, 0})
	// world z up → z down: (x, y, z) → (x, −y, −z), a half turn about x
	fz, fy, fx := -fwd[2], -fwd[1], fwd[0]
	rz, dz := -right[2], -down[2]
	pitch = float32(-math.Asin(float64(min(max(fz, -1), 1))))
	roll = float32(math.Atan2(float64(rz), float64(dz)))
	yaw = float32(math.Atan2(float64(fy), float64(fx)))
	return roll, pitch, yaw
}

// wrapPi folds an angle into (−π, π].
func wrapPi(a float32) float32 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// learnBias estimates the gyro's offset while the badge rests: the mean of
// the first restFirst resting samples, then a slow follow.
func (m *motion) learnBias(acc, gyro [3]float32) {
	an := float32(math.Sqrt(float64(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])))
	m.resting = an > gravity-restAccel && an < gravity+restAccel
	for i := range gyro {
		if d := gyro[i] - m.bias[i]; d > restGyro || d < -restGyro {
			if m.biasN >= restFirst { // a learnt bias: this is motion
				m.resting = false
			} else if d := gyro[i]; d > restGyro*4 || d < -restGyro*4 {
				m.resting = false // even an unlearnt bias isn't that large
			}
		}
	}
	if !m.resting {
		return
	}
	if m.biasN < restFirst {
		m.biasN++
		for i := range gyro {
			m.bias[i] += (gyro[i] - m.bias[i]) / float32(m.biasN)
		}
		return
	}
	for i := range gyro {
		m.bias[i] += (gyro[i] - m.bias[i]) * biasRate
	}
}

// A moving episode (from rest to rest) reaches the badge-frame displacement
// only once it has gone dispMin (m) and lasted dispMinT (s). At rest, on the
// badge's sensor noise, the strapdown now and then calls the badge moving
// for 0.30 s, the time it takes to see rest again, and integrates 2–5 mm in
// that time (host model at the badge's noise, matching its 94–97 % rest;
// verify2 @1414: the paddle moved 17 px, 2.2 mm, in 30 s); a deliberate move
// lasts longer.
const (
	dispMin  = 0.0015
	dispMinT = 0.45
)

// addStep adds one sample's step (m, badge axes, dt s) to the badge-frame
// displacement: within a moving episode it waits in pend until the episode
// passes dispMin and dispMinT, then all of it counts and the rest goes straight in (a
// real move stays 1:1); an episode that ends in rest short of dispMin is
// dropped. At rest only the first step after a counted move goes in: the
// strapdown's correction as it detects rest belongs to that move; any
// other step at rest is creep (its position wanders by millimetres at rest
// on the badge's sensor noise, under the Motion page's 1 cm readout).
func (m *motion) addStep(b [3]float32, still bool, dt float32) {
	if still {
		switch {
		case m.live:
			for i := range b {
				m.disp[i] += b[i]
			}
			m.movesN++
		case m.pendT > 0:
			m.falseN++ // a false start: dropped
		}
		m.pend, m.pendT, m.live = [3]float32{}, 0, false
		return
	}
	if m.live {
		for i := range b {
			m.disp[i] += b[i]
		}
		return
	}
	for i := range b {
		m.pend[i] += b[i]
	}
	m.pendT += dt
	if p := m.pend; m.pendT >= dispMinT && p[0]*p[0]+p[1]*p[1]+p[2]*p[2] >= dispMin*dispMin {
		for i := range p {
			m.disp[i] += p[i]
		}
		m.pend, m.live = [3]float32{}, true
	}
}
