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
	// its attitude) is below settleAccel, on the force averaged over
	// settleTau: a still hand's tremor (0.06–0.12 m/s², MATH-1) is
	// zero-mean and averages out, a real motion doesn't
	settleAccel = 0.15 // m/s² across gravity (0.88°): under the strapdown's 0.2 rest threshold
	settleTau   = 1.0  // s
	// the specific force the demos feel is low-passed over forceTau (s):
	// sensor noise off, a shake (a few Hz) kept
	forceTau = 0.03
	// the filter has settled when the attitude points along the averaged
	// accelerometer (within settleAccel across gravity: its direction, not
	// its size) for settleTime in a row, once the gyro bias is learnt: the
	// strapdown learns its bias then (0.3 m/s² left it 0.09 off; under 0.2
	// it re-learns at later rests)
	settleTime = 0.5 // s
	// the filter's gain: high until it settles (it starts flat and has to
	// turn to the badge's pose: 0.1 s instead of 0.5 s per e-fold), then the
	// steady gain
	kpSettle = 10
	kp       = 2
	// the shown position (track): a motion shorter than holdMove ends where
	// it went; longer, the leak fades in over fadeMove up to 1/leakTau; after
	// a long motion the position goes back to where it began (recentreTau)
	holdMove    = 2.0 // s: a 10 cm stroke takes about 1 s
	fadeMove    = 1.0 // s
	leakTau     = 1.5 // s
	recentreTau = 2.0 // s
)

type motion struct {
	ahrs     *ahrs.MahonyAHRS
	in       mat.Matrix3x3 // accel, gyro, mag rows; passed by pointer (no boxing)
	bias     [3]float32
	biasN    int        // rest samples in the first estimate (restFirst: done)
	resting  bool       // the last sample was at rest
	gRest    float32    // |a| at rest (m/s²): gravity as this accelerometer reads it (9.7 upright, 10.0 flat)
	up       [3]float32 // world up in the badge's frame, once a frame (frameMotion)
	fresh    bool       // samples arrived since the last frameMotion
	samples  int
	rpy      [3]float32 // roll, pitch, yaw (rad), badge-relative, once a frame: see badgeEuler
	yaw0     float32    // the yaw at the last path reset: yaw is shown from there
	lastGyro [3]float32 // after the bias
	// EasyRobot's strapdown brick (x/math/nav/strapdown) on the filter's
	// attitude, for nav.Still: whether the badge rests
	nav strapdown.Nav
	// the shared motion state every demo reads (owner 2026-11-13: one
	// motion state, computed once per IMU sample): the attitude (ahrs),
	// the shown world position (pos, see track), and, in the badge's own
	// frame (owner: "vertical position in the badge frame of reference ...
	// same for water and fire"), the displacement (each world step turned
	// into the badge's axes as it is now, summed) and the specific force
	// (what the accelerometer reads: gravity plus the badge's own
	// acceleration)
	pos       [3]float32 // m, world (z up): the shown position (track)
	vel       [3]float32 // m/s, world: pos's velocity during a long motion
	off       [3]float32 // pos − nav.P while pos follows the strapdown (at rest, in a stroke)
	anchor    [3]float32 // pos where the current or last motion began
	moveFor   float32    // s since the badge left rest (0 at rest)
	long      bool       // the last motion lasted holdMove or more: at rest pos goes back to anchor
	disp      [3]float32 // m, badge axes (+X right, +Y up the screen, +Z out of it)
	force     [3]float32 // m/s², badge axes, low-passed (forceTau)
	lastPos   [3]float32 // pos at the previous sample
	movesN    int        // the strapdown's moving episodes since the last motion report
	wasStill  bool       // the last sample was at rest (an episode starts when it leaves rest)
	settleFor float32    // s in a row the attitude has explained the averaged accelerometer
	fSettle   [3]float32 // the force averaged over settleTau for the settle test (badge axes)
	fSeeded   bool       // fSettle holds a sample
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
	// kept turning it after
	return &motion{ahrs: ahrs.NewMahony(ahrs.WithKP(kpSettle), ahrs.WithKI(0), ahrs.WithMagnetometer(false)),
		up: [3]float32{0, 1, 0}, nav: nav, force: [3]float32{0, gravity, 0}, gRest: gravity}
}

// SetIMU feeds one IMU sample (accelerometer m/s², gyro rad/s, sensor =
// badge axes) taken dt seconds after the previous one: every sample (the
// firmware drains the sensor's FIFO, 100 Hz), before Update. It updates the
// shared motion state; Update hands it to the demos once a frame
// (frameMotion).
func (a *App) SetIMU(acc, gyro [3]float32, dt float32) {
	m := a.motion
	if m == nil {
		return
	}
	if a.calib != nil { // the Calibration page's calibrator; the motion state runs on regardless
		a.calib.add(acc, gyro, dt)
	}
	m.samples++
	m.learnBias(acc, gyro)
	for i := range gyro {
		gyro[i] -= m.bias[i]
	}
	m.lastGyro = gyro
	// the accelerometer corrects the tilt on every sample: no gyro-only
	// gate. Every gate tried held the correction off through hand motion
	// (it turns at 0.4–1.4 rad/s) and after a flip, and the tilt error then
	// leaked gravity into the motion; one gated on the force turned into
	// the world by the filter's own attitude even locked itself (an
	// attitude error read as acceleration: TG-6 capture 0, still 7° off
	// after 60 s at rest). The owner's squares close only ungated (TG-6
	// capture 2).
	m.settle(acc, dt)
	m.in = mat.Matrix3x3{acc, gyro, {}}
	m.ahrs.GainP = kp
	if !m.settled {
		m.ahrs.GainP = kpSettle
	}
	m.ahrs.Update(dt, &m.in)
	q := m.ahrs.Output()
	if m.settled {
		// only once the filter has turned to the badge's pose: fed from
		// power-on, the strapdown learnt its bias while the filter still
		// turned from flat to upright (0.27 m/s² off), and a bias error above
		// its rest threshold kept it from ever seeing a rest to re-learn at
		m.nav.Update(dt, [4]float32(q), acc, gyro)
		m.track(q, acc, dt)
	}
	m.sampleN++
	if m.nav.Still {
		m.restN++
	}
	// the world step since the last sample, turned into the badge's axes
	d := [3]float32{m.pos[0] - m.lastPos[0], m.pos[1] - m.lastPos[1], m.pos[2] - m.lastPos[2]}
	m.lastPos = m.pos
	m.addStep(toBadge(q, d), m.nav.Still)
	k := dt / (forceTau + dt)
	for i := range acc {
		m.force[i] += (acc[i] - m.force[i]) * k
	}
	m.fresh = true
}

// frameMotion hands the samples since the last frame to the demos, once a
// frame (Update, after the firmware drained the sensor): the attitude as
// world up and Euler angles, then applyMotion. The demos read these once a
// frame; computed every sample they took 1.2 ms of the 2 ms a sample cost
// on the badge (TG-5 5b).
func (a *App) frameMotion() {
	m := a.motion
	if m == nil || !m.fresh {
		return
	}
	m.fresh = false
	q := m.ahrs.Output()
	q0, q1, q2, q3 := q[0], q[1], q[2], q[3]
	// world up in the badge's frame (what a resting accelerometer measures)
	m.up = [3]float32{2 * (q1*q3 - q0*q2), 2 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3}
	m.rpy[0], m.rpy[1], m.rpy[2] = badgeEuler(q)
	m.rpy[2] = wrapPi(m.rpy[2] - m.yaw0)
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
//   - The Motion page's path is the shown world position, seen from the
//     badge (pathTrack reads pos; pathView turns the camera with the attitude).
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

// settle notes when the filter has turned from flat to the badge's pose
// (the path starts there).
func (m *motion) settle(acc [3]float32, dt float32) {
	if m.settled {
		return
	}
	if !m.fSeeded {
		m.fSettle, m.fSeeded = acc, true
	}
	k := dt / (settleTau + dt)
	for i := range acc {
		m.fSettle[i] += (acc[i] - m.fSettle[i]) * k
	}
	// the filter starts flat and turns to the badge's pose: settled when the
	// attitude points along the averaged force, its direction only. An
	// attitude can turn the reading but never change its size, and an
	// uncalibrated MPU6050 reads 0.1–0.5 m/s² off g at rest (the badge:
	// 9.7), so a test on the size never passed there and the path never
	// started. No test of rest sample by sample: in a still hand the gyro
	// tremor (0.03 rad/s) broke the app's rest test on 30 % of the samples,
	// the run never reached settleTime and nothing moved (MATH-1 A1); a
	// real motion turns the average away from the attitude instead.
	fw := toWorld(m.ahrs.Output(), m.fSettle)
	f2 := fw[0]*fw[0] + fw[1]*fw[1] + fw[2]*fw[2]
	const tilt = settleAccel / gravity // the largest angle (rad) left between them
	if m.biasN >= restFirst && fw[2] > 0 && fw[0]*fw[0]+fw[1]*fw[1] < tilt*tilt*f2 {
		m.settleFor += dt
	} else {
		m.settleFor = 0
	}
	if m.settleFor >= settleTime {
		m.settled = true
		m.resetPath() // the path starts here
	}
}

// resetPath puts the path's origin where the badge is now (the learnt
// accelerometer bias is kept) and makes the badge's heading now yaw 0.
func (m *motion) resetPath() {
	m.nav.Reset()
	m.pos, m.vel, m.off, m.anchor, m.lastPos = [3]float32{}, [3]float32{}, [3]float32{}, [3]float32{}, [3]float32{}
	m.moveFor, m.long = 0, false
	_, _, m.yaw0 = badgeEuler(m.ahrs.Output())
}

// track moves the shown position: the hybrid the owner chose (TG-6 3b,
// 2026-11-14).
//   - At rest and through a short stroke it follows the strapdown, which
//     integrates rest to rest (its own motion onset and rotation in place
//     included): the stroke ends where it went and holds there.
//   - A motion longer than holdMove can't be integrated with this sensor (a
//     1° tilt error is about 30 m in 20 s of hand motion: the strapdown's
//     position ran away in the owner's capture 2). From then on the world
//     acceleration (the force turned by the filter's attitude, which the
//     accelerometer keeps level, less gravity as this accelerometer reads it
//     at rest) is integrated with velocity and position leaking toward
//     where the motion began, faded in over fadeMove: the motion's shape
//     shows at its size (the owner's 15–30 cm squares) and slow drift
//     doesn't.
//   - After such a motion the position goes back there at rest, and the
//     strapdown is followed again from wherever pos is.
func (m *motion) track(q vec.Quaternion, acc [3]float32, dt float32) {
	if m.nav.Still {
		if m.moveFor > 0 { // a motion has just ended
			m.long = m.moveFor >= holdMove
			m.moveFor = 0
		}
		if !m.long {
			m.followNav()
			return
		}
		k := dt / recentreTau
		for i := range m.pos {
			m.pos[i] += (m.anchor[i] - m.pos[i]) * k
			m.off[i] = m.pos[i] - m.nav.P[i]
		}
		return
	}
	if m.moveFor == 0 { // a motion begins
		m.anchor, m.long = m.pos, false
	}
	m.moveFor += dt
	if m.moveFor < holdMove { // a stroke so far: the strapdown's path
		m.followNav()
		m.vel = m.nav.V
		return
	}
	leak := min(1, (m.moveFor-holdMove)/fadeMove) / leakTau
	aw := toWorld(q, acc)
	aw[2] -= m.gRest
	for i := range aw {
		m.vel[i] += (aw[i] - m.vel[i]*leak) * dt
		m.pos[i] += (m.vel[i] - (m.pos[i]-m.anchor[i])*leak) * dt
	}
}

// followNav puts the shown position on the strapdown's, shifted by off.
func (m *motion) followNav() {
	for i := range m.pos {
		m.pos[i] = m.nav.P[i] + m.off[i]
	}
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
	// the band is centred on gravity as this accelerometer reads it once
	// learnt (this badge: 9.73, not 9.81): centred on 9.80665 it let in
	// −0.52..+0.68 m/s², and gRest crept up (MATH-1 A4)
	g := float32(gravity)
	if m.biasN >= restFirst {
		g = m.gRest
	}
	m.resting = an > g-restAccel && an < g+restAccel
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
		m.gRest += (an - m.gRest) / float32(m.biasN)
		return
	}
	for i := range gyro {
		m.bias[i] += (gyro[i] - m.bias[i]) * biasRate
	}
	// gravity as this accelerometer reads it: only while the strapdown calls
	// the badge still (its test is 0.2 m/s² over 0.3 s). The app's rest test
	// (|f| within 0.6 of g) passes through a gentle vertical stroke, and
	// gRest learnt it as gravity: long mode then integrated nothing (MATH-1
	// A4: 10 cm up in 1 s read −0.1 cm)
	if m.nav.Still {
		m.gRest += (an - m.gRest) * biasRate
	}
}

// addStep adds one sample's step (m, badge axes) to the badge-frame
// displacement and counts the strapdown's moving episodes (the console's
// motion readout). No gate: strapdown r10 doesn't start a move on sensor
// noise at rest (r3 did, 0.3 s at a time, which a gate had to hold back,
// and the gate froze the paddle for half a second, then jumped it: owner,
// TG-5 #2).
func (m *motion) addStep(b [3]float32, still bool) {
	if !still && m.wasStill {
		m.movesN++
	}
	m.wasStill = still
	for i := range b {
		m.disp[i] += b[i]
	}
}
