package app

import (
	"math"

	"github.com/itohio/EasyRobot/x/math/nav/calib"

	"github.com/itohio/EasyRobot/x/math/vec"
)

// Motion from the IMU, at full precision (owner: "imu is PRECISE and should
// be used precisely"). Samples arrive in the badge's axes (+X right, +Y up
// the screen, +Z out of it; the firmware maps the chip's mounting, which
// faces away from the screen) and SI units, the calibration's fit applied.
// Two estimators, each for its own readers, neither with rules (no rest
// detection, hold, leak, recentre or clamp):
//   - the attitude (attitude.go: EasyRobot's Mahony filter, the gyro's bias
//     from the fit in use or the Calibration page's still measurement):
//     Boids, Pong and the demos' gravity;
//   - the Kalman filter (navState: EasyRobot's error-state filter): the
//     badge's path on the Motion page, seen through its own attitude.

const (
	gravity = 9.80665
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
)

// motion is the motion pipeline, each part doing one thing: the attitude,
// the Kalman estimator, the Motion page's conditioning of what it shows,
// and the specific force the demos feel. No part decides anything for
// another, and nothing on top decides (no hold, rest or recentre rule).
type motion struct {
	att     attitude   // which way the badge is turned (Boids, Pong)
	kal     navState   // the Kalman estimator: the path (kal.P), in place
	shKal   lowpass3   // its position through the output bandwidth (the Motion page)
	force   lowpass3   // m/s², badge axes: the specific force over forceTau (Water, Fire)
	bw      int        // the Motion page's output bandwidth: bw20, bw10
	up      [3]float32 // world up in the badge's frame, once a frame (frameMotion)
	rpy     [3]float32 // roll, pitch, yaw (rad), badge-relative, once a frame: see badgeEuler
	yaw0    float32    // the yaw at the last path reset: yaw is shown from there
	fresh   bool       // samples arrived since the last frameMotion
	samples int
	txt     [4]textBuf // the Motion page's lines
}

// the Motion page's output bandwidth: a first-order low-pass on the shown
// position (the estimator and the demos don't see it)
const (
	bw20 = iota
	bw10
)

var bwHz = [...]float32{bw20: 20, bw10: 10}

func newMotion() *motion {
	m := &motion{att: newAttitude(), up: [3]float32{0, 1, 0}, force: lowpass3{[3]float32{0, gravity, 0}}}
	m.kal.init()
	return m
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
	if a.calib != nil {
		// the calibrator measures the raw samples, only while its page is
		// open; a fit in use corrects the accelerometer for everything after
		if a.cur == a.calibPage {
			a.calib.add(acc, gyro, dt)
		}
		acc = a.calib.correct(acc)
	}
	m.step(acc, gyro, dt)
}

// step runs the pipeline on one corrected sample: the attitude, the Kalman
// estimator (the raw gyro: it estimates its own bias), then the
// conditioning of what is shown and felt.
func (m *motion) step(acc, gyro [3]float32, dt float32) {
	m.samples++
	if m.att.update(acc, gyro, dt) {
		m.resetPath() // the path starts where the attitude first settles
	}
	m.kal.update(dt, acc, gyro)
	kb := dt / (1/(2*math.Pi*bwHz[m.bw]) + dt) // the output bandwidth
	m.shKal.step(m.kal.P, kb)
	m.force.step(acc, dt/(forceTau+dt))
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
	q := m.att.q()
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
//   - Boids look through the badge's attitude (boids.camera reads it).
//   - Pong's paddle follows the badge's pitch (pong.follow).
//   - The Motion page's path is the Kalman filter's position, seen through
//     its own attitude (pathTrack, pathView).
func (a *App) applyMotion() {
	m := a.motion
	// screen x right = +X, y down = −Y; things fall against the specific force
	gx, gy := -m.force.y[0], m.force.y[1]
	a.SetTilt(gx, gy)

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

// resetPath puts the Kalman path's origin where the badge is now (its
// learnt state is kept) and makes the badge's heading now yaw 0.
func (m *motion) resetPath() {
	m.kal.Reset()
	m.shKal = lowpass3{}
	_, _, m.yaw0 = badgeEuler(m.att.q())
}

// badgeEuler is the badge's attitude as the owner reads it (seen on
// the badge: upright, the filter's Euler angles read roll −179°): 0, 0,
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

// seedBias puts a still measurement's gyro bias (the Calibration page's)
// into both estimators.
func (m *motion) seedBias(b [3]float32) {
	m.att.bias = b
	m.kal.seedGyro(b)
}

// shown is the Kalman position as the Motion page shows it (through the
// bw low-pass).
func (m *motion) shown() [3]float32 { return m.shKal.y }

// usePrior hands a calibration put in use to both estimators: its gyro bias
// to the attitude, the fit as the Kalman filter's prior (the samples arrive
// corrected by it from then).
func (m *motion) usePrior(fit *calib.Fit) {
	m.att.bias = fit.GyroBias
	m.kal.usePrior(fit)
}

// lowpass3 is a first-order low-pass on a 3-vector (conditioning: the
// Motion page's output bandwidth, the specific force the demos feel).
type lowpass3 struct{ y [3]float32 }

func (l *lowpass3) step(x [3]float32, k float32) {
	for i := range x {
		l.y[i] += (x[i] - l.y[i]) * k
	}
}
