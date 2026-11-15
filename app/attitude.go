package app

import (
	"github.com/itohio/EasyRobot/x/math/filter/ahrs"
	"github.com/itohio/EasyRobot/x/math/mat"
	"github.com/itohio/EasyRobot/x/math/vec"
)

// attitude is which way the badge is turned, and nothing else: EasyRobot's
// Mahony filter fusing the gyro (its bias taken out: the fit in use's, or
// the Calibration page's still measurement) and the accelerometer (the
// tilt), and whether it has first turned from flat to the badge's pose
// (settle). Boids and Pong read only this.
type attitude struct {
	ahrs      *ahrs.MahonyAHRS
	in        mat.Matrix3x3 // accel, gyro, mag rows; passed by pointer (no boxing)
	bias      [3]float32
	gyro      [3]float32 // the last gyro sample, its bias out
	settleFor float32    // s in a row the attitude has explained the averaged accelerometer
	fSettle   [3]float32 // the force averaged over settleTau for the settle test (badge axes)
	fSeeded   bool       // fSettle holds a sample
	settled   bool       // the filter has agreed with the accelerometer at rest once
}

func newAttitude() attitude {
	// no integral term (KI 0): the gyro's bias comes from the calibration,
	// and an integral wound up while the filter turns from flat to the badge's pose
	// kept turning it after
	return attitude{ahrs: ahrs.NewMahony(ahrs.WithKP(kpSettle), ahrs.WithKI(0), ahrs.WithMagnetometer(false))}
}

// q is the attitude: badge → world (z up).
func (t *attitude) q() vec.Quaternion { return t.ahrs.Output() }

// update fuses one sample (the accelerometer corrected, the raw gyro); true
// on the sample it first settles. The accelerometer corrects the tilt on
// every sample: no gyro-only gate (every gate tried held the correction off
// through hand motion and after a flip, and the tilt error leaked gravity
// into the motion).
func (t *attitude) update(acc, gyro [3]float32, dt float32) (settledNow bool) {
	for i := range gyro {
		gyro[i] -= t.bias[i]
	}
	t.gyro = gyro
	was := t.settled
	t.settle(acc, dt)
	t.in = mat.Matrix3x3{acc, gyro, {}}
	t.ahrs.GainP = kp
	if !t.settled {
		t.ahrs.GainP = kpSettle
	}
	t.ahrs.Update(dt, &t.in)
	return t.settled && !was
}

// settle notes when the filter has turned from flat to the badge's pose
// (the path starts there).
func (t *attitude) settle(acc [3]float32, dt float32) {
	if t.settled {
		return
	}
	if !t.fSeeded {
		t.fSettle, t.fSeeded = acc, true
	}
	k := dt / (settleTau + dt)
	for i := range acc {
		t.fSettle[i] += (acc[i] - t.fSettle[i]) * k
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
	fw := toWorld(t.q(), t.fSettle)
	f2 := fw[0]*fw[0] + fw[1]*fw[1] + fw[2]*fw[2]
	const tilt = settleAccel / gravity // the largest angle (rad) left between them
	if fw[2] > 0 && fw[0]*fw[0]+fw[1]*fw[1] < tilt*tilt*f2 {
		t.settleFor += dt
	} else {
		t.settleFor = 0
	}
	if t.settleFor >= settleTime {
		t.settled = true
	}
}
