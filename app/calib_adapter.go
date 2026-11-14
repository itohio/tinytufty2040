package app

import "github.com/itohio/EasyRobot/x/math/nav/calib"

// calibAdapter is EasyRobot's Calibrator (x/math/nav/calib) as the
// Calibration page's calibrator. The Calibrator is a value (212 B on the
// RP2040): it lives here, no further allocation.
type calibAdapter struct{ c calib.Calibrator }

func newCalibAdapter() calibrator {
	c, err := calib.New(calib.DefaultConfig())
	if err != nil {
		panic("calib: " + err.Error()) // DefaultConfig is valid
	}
	return &calibAdapter{c: c}
}

func (a *calibAdapter) add(acc, gyro [3]float32, dt float32) { a.c.Update(dt, acc, gyro) }
func (a *calibAdapter) progress() float32                    { return a.c.Progress() }
func (a *calibAdapter) reset()                               { a.c.Reset() }
func (a *calibAdapter) thresholds() (acc, gyro float32)      { return a.c.Thresholds() }

func (a *calibAdapter) phase() calPhase {
	switch a.c.State() {
	case calib.Measuring:
		return calMeasuring
	case calib.Moved:
		return calMoved
	case calib.Done:
		return calDone
	}
	return calWaiting
}

func (a *calibAdapter) result() calResult {
	r, _ := a.c.Result()
	return calResult{gyroBias: r.GyroBias, forecast: r.Forecast}
}

// stats: axes 0..2 the accelerometer (m/s²), 3..5 the gyro (rad/s); the
// spread is the standard deviation over the Calibrator's window (0.5 s).
func (a *calibAdapter) stats(i int) (mean, spread float32) {
	if i < 3 {
		s := a.c.Accel()
		return s.Mean[i], s.Spread[i]
	}
	s := a.c.Gyro()
	return s.Mean[i-3], s.Spread[i-3]
}
