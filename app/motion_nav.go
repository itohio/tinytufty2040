package app

import (
	"github.com/itohio/EasyRobot/x/math/nav/calib"
	"github.com/itohio/EasyRobot/x/math/nav/eskf"
)

// navState is the Kalman estimator: EasyRobot's error-state Kalman filter
// (x/math/nav/eskf), which estimates the attitude, velocity, position and
// both sensors' biases together. Its position is shown as it is (from the
// last path reset), with no rules on top.
type navState struct {
	f     eskf.Filter
	P     [3]float32 // m, world (z up): the filter's position from the last Reset
	Still bool       // the filter's calm: not translating
	p0    [3]float32 // the filter's position at the last Reset
}

// init makes the filter in place: navState lives inside motion (one
// allocation; the filter is 2.9 KB, and a second block of it at boot ran
// the badge out of memory).
func (n *navState) init() {
	c := eskf.DefaultConfig()
	c.AccelRange = AccelFullScale
	var err error
	if n.f, err = eskf.New(c); err != nil {
		panic("eskf: " + err.Error()) // DefaultConfig is valid
	}
}

// Reset puts the shown origin where the filter is now (the filter keeps its
// state: it has no position fix to lose).
func (n *navState) Reset() {
	n.p0 = n.f.P
	n.P = [3]float32{}
}

// seedGyro puts a still measurement's gyro bias into the filter before its
// first rest; from then the filter estimates the bias itself.
func (n *navState) seedGyro(b [3]float32) {
	if !n.f.Ready {
		n.f.GyroBias = b
	}
}

// calAccelBiasSigma is how well a saved calibration knows the
// accelerometer's bias, per axis (m/s²), as the filter's prior (not the
// fit's own RMS: six orientations for six unknowns leave none).
const calAccelBiasSigma = 0.05

// usePrior hands the filter a calibration put in use: its gyro bias and the
// accelerometer's residual bias at zero known to calAccelBiasSigma. The clip
// limits stay the sensor's raw full scales: Prior maps them through the fit
// per axis and sign (a range corrected here would be corrected twice).
func (n *navState) usePrior(fit *calib.Fit) {
	p := *fit
	p.RMS = calAccelBiasSigma // Prior's accelerometer-bias sigma (until Prior takes it as an argument)
	n.f.Prior(&p)
	_ = n.f.SetRange(AccelFullScale, GyroFullScale) // the raw full scales: finite and ≥ 0, never refused
}

// update feeds one sample: the accelerometer as the app corrects it, the
// raw gyro.
func (n *navState) update(dt float32, acc, gyro [3]float32) {
	f := &n.f
	f.Update(dt, acc, gyro)
	if !f.Ready {
		n.Still = false
		return
	}
	for i := range n.P {
		n.P[i] = f.P[i] - n.p0[i]
	}
	n.Still = f.Calm
}
