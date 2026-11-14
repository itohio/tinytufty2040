package app

import (
	"bufio"
	"math"
	"os"
	"strconv"
	"strings"
	"testing"
)

// capSample is one sample of a badge capture (TG-6), in badge axes as the
// firmware's drain hands it to SetIMU: accelerometer m/s², gyro rad/s.
type capSample struct {
	t        float64 // s, from the capture's seq
	acc, gyr [3]float32
}

// loadCapture reads an imucap capture (testdata): the header's scale and
// the drain's axis map (badge right = −X, up = +Y, out = −Z).
func loadCapture(t *testing.T, path string) (s []capSample, rate float64) {
	t.Helper()
	f, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var ak, gk float64
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		fs := strings.Fields(sc.Text())
		switch {
		case len(fs) > 1 && fs[0] == "IMUCAP" && fs[1] != "END":
			for i := 0; i+1 < len(fs); i++ {
				v, _ := strconv.ParseFloat(fs[i+1], 64)
				switch fs[i] {
				case "rate_hz":
					rate = v
				case "accel_nm_s2_per_lsb":
					ak = v * 1e-9
				case "gyro_nrad_s_per_lsb":
					gk = v * 1e-9
				}
			}
		case len(fs) == 8 && fs[0] == "R":
			var r [7]float64
			for j := range 7 {
				r[j], _ = strconv.ParseFloat(fs[1+j], 64)
			}
			s = append(s, capSample{t: r[0] / rate,
				acc: [3]float32{float32(-r[1] * ak), float32(r[2] * ak), float32(-r[3] * ak)},
				gyr: [3]float32{float32(-r[4] * gk), float32(r[5] * gk), float32(-r[6] * gk)}})
		}
	}
	if rate == 0 || len(s) == 0 {
		t.Fatalf("%s: no capture", path)
	}
	return s, rate
}

// replay feeds a capture through the app as the firmware does: SetIMU every
// sample, the frame hand-off every 5 samples (~50 ms frames); each runs
// after every sample with its index.
func replay(a *App, s []capSample, rate float64, each func(i int)) {
	for i, c := range s {
		a.SetIMU(c.acc, c.gyr, float32(1/rate))
		if i%5 == 4 {
			a.frameMotion()
		}
		if each != nil {
			each(i)
		}
	}
}

// After a hard turn the attitude agrees with the accelerometer again within
// seconds of the badge coming to rest (TG-6 capture 0, owner 2026-11-14:
// picked up, turned past 90°, held, set down flat at 55.3 s). The filter's
// gyro-only gate measured the "linear acceleration" with the filter's own
// attitude, so an attitude error read as acceleration and kept the
// accelerometer out: 11° off at the set-down, still 7° after 60 s at rest.
func TestAttitudeRecoversAfterATurnAtRest(t *testing.T) {
	s, rate := loadCapture(t, "testdata/tg6-pickup.log")
	a := testApp()
	a.motion = newMotion()
	m := a.motion
	var sum [3]float64
	n := 0
	check := func(at float64) {
		// the accelerometer's direction over the half second before
		up := [3]float64{sum[0], sum[1], sum[2]}
		l := math.Sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2])
		dot := (up[0]*float64(m.up[0]) + up[1]*float64(m.up[1]) + up[2]*float64(m.up[2])) / l
		deg := math.Acos(math.Min(1, dot)) * 180 / math.Pi
		t.Logf("%.1f s after the set-down: the attitude %.2f° off the accelerometer", at-55.3, deg)
		if deg > 2 {
			t.Errorf("%.1f s after the set-down, still: the attitude is %.1f° off the accelerometer, want ≤ 2°", at-55.3, deg)
		}
	}
	replay(a, s, rate, func(i int) {
		c := s[i]
		if c.t >= 58.3-0.5 && c.t < 58.3 || c.t >= 70-0.5 && c.t < 70 {
			for j := range 3 {
				sum[j] += float64(c.acc[j])
			}
			n++
		}
		if math.Abs(c.t-58.3) < 0.5/rate || math.Abs(c.t-70) < 0.5/rate {
			a.frameMotion() // the up the demos see now
			check(c.t)
			sum, n = [3]float64{}, 0
		}
	})
	if n != 0 {
		t.Fatal("the capture ended before the checks")
	}
}
