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

// The owner's squares show at their size, in their planes, and close on
// their start (TG-6 capture 2, owner 2026-11-14: screen facing him, three
// square loops in the horizontal plane, first away from him; three in the
// screen plane, up, right, down, left; the badge turned screen up; squares
// in the horizontal plane again, then vertical ones). The badge-frame
// displacement is checked here, in the badge's axes (+X right, +Y up the screen, +Z out of it): the badge
// keeps its pose within each window. The strapdown's position ran away
// instead (metres within seconds: in the hand it never rests).
func TestOwnersSquaresShowInTheirPlanes(t *testing.T) {
	s, rate := loadCapture(t, "testdata/tg6-squares.log")
	a := testApp()
	a.motion = newMotion()
	m := a.motion
	type seg struct {
		name     string
		from, to float64
		u, v     func(d [3]float32) float64 // the plane's axes, cm
		out      func(d [3]float32) float64 // across it
	}
	cm := func(i int, k float64) func([3]float32) float64 {
		return func(d [3]float32) float64 { return k * 100 * float64(d[i]) }
	}
	segs := []seg{
		{"(a) horizontal, upright: right/away", 31, 39, cm(0, 1), cm(2, -1), cm(1, 1)},
		{"(b) screen plane, upright: right/up", 41, 48.5, cm(0, 1), cm(1, 1), cm(2, 1)},
		{"(c) horizontal, flat: right/away", 52, 58.5, cm(0, 1), cm(1, 1), cm(2, 1)},
	}
	var path [][3]float32
	replay(a, s, rate, func(i int) { path = append(path, m.disp) })
	at := func(tt float64) int { return int((tt - s[0].t) * rate) }
	for _, g := range segs {
		i0, i1 := at(g.from), at(g.to)
		d0 := path[i0]
		var umin, umax, vmin, vmax, omin, omax, area float64
		var pu, pv float64
		returns, far := 0, false
		first := ""
		for i := i0; i < i1; i++ {
			d := [3]float32{path[i][0] - d0[0], path[i][1] - d0[1], path[i][2] - d0[2]}
			u, v, o := g.u(d), g.v(d), g.out(d)
			umin, umax = math.Min(umin, u), math.Max(umax, u)
			vmin, vmax = math.Min(vmin, v), math.Max(vmax, v)
			omin, omax = math.Min(omin, o), math.Max(omax, o)
			area += pu*v - u*pv
			pu, pv = u, v
			r := math.Hypot(u, v)
			if first == "" && r > 3 {
				switch {
				case math.Abs(v) >= math.Abs(u) && v > 0:
					first = "+v"
				case math.Abs(v) >= math.Abs(u):
					first = "-v"
				case u > 0:
					first = "+u"
				default:
					first = "-u"
				}
			}
			if r > 10 {
				far = true
			} else if far && r < 10 {
				returns++
				far = false
			}
		}
		su, sv, so := umax-umin, vmax-vmin, omax-omin
		t.Logf("%s: span %.0f × %.0f cm, across %.0f cm, back to start %d times, first move %s, area %.0f cm²", g.name, su, sv, so, returns, first, area/2)
		if su < 12 || su > 45 || sv < 12 || sv > 45 {
			t.Errorf("%s: the squares span %.0f × %.0f cm, want 12–45 cm each way (the owner's 15–30 cm squares)", g.name, su, sv)
		}
		if so >= math.Min(su, sv) {
			t.Errorf("%s: %.0f cm across the squares' plane, want less than its shorter span (%.0f cm): the motion is in that plane", g.name, so, math.Min(su, sv))
		}
		if returns < 2 {
			t.Errorf("%s: back within 10 cm of the start %d times, want at least 2 (he returned each loop)", g.name, returns)
		}
		if g.name[:3] == "(b)" && area > 0 {
			t.Errorf("%s: turning %.0f cm², want clockwise (negative): up, right, down, left as he sees it", g.name, area/2)
		}
	}
	// the first loop's first move, from his still start at 30 s: away from him
	d0 := path[at(30)]
	for i := at(30); i < at(31.5); i++ {
		right, away := 100*float64(path[i][0]-d0[0]), -100*float64(path[i][2]-d0[2])
		if math.Hypot(right, away) > 3 {
			if away < math.Abs(right) {
				t.Errorf("(a) first move (right %.0f, away %.0f cm), want away from him first", right, away)
			}
			break
		}
	}
}
