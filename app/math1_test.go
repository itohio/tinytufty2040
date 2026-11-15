package app

// MATH-1: the motion audit's tests, copied unchanged from its harness (the
// rig and the sensor model) and its targets (one target per defect, added
// with its fix).

import (
	"math"
	"math/rand/v2"
	"strings"
	"testing"
	"time"

	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/examples/framebuffer"
)

// rig is an app with the motion state, your paddle in play, frames every frameN samples (as the firmware: ~50 ms)
type rig struct {
	a      *App
	frameN int
	i      int
}

func newRig() *rig {
	a := testApp()
	a.motion = newMotion()
	a.pong.motion = a.motion
	a.pong.showFull()
	return &rig{a: a, frameN: 5}
}

func (r *rig) feed(acc, gyro [3]float32, dt float32) {
	r.a.SetIMU(acc, gyro, dt)
	r.i++
	if r.i%r.frameN == 0 {
		r.a.frameMotion()
		r.a.pong.tick()
	}
}

// paddle in cells (top), window in cells (right and down the screen).
// These targets measure the badge-frame displacement in the cells the old
// boids window moved: 20 per metre (1 px = 10 mm, 5 px a cell), up the screen
// negative, as the paddle once did.
const rigCellsPerM = 20

func (r *rig) paddle() float64 { _, y := r.window(); return y }
func (r *rig) window() (float64, float64) {
	d := r.a.motion.disp
	return float64(d[0]) * rigCellsPerM, -float64(d[1]) * rigCellsPerM
}

// sensor model, badge upright facing you (badge +Y = world up, +X = world right, +Z = towards you). Hand model v2,
// fitted to the owner's still-hand capture and physically consistent: the gyro tremor is a real rotation θ
// (θ' = w − λθ, w an AR(1) of sd tremG), which the accelerometer sees; the translational tremor is a damped 8 Hz
// oscillator's spring-and-damping force, of sd tremA (its band content matches the capture's residual after
// rotation: below 0.5 Hz 0.004, 0.5–2 Hz 0.006, 2–15 Hz 0.028 m/s²). White noise accSD, gyroSD.
type sensor struct {
	rng        *rand.Rand
	accSD      float64 // white accel noise (m/s²)
	gyroSD     float64 // white gyro noise (rad/s)
	tremA      float64 // translational tremor: acceleration sd (m/s²)
	tremG      float64 // rotational tremor: gyro sd (rad/s)
	off, gbias [3]float64
	tg, th     [3]float64 // rotational tremor: rate (AR(1)) and angle
	ox, ov     [3]float64 // translational tremor oscillator
}

const (
	tremRhoG = 0.92            // the owner's still hand: gyro lag-1 0.86–0.94
	tremLam  = 1.0             // 1/s: a hand's tilt is bounded (mean-reverting); fitted to the owner's accel x/z sd
	oscW0    = 2 * math.Pi * 8 // physiological tremor 8–12 Hz; fitted to the owner's vertical lag-1
	oscZeta  = 0.3
)

// oscUnit is the oscillator's acceleration sd for a unit drive (computed once, fixed RNG).
var oscUnit = func() float64 {
	r := rand.New(rand.NewPCG(1, 2))
	var x, v, sq float64
	n := 0
	for i := range 200000 {
		a := -oscW0*oscW0*x - 2*oscZeta*oscW0*v
		v += (a + r.NormFloat64()) * 0.01
		x += v * 0.01
		if i > 1000 {
			sq += a * a
			n++
		}
	}
	return math.Sqrt(sq / float64(n))
}()

func newSensor(seed uint64, accSD, gyroSD, tremA, tremG float64) *sensor {
	return &sensor{rng: rand.New(rand.NewPCG(seed, seed^0x9e37)), accSD: accSD, gyroSD: gyroSD, tremA: tremA, tremG: tremG,
		off: [3]float64{0.04, -0.06, 0.05}, gbias: [3]float64{0.01, -0.006, 0.008}}
}

// tremor advances the hand's tremor one sample: translational acceleration ta (world), the tremor's rotation
// angle th (small, badge axes) and its rate w.
func (s *sensor) tremor() (ta, th, w [3]float64) {
	for k := range 3 {
		s.tg[k] = tremRhoG*s.tg[k] + math.Sqrt(1-tremRhoG*tremRhoG)*s.rng.NormFloat64()*s.tremG
		w[k] = s.tg[k] - tremLam*s.th[k]
		s.th[k] += w[k] * 0.01
		a := -oscW0*oscW0*s.ox[k] - 2*oscZeta*oscW0*s.ov[k]
		s.ov[k] += (a + s.rng.NormFloat64()*s.tremA/oscUnit) * 0.01
		s.ox[k] += s.ov[k] * 0.01
		ta[k] = a
	}
	return ta, s.th, w
}

func (s *sensor) sample(worldAcc [3]float64) (acc, gyro [3]float32) {
	ta, th, w := s.tremor()
	v := [3]float64{worldAcc[0] + ta[0], worldAcc[1] + ta[1] + gravity, worldAcc[2] + ta[2]}
	c := [3]float64{th[1]*v[2] - th[2]*v[1], th[2]*v[0] - th[0]*v[2], th[0]*v[1] - th[1]*v[0]} // f = v − θ×v
	for k := range 3 {
		acc[k] = float32(v[k] - c[k] + s.off[k] + s.rng.NormFloat64()*s.accSD)
		gyro[k] = float32(s.gbias[k] + w[k] + s.rng.NormFloat64()*s.gyroSD)
	}
	return
}

const desk, hand = 0, 1

func noiseOf(kind int) (accSD, gyroSD, tremA, tremG float64) {
	if kind == desk {
		return 0.016, 0.0006, 0, 0 // the badge on a desk: accel 0.014–0.018, gyro 0.0006
	}
	return 0.016, 0.0006, 0.031, 0.016 // the owner's still hand: gyro sd 0.013–0.021 (real rotation), residual accel 0.026–0.043
}

// shaky: the owner's 1–10 s hold (gyro sd 0.025–0.028, residual accel 0.048–0.122)
const shakyA, shakyG = 0.09, 0.027

func settledBy(t *testing.T, tremA, tremG float64, seed uint64, limit float64) float64 {
	r := newRig()
	s := newSensor(seed, 0.016, 0.0006, tremA, tremG)
	for i := range int(limit * imuRate) {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
		if r.a.motion.settled {
			return float64(i) / imuRate
		}
	}
	return -1
}

// A1 (app): booted in a hand, the motion state starts within 5 s (until it does, nothing moves the paddle or the window).
func TestMath1MotionStartsInTheHand(t *testing.T) {
	for seed := uint64(1); seed <= 5; seed++ {
		if at := settledBy(t, shakyA, shakyG, seed, 5); at < 0 {
			t.Errorf("seed %d: not settled within 5 s in the owner's 1–10 s hold (hand model v2: translational %.3f m/s², gyro %.3f rad/s of real rotation)", seed, shakyA, shakyG)
		}
	}
}

func strokeIn(t *testing.T, kind int, axis int, d, T float64) (paddle, winX float64) {
	r := newRig()
	s := newSensor(7, 0.016, 0.0006, 0, 0)
	for range 10 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	accSD, gyroSD, tremA, tremG := noiseOf(kind)
	s.accSD, s.gyroSD, s.tremA, s.tremG = accSD, gyroSD, tremA, tremG
	for range 3 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	p0 := r.paddle()
	wx0, _ := r.window()
	n := int(T * imuRate)
	for i := range n + imuRate/2 {
		var w [3]float64
		if i < n {
			u := (float64(i) + 0.5) / float64(n)
			w[axis] = d / (T * T) * (60*u - 180*u*u + 120*u*u*u)
		}
		a, g := s.sample(w)
		r.feed(a, g, 0.01)
	}
	wx, _ := r.window()
	return p0 - r.paddle(), wx - wx0
}

// Green now (keep): on a still support, a 10 cm move is 10 px (2 cells) on the paddle and the window.
func TestMath1StrokeFromRestMovesTenPixels(t *testing.T) {
	if p, _ := strokeIn(t, desk, 1, 0.10, 1); math.Abs(p-2) > 0.25 {
		t.Errorf("10 cm up in 1 s from rest: paddle %.2f cells, want 2 ± 0.25", p)
	}
	if _, w := strokeIn(t, desk, 0, 0.10, 1); math.Abs(w-2) > 0.25 {
		t.Errorf("10 cm right in 1 s from rest: window %.2f cells, want 2 ± 0.25", w)
	}
}

// longModeInput is the double integral, over a 10 cm / 1 s stroke and 0.5 s after, of the world acceleration
// the app's long mode integrates (R(q)·f − gRest ẑ, with the app's live q and gRest), along world axis k.
func longModeInput(t *testing.T, badgeAxis, k int) float64 {
	const d, T = 0.10, 1.0
	r := newRig()
	s := newSensor(7, 0.016, 0.0006, 0, 0)
	for range 13 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	m := r.a.motion
	var v, p float64
	n := int(T * imuRate)
	for i := range n + imuRate/2 {
		var w [3]float64
		if i < n {
			u := (float64(i) + 0.5) / float64(n)
			w[badgeAxis] = d / (T * T) * (60*u - 180*u*u + 120*u*u*u)
		}
		a, g := s.sample(w)
		r.feed(a, g, 0.01)
		aw := toWorld(m.ahrs.Output(), a)
		x := float64(aw[k])
		if k == 2 {
			x -= float64(m.gRest)
		}
		v += x * 0.01
		p += v * 0.01
	}
	return p
}

// A4 (app): a gentle vertical move is not taken for rest: gRest (the app's g, learnt while its rest test
// passes: |f| within 0.6 m/s² of g, gyro < 0.05 rad/s) must not follow it, or long mode subtracts the move.
func TestMath1GRestKeepsVerticalMotion(t *testing.T) {
	if p := longModeInput(t, 1, 2); p < 0.085 {
		t.Errorf("10 cm up in 1 s: R·f − gRest integrates to %.1f cm (want ≥ 8.5): gRest followed the move", p*100)
	}
}

// A4 (app), the band: learnBias's rest band is centred on the nominal g (9.80665), not on the badge's |f| at rest,
// so during a shake it admits more upward than downward samples and gRest climbs; R·f − gRest then pushes the
// shown position down. A 50 mm, 1 Hz shake: the shown centre over 4–6 s within 15 mm of 3b's own (the anchor).
func TestMath1ShakeKeepsItsCentre(t *testing.T) {
	const A, w = 0.025, 2 * math.Pi
	r := newRig()
	r.frameN = 3
	s := newSensor(7, 0.016, 0.0006, 0, 0)
	for range 13 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	m := r.a.motion
	d0, g0 := m.disp[1], m.gRest
	var sum float64
	n := 0
	for i := range 6 * imuRate {
		tt := float64(i+1) * 0.01
		a, g := s.sample([3]float64{0, A * w * w * math.Cos(w*(tt-0.005)), 0})
		r.feed(a, g, 0.01)
		if tt > 4 {
			sum += float64(m.disp[1] - d0)
			n++
		}
	}
	if c := sum / float64(n); math.Abs(c) > 0.015 {
		t.Errorf("50 mm 1 Hz shake: shown centre %.1f mm (want within 15 mm of the anchor); gRest %.3f → %.3f m/s²", c*1000, g0, m.gRest)
	}
}

// strokeShown runs one stroke (d m over T s along a badge axis: 1 up = paddle, 0 right = window) after a desk boot,
// optionally 3 s held in the owner's still hand first, and returns the shown cells 0.5 s after it ends.
func strokeShown(seed uint64, inHand bool, axis int, d, T float64) float64 {
	r := newRig()
	s := newSensor(seed, 0.016, 0.0006, 0, 0)
	for range 10 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	if inHand {
		_, _, s.tremA, s.tremG = noiseOf(hand)
		for range 3 * imuRate {
			a, g := s.sample([3]float64{})
			r.feed(a, g, 0.01)
		}
	}
	p0 := r.paddle()
	wx0, _ := r.window()
	n := int(T * imuRate)
	for i := range n + imuRate/2 {
		var w [3]float64
		if i < n {
			u := (float64(i) + 0.5) / float64(n)
			w[axis] = d / (T * T) * (60*u - 180*u*u + 120*u*u*u)
		}
		a, g := s.sample(w)
		r.feed(a, g, 0.01)
	}
	if axis == 1 {
		return p0 - r.paddle()
	}
	wx, _ := r.window()
	return wx - wx0
}

// A3 (app): a still badge's display holds, but no real stroke from rest may be held away. 3/5/10 cm over
// 0.5/1/2 s, up and right, 3 seeds: shown 0.5–1.5× the truth (cm/5 cells at 10 mm/px), right sign
// (a29fb52: 0.77–1.10×; A3 r1 showed 0.00 for 3 cm/1–2 s and 5–10 cm/2 s).
func TestMath1GentleStrokesFromRestRegister(t *testing.T) {
	for _, axis := range []int{1, 0} {
		for _, d := range []float64{0.03, 0.05, 0.10} {
			for _, T := range []float64{0.5, 1, 2} {
				for seed := uint64(7); seed <= 9; seed++ {
					want := d * 100 / 5
					if got := strokeShown(seed, false, axis, d, T); got < 0.5*want || got > 1.5*want {
						t.Errorf("%.0f cm %s in %.1f s from rest, seed %d: shown %.2f cells (want %.2f, 0.5–1.5×)", d*100, map[int]string{0: "right", 1: "up"}[axis], T, seed, got, want)
					}
				}
			}
		}
	}
}

// A3 (app): in the owner's still hand, a vertical stroke that stands above the tremor still registers with the
// right sign and at least a quarter of its size: 3 cm in 0.5–1 s, 5 cm in 0.5–1 s, 10 cm in 0.5–2 s (above the
// steady tremor: D2, the world acceleration's 0.25 s average less its 2 s average, ≤ 0.044 m/s² in the steady
// hand; 3 cm/2 s and 5 cm/2 s are under it and excluded).
// a29fb52: 0.32–1.0×. Horizontal in the hand is E2/A5's (StrokeInTheHandMovesTenPixels).
func TestMath1StrokesInTheHandRegister(t *testing.T) {
	for _, c := range []struct{ d, T float64 }{{0.03, 0.5}, {0.03, 1}, {0.05, 0.5}, {0.05, 1}, {0.10, 0.5}, {0.10, 1}, {0.10, 2}} {
		for seed := uint64(7); seed <= 9; seed++ {
			want := c.d * 100 / 5
			if got := strokeShown(seed, true, 1, c.d, c.T); got < 0.25*want {
				t.Errorf("%.0f cm up in %.1f s in the hand, seed %d: shown %.2f cells (want ≥ %.2f, right sign; truth %.2f)", c.d*100, c.T, seed, got, 0.25*want, want)
			}
		}
	}
}

// A1: until the motion state has started, the screen says so: a
// frozen paddle is never silent. The footer, and the full-screen views
// (here Pong's PLAY, which has no footer), until it settles.
func TestSettlingIsShown(t *testing.T) {
	fb := framebuffer.New(ScreenW, ScreenH, make([]uint16, ScreenW*ScreenH), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	now := time.Now()
	a := New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id, IMU: true}, now)
	a.pong.showFull()
	warn := framebuffer.To565(colWarn)
	notice := func() (footer bool, pixels int) {
		ctx := ui.NewContext(fb, ScreenW, ScreenH, 0, 0)
		a.pong.full.Draw(&ctx)
		for y := ScreenH - 14; y < ScreenH; y++ {
			for x := range ScreenW {
				if fb.Pix[y*ScreenW+x] == warn {
					pixels++
				}
			}
		}
		return strings.Contains(a.footerText(), settlingText), pixels
	}
	if f, n := notice(); !f || n == 0 {
		t.Fatalf("before the motion state starts: footer notice %v, %d notice pixels on PLAY; want both", f, n)
	}
	s := newSensor(1, 0.03, 0.002, 0, 0)
	for range 5 * imuRate { // on the desk: it settles in about a second
		acc, gyro := s.sample([3]float64{})
		a.SetIMU(acc, gyro, 0.01)
	}
	if !a.motion.settled {
		t.Fatal("not settled after 5 s on the desk")
	}
	if f, n := notice(); f || n != 0 {
		t.Fatalf("after it started: footer notice %v, %d notice pixels on PLAY; want neither", f, n)
	}
}

// A2/A3/E2: held still in the hand for 60 s, the paddle and the window stay put.
func TestMath1StillInTheHandShowsNoMotion(t *testing.T) {
	r := newRig()
	s := newSensor(3, 0.016, 0.0006, 0, 0)
	for range 10 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
	}
	accSD, gyroSD, tremA, tremG := noiseOf(hand)
	s.accSD, s.gyroSD, s.tremA, s.tremG = accSD, gyroSD, tremA, tremG
	p0 := r.paddle()
	wx0, wy0 := r.window()
	var pMax, wMax float64
	for range 60 * imuRate {
		a, g := s.sample([3]float64{})
		r.feed(a, g, 0.01)
		wx, wy := r.window()
		pMax = max(pMax, math.Abs(r.paddle()-p0))
		wMax = max(wMax, math.Hypot(wx-wx0, wy-wy0))
	}
	if pMax > 0.5 || wMax > 0.5 {
		t.Errorf("still in the hand 60 s: paddle moved up to %.2f cells, window %.2f cells (want ≤ 0.5)", pMax, wMax)
	}
}
