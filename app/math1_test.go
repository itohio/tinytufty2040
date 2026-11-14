package app

// MATH-1: fpv-gate's audit tests (docs/evidence/MATH-1-audit.md), copied
// unchanged from tmp/fpv-gate/harness/fpvgate_math1_test.go (the rig
// and the sensor model) and fpvgate_math1_targets_test.go (one target per
// defect, added with its fix).

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

// paddle in cells (top), window in cells (box base, right and up)
func (r *rig) paddle() float64 { return float64(r.a.pong.pad[0]) / pongFix }
func (r *rig) window() (float64, float64) {
	b := r.a.boids
	return float64(b.pose.baseX) / boidsFix, float64(b.pose.baseY) / boidsFix
}

// sensor model, badge upright facing you (badge +Y = world up, +X = world right, +Z = towards you):
// specific force in badge axes = world accel + g·ŷ + offset + noise; tremor is AR(1) noise (~8 Hz band)
type sensor struct {
	rng        *rand.Rand
	accSD      float64 // white accel noise (m/s²)
	tremA      float64 // tremor accel sd (m/s²)
	tremG      float64 // tremor gyro sd (rad/s)
	gyroSD     float64
	off, gbias [3]float64
	ta, tg     [3]float64
	rho        float64 // tremor AR(1) coefficient at 100 Hz
}

func newSensor(seed uint64, accSD, gyroSD, tremA, tremG float64) *sensor {
	return &sensor{rng: rand.New(rand.NewPCG(seed, seed^0x9e37)), accSD: accSD, gyroSD: gyroSD, tremA: tremA, tremG: tremG,
		off: [3]float64{0.04, -0.06, 0.05}, gbias: [3]float64{0.01, -0.006, 0.008}, rho: 0.85}
}

func (s *sensor) sample(worldAcc [3]float64) (acc, gyro [3]float32) {
	rho := s.rho
	for k := range 3 {
		s.ta[k] = rho*s.ta[k] + math.Sqrt(1-rho*rho)*s.rng.NormFloat64()*s.tremA
		s.tg[k] = rho*s.tg[k] + math.Sqrt(1-rho*rho)*s.rng.NormFloat64()*s.tremG
		f := worldAcc[k] + s.off[k] + s.ta[k] + s.rng.NormFloat64()*s.accSD
		if k == 1 {
			f += gravity
		}
		acc[k] = float32(f)
		gyro[k] = float32(s.gbias[k] + s.tg[k] + s.rng.NormFloat64()*s.gyroSD)
	}
	return
}

const desk, hand = 0, 1

func noiseOf(kind int) (accSD, gyroSD, tremA, tremG float64) {
	if kind == desk {
		return 0.03, 0.002, 0, 0
	}
	return 0.03, 0.002, 0.06, 0.017 // owner-1 10–30 s held still: accel sd 0.04–0.09 (lag-1 0.63–0.94), gyro sd 0.014–0.019
}

func settledBy(t *testing.T, tremA, tremG float64, seed uint64, limit float64) float64 {
	r := newRig()
	s := newSensor(seed, 0.03, 0.002, tremA, tremG)
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
		if at := settledBy(t, 0.12, 0.032, seed, 5); at < 0 {
			t.Errorf("seed %d: not settled within 5 s at hand tremor 0.12 m/s², 0.032 rad/s (the owner's 1–10 s hold is 0.065–0.12, 0.025–0.028)", seed)
		}
	}
}

func strokeIn(t *testing.T, kind int, axis int, d, T float64) (paddle, winX float64) {
	r := newRig()
	s := newSensor(7, 0.03, 0.002, 0, 0)
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

// A1 (Leader): until the motion state has started, the screen says so: a
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
