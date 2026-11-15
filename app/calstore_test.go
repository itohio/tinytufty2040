package app

import (
	"strings"
	"testing"
	"time"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
	"github.com/itohio/tinygui/examples/framebuffer"
)

// ramStore is a CalStore in memory (the badge's is its flash).
type ramStore struct {
	b     [calRecordSize]byte
	saved bool
}

func (s *ramStore) Read(p []byte) error {
	if !s.saved {
		for i := range p {
			p[i] = 0xff // erased flash
		}
		return nil
	}
	copy(p, s.b[:])
	return nil
}
func (s *ramStore) Write(p []byte) error { copy(s.b[:], p); s.saved = true; return nil }
func (s *ramStore) Erase() error         { s.saved = false; return nil }

var ownerFit = calib.Fit{Model: calib.OffsetScale, Offset: [3]float32{-0.4308, -0.1660, -0.1652},
	Scale: [3]float32{1.00234, 1.00852, 1.03079}, GyroBias: [3]float32{0.00328, -0.00373, 0.01237}, Cond: 1.14}

// A record holds the fit and the ranges it was made at; it reads back
// exactly, and a damaged one, an older version or another range is refused.
func TestCalRecordRoundTripAndRefusals(t *testing.T) {
	var b [calRecordSize]byte
	encodeCalRecord(&b, &ownerFit, AccelFullScale, GyroFullScale)
	f, err := decodeCalRecord(&b, AccelFullScale, GyroFullScale)
	if err != nil || f != ownerFit {
		t.Fatalf("round trip: %v, %+v", err, f)
	}
	bad := b
	bad[20] ^= 1
	if _, err := decodeCalRecord(&bad, AccelFullScale, GyroFullScale); err == nil {
		t.Fatal("a flipped bit was taken")
	}
	old := b
	old[4] = 0 // version
	if _, err := decodeCalRecord(&old, AccelFullScale, GyroFullScale); err == nil {
		t.Fatal("another version was taken")
	}
	if _, err := decodeCalRecord(&b, 2*gravity, GyroFullScale); err == nil {
		t.Fatal("a record made at another accelerometer range (±2 g) was taken")
	}
	var erased [calRecordSize]byte
	for i := range erased {
		erased[i] = 0xff
	}
	if _, err := decodeCalRecord(&erased, AccelFullScale, GyroFullScale); err == nil {
		t.Fatal("erased flash was taken for a record")
	}
}

func storeApp(st CalStore) (*App, []string) {
	fb := framebuffer.New(ScreenW, 80, make([]uint16, ScreenW*80), nil)
	id := func(d, x time.Duration) time.Duration { return d }
	var lines []string
	a := New(fb, testHW{}, Config{DurationAdd: id, DurationSub: id, IMU: true, CalStore: st,
		Log: func(s string) { lines = append(lines, strings.Clone(s)) }}, time.Now())
	a.boot.on = false // the tests start after the gopher
	return a, lines
}

// At boot a saved calibration for this range is put in use (the samples
// arrive corrected) and said; one for another range isn't, and that is said.
func TestSavedCalibrationIsUsedAtBoot(t *testing.T) {
	st := &ramStore{}
	encodeCalRecord(&st.b, &ownerFit, AccelFullScale, GyroFullScale)
	st.saved = true
	a, _ := storeApp(st)
	if !a.calib.g.inUse || a.calib.g.use != ownerFit {
		t.Fatalf("a saved calibration for this range: in use %v, fit %+v", a.calib.g.inUse, a.calib.g.use)
	}
	raw := [3]float32{-0.4308, 9.80665*1.00852 - 0.1660, -0.1652}
	if c := a.calib.correct(raw); c[1] < 9.80 || c[1] > 9.813 || c[0] > 1e-3 || c[0] < -1e-3 {
		t.Fatalf("the saved fit doesn't correct the samples: %v → %v", raw, c)
	}
	other := &ramStore{saved: true}
	encodeCalRecord(&other.b, &ownerFit, 2*gravity, GyroFullScale)
	b, lines := storeApp(other)
	if b.calib.g.inUse {
		t.Fatal("a calibration made at ±2 g was put in use at ±8 g")
	}
	if !strings.Contains(strings.Join(lines, "\n"), "cal: saved one not used: made at another range") {
		t.Fatalf("not said why the saved one isn't used: %q", lines)
	}
}

// Putting a full fit (six orientations: offsets and scales) in use saves
// it; an offsets-only fit isn't saved.
func TestPuttingAFullFitInUseSavesIt(t *testing.T) {
	off := [3]float32{0.12, -0.25, 0.31}
	sc := [3]float32{1.02, 0.97, 1.01}
	for _, six := range []bool{false, true} {
		a, c, wait := orientApp(t)
		st := &ramStore{}
		a.calStore = st
		faces := []calFace{faceUp, faceUpright, faceArrows}
		if six {
			faces = []calFace{0, 1, 2, 3, 4, 5}
		}
		for _, f := range faces {
			choose(t, a, f)
			measure(c, wait, f, off, sc, [3]float32{})
		}
		a.Press(BtnA, false) // use it
		wait(100 * time.Millisecond)
		if st.saved != six {
			t.Fatalf("six orientations %v, put in use: saved %v", six, st.saved)
		}
		if six {
			f, err := decodeCalRecord(&st.b, AccelFullScale, GyroFullScale)
			u := &a.calib.g.use // the record keeps the model, offsets, scales, gyro bias, rms and spread
			if err != nil || f.Model != u.Model || f.Offset != u.Offset || f.Scale != u.Scale || f.GyroBias != u.GyroBias || f.RMS != u.RMS || f.Cond != u.Cond {
				t.Fatalf("saved %+v (%v), in use %+v", f, err, *u)
			}
		}
	}
}

// cal set puts a fit in use and saves it (offsets m/s², scales, gyro bias
// rad/s), refusing implausible numbers; cal clear erases the saved one.
func TestCalSetAndClear(t *testing.T) {
	st := &ramStore{}
	a, _ := storeApp(st)
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, strings.Clone(s)) }
	run := func(arg string) string {
		lines = lines[:0]
		a.Command("cal", arg)
		a.Update(a.now)
		return strings.Join(lines, "\n")
	}
	out := run("set -0.4308 -0.1660 -0.1652 1.00234 1.00852 1.03079 0.00328 -0.00373 0.01237")
	if !a.calib.g.inUse || a.calib.g.use.Offset != ownerFit.Offset || a.calib.g.use.Scale != ownerFit.Scale || a.calib.g.use.GyroBias != ownerFit.GyroBias || !st.saved {
		t.Fatalf("cal set: in use %v %+v, saved %v; said %q", a.calib.g.inUse, a.calib.g.use, st.saved, out)
	}
	if !strings.Contains(run(""), "cal offset -0.4308 -0.1660 -0.1652 m/s2") {
		t.Fatal("cal doesn't show the fit cal set put in use")
	}
	for _, bad := range []string{"set 1 2 3", "set 0 0 0 1.5 1 1 0 0 0", "set 3 0 0 1 1 1 0 0 0", "set 0 0 0 1 1 1 0.5 0 0", "set x 0 0 1 1 1 0 0 0"} {
		st.saved = false
		if out := run(bad); st.saved || !strings.Contains(out, "cal set:") {
			t.Fatalf("cal %s: saved %v, said %q (want refused)", bad, st.saved, out)
		}
	}
	st.saved = true
	if out := run("clear"); st.saved || !strings.Contains(out, "cal: saved calibration erased") {
		t.Fatalf("cal clear: still saved %v, said %q", st.saved, out)
	}
}

// cal says what is saved, at any time (the boot line comes before a host
// connects): none, the one in use, or one not used and why; every line
// within the log line.
func TestCalSaysWhatIsSaved(t *testing.T) {
	run := func(a *App, arg string) string {
		var lines []string
		a.cfg.Log = func(s string) { lines = append(lines, strings.Clone(s)) }
		a.Command("cal", arg)
		a.Update(a.now)
		return strings.Join(lines, "\n")
	}
	a, _ := storeApp(&ramStore{})
	if out := run(a, ""); !strings.Contains(out, "cal saved: none") {
		t.Fatalf("nothing saved: %q", out)
	}
	run(a, "set -0.4308 -0.1660 -0.1652 1.00234 1.00852 1.03079 0.00328 -0.00373 0.01237")
	if out := run(a, ""); !strings.Contains(out, "cal saved: in use") {
		t.Fatalf("after cal set: %q", out)
	}
	run(a, "clear")
	if out := run(a, ""); !strings.Contains(out, "cal saved: none") {
		t.Fatalf("after cal clear: %q", out)
	}
	other := &ramStore{saved: true}
	encodeCalRecord(&other.b, &ownerFit, 2*gravity, GyroFullScale)
	b, _ := storeApp(other)
	if out := run(b, ""); !strings.Contains(out, "cal saved: not used: made at another range") {
		t.Fatalf("another range at boot: %q", out)
	}
}

// What the badge says about saving follows the fit in use: with a record
// saved, an offsets-only fit put in use isn't saved, so neither cal nor the
// page may call it saved; a full fit put in use is saved, and both say so.
func TestOnlyASavedFitIsCalledSaved(t *testing.T) {
	off := [3]float32{0.12, -0.25, 0.31}
	sc := [3]float32{1.02, 0.97, 1.01}
	a, c, wait := orientApp(t)
	st := &ramStore{}
	a.calStore = st
	var lines []string
	a.cfg.Log = func(s string) { lines = append(lines, strings.Clone(s)) }
	savedLine := func() string {
		lines = lines[:0]
		a.Command("cal", "")
		a.Update(a.now) // the log reaches Config.Log here
		for _, l := range lines {
			if strings.HasPrefix(l, "cal saved:") {
				return l
			}
		}
		return ""
	}
	use := func(faces ...calFace) {
		for _, f := range faces {
			choose(t, a, f)
			measure(c, wait, f, off, sc, [3]float32{})
		}
		a.Press(BtnA, false)
		wait(100 * time.Millisecond)
	}
	use(0, 1, 2, 3, 4, 5)
	if l, h := savedLine(), a.calib.g.useHint(); l != "cal saved: in use" || h != gSavedText {
		t.Fatalf("a full fit in use: %q, page %q; want saved", l, h)
	}
	a.Press(BtnB, false) // leave the finished set; start a new one
	wait(100 * time.Millisecond)
	a.Press(BtnA, false)
	a.Press(BtnA, false)
	use(faceUp, faceUpright, faceArrows) // offsets only: not saved
	if !st.saved {
		t.Fatal("the saved record is gone")
	}
	if l, h := savedLine(), a.calib.g.useHint(); l != "cal saved: not used: a newer fit is in use" || h != gInUseText {
		t.Fatalf("an offsets-only fit in use over a saved one: %q, page %q", l, h)
	}
}
