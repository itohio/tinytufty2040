package app

import (
	"errors"
	"image/color"
	"time"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/widget"
	"tinygo.org/x/drivers"
)

// The guided calibration (CAL-1 Part 2, owner-approved): A on the Calibration page offers it. The owner
// lays the badge on three faces (the offsets) or six (offsets and scales),
// in any order: each face is one run of the page's still measurement, and
// the page tells the face from gravity, so a face already done is refused
// and the next missing one asked for. EasyRobot's Solver fits the poses;
// the result is shown, and A uses it: from then until power-off every
// sample is corrected (Fit.Apply, and its gyro bias) before the motion
// state, while the calibrator keeps the raw ones.

// calFace is a face the badge lies on, by where gravity's reaction points
// in the badge's axes (x right, y up, z out of the screen).
type calFace uint8

const (
	faceUp       calFace = iota // flat, screen up: +z
	faceDown                    // screen down: −z
	faceUpright                 // standing on its A/B/C edge: +y
	faceUpside                  // on its top edge: −y
	faceArrows                  // on the edge with the UP/DOWN buttons: −x
	faceOpposite                // on the plain edge opposite them: +x
)

// what each face asks for (a landmark, not left/right; the A/B/C
// buttons are on the bottom edge, so the side edges are named by UP/DOWN)
var calFaceWords = [6]string{
	"lay it flat, screen up", "lay it screen down", "stand it upright",
	"stand it upside down", "lay it on the up/down-button edge", "lay it on the edge opposite up/down",
}

// the same in the panel's still-to-do list
var calFaceShort = [6]string{"screen up", "screen down", "upright", "upside down", "up/down edge", "opposite edge"}

// the order each set asks in: three faces whose up axes differ (the
// solver's minimal set: orthogonal, condition near 1), or all six
var (
	calSet3 = []calFace{faceUp, faceArrows, faceUpright}
	calSet6 = []calFace{faceUp, faceDown, faceUpright, faceUpside, faceArrows, faceOpposite}
)

// faceOf is the face a still pose rests on: its force's largest axis.
func faceOf(f [3]float32) calFace {
	i := 0
	for k := 1; k < 3; k++ {
		if abs32f(f[k]) > abs32f(f[i]) {
			i = k
		}
	}
	switch {
	case i == 2 && f[2] > 0:
		return faceUp
	case i == 2:
		return faceDown
	case i == 1 && f[1] > 0:
		return faceUpright
	case i == 1:
		return faceUpside
	case f[0] < 0:
		return faceArrows
	}
	return faceOpposite
}

// axisOf is the badge axis a face's force lies along (0 x, 1 y, 2 z).
func axisOf(f calFace) int { return 2 - int(f)/2 }

// gStage is what the guided calibration shows.
type gStage uint8

const (
	gOff       gStage = iota // not guided: the single-pose page
	gAsk                     // asking for the next missing face
	gMeasure                 // measuring it
	gMoved                   // it moved: this face again
	gAlready                 // a face already done
	gNotStill                // the solver refused the pose
	gFaceDone                // a face taken
	gResult                  // fitted: A use it, B discard
	gRefused                 // the faces too alike
	gInUse                   // the fit corrects the samples
	gDiscarded               // B on the result
)

const (
	gHoldFor    = 1500 * time.Millisecond // "already done", "not still enough", "discarded" stay up this long
	gFaceDoneIn = time.Second             // "Face k done"
)

// guided is the guided calibration's state; it lives in the calibration
// (the Solver keeps its poses and scratch in itself: no allocation after
// setup).
type guided struct {
	stage  gStage
	set    []calFace // calSet3 or calSet6
	done   [6]bool   // faces done (with three faces: one per axis)
	n      int       // how many
	last   calFace   // the face just done
	at     time.Time // when a held message went up
	solver calib.Solver
	fit    calib.Fit // the result shown, or in use
	inUse  bool      // the fit in use corrects the samples
	use    calib.Fit // the fit in use (a new result is only shown until A)
	txt    [6]textBuf
}

// startGuided starts a guided calibration on three or six faces.
func (k *calibration) startGuided(faces int) {
	g := &k.g
	g.set = calSet3
	if faces == 6 {
		g.set = calSet6
	}
	g.done, g.n, g.stage, g.at = [6]bool{}, 0, gAsk, time.Time{}
	g.solver.Reset()
	k.c.reset()
	k.movedAt = time.Time{}
}

// covered reports whether face f is already done: with three faces any
// face on a done axis (the opposite edge counts for the up/down edge:
// "mislabeling self-corrects").
func (g *guided) covered(f calFace) bool {
	if len(g.set) == 3 {
		for i, d := range g.done {
			if d && axisOf(calFace(i)) == axisOf(f) {
				return true
			}
		}
		return false
	}
	return g.done[f]
}

// next is the first face in the set still missing.
func (g *guided) next() calFace {
	for _, f := range g.set {
		if !g.covered(f) {
			return f
		}
	}
	return g.set[0]
}

// updateGuided moves the guided calibration on (once per frame, from
// update): the calibrator's finished runs become poses, and the last one
// the fit.
func (k *calibration) updateGuided(now time.Time) {
	g := &k.g
	switch g.stage {
	case gResult, gRefused, gInUse:
		k.pct = 100
		return
	case gDiscarded:
		if now.Sub(g.at) >= gHoldFor {
			g.stage = gOff
		}
		return
	case gAlready, gNotStill:
		if now.Sub(g.at) < gHoldFor {
			return
		}
	case gFaceDone:
		if now.Sub(g.at) < gFaceDoneIn {
			return
		}
	}
	switch k.c.phase() {
	case calDone:
		k.takePose(now)
	case calMoved:
		g.stage, k.pct = gMoved, 0
	case calMeasuring:
		g.stage, k.pct = gMeasure, min(max(k.c.progress(), 0), 1)*100
	default:
		if g.stage != gMoved || now.Sub(k.movedAt) >= calMovedFor {
			g.stage, k.pct = gAsk, 0
		}
	}
}

// takePose turns the calibrator's finished run into a pose (or refuses it)
// and starts the next run.
func (k *calibration) takePose(now time.Time) {
	g := &k.g
	p, ok := k.c.pose()
	k.c.reset()
	if !ok {
		return
	}
	f := faceOf(p.Accel)
	if g.covered(f) {
		g.stage, g.at = gAlready, now
		return
	}
	if err := g.solver.AddPose(p); err != nil {
		g.stage, g.at = gNotStill, now
		return
	}
	g.done[f], g.last = true, f
	g.n++
	if g.n < len(g.set) {
		g.stage, g.at, k.pct = gFaceDone, now, 100
		return
	}
	model := calib.OffsetOnly
	if len(g.set) == 6 {
		model = calib.OffsetScale
	}
	fit, err := g.solver.SolveModel(model)
	g.fit = fit
	g.stage, k.pct = gResult, 100
	if err != nil {
		g.stage = gRefused
		if !errors.Is(err, calib.ErrIllPosed) {
			g.fit.Cond = 0
		}
	}
}

// useFit puts the shown fit in use (A on the result).
func (k *calibration) useFit() {
	k.g.use, k.g.inUse, k.g.stage = k.g.fit, true, gInUse
}

// discard drops the shown result (B on it); a fit already in use stays.
func (k *calibration) discard(now time.Time) {
	k.g.stage, k.g.at = gDiscarded, now
	if k.g.inUse {
		k.g.stage = gInUse
	}
}

// correct is a sample as the motion state gets it: with a fit in use, its
// offsets, scales and gyro bias taken out.
func (k *calibration) correct(acc, gyro [3]float32) ([3]float32, [3]float32) {
	if k == nil || !k.g.inUse {
		return acc, gyro
	}
	acc = k.g.use.Apply(acc)
	for i := range gyro {
		gyro[i] -= k.g.use.GyroBias[i]
	}
	return acc, gyro
}

// calibChoices are the Calibration page's A (the highlight on the
// middle one: up quick, down thorough).
var calibChoices = []string{"This pose (quick)", "3 faces: offsets", "6 faces: offsets and scale"}

// initCalibDialog builds the Calibration page's choice once, in setup.
func (a *App) initCalibDialog() {
	a.calDialog = widget.NewDialog(a.overlayStyle(), "Calibrate", nil, calibChoices, func(i int) {
		switch i {
		case 0:
			a.calib.again()
			a.log("calibration started again")
		case 1:
			a.calib.startGuided(3)
			a.log("calibration: 3 faces")
		case 2:
			a.calib.startGuided(6)
			a.log("calibration: 6 faces")
		}
	}, nil)
}

// The guided texts (7-bit ASCII).
const (
	gMovedText     = "Moved - starting this face again"
	gAlreadyText   = "Already done - turn it to another face"
	gNotStillText  = "Not still enough - this face again"
	gRefusedText   = "Faces too alike - add a different face"
	gInUseText     = "Using this calibration until power-off"
	gDiscardedText = "Discarded - back to self-learnt offsets"
	gTodoText      = "still to do:"
	gUseText       = "A: use it   B: discard"
	gAgainText     = "A: calibrate again"
	gNoScaleText   = "scale: not measured"
	gExactText     = "fit exact with 3 faces"
	gGyroSetText   = "with the gyro bias"
)

// the word under the ring
var gWords = [...]string{gAsk: "waiting", gMeasure: "measuring", gMoved: "moved", gAlready: "waiting",
	gNotStill: "waiting", gFaceDone: "done", gResult: "calibrated", gRefused: "refused", gInUse: "in use"}

// guidedLine is the guided calibration's top line and its colour.
func (k *calibration) guidedLine() (string, color.RGBA) {
	g := &k.g
	t := g.txt[0].reset()
	switch g.stage {
	case gMeasure:
		return g.face(t).str("measuring - keep still").String(), colFG
	case gMoved:
		return gMovedText, calWarnC
	case gAlready:
		return gAlreadyText, calWarnC
	case gNotStill:
		return gNotStillText, calWarnC
	case gFaceDone:
		return t.str("Face ").int(g.n).str(" done").String(), calOK
	case gResult:
		t.str("Calibrated on ").int(g.n).str(" faces")
		if len(g.set) == 3 {
			t.str(" (offsets only)")
		}
		return t.String(), calOK
	case gRefused:
		return gRefusedText, calWarnC
	case gInUse:
		return gInUseText, calOK
	case gDiscarded:
		return gDiscardedText, colMuted
	}
	return g.face(t).str(calFaceWords[g.next()]).String(), colMuted
}

// face writes "Face k of n: ": k the face being asked for.
func (g *guided) face(t *textBuf) *textBuf {
	return t.str("Face ").int(g.n + 1).str(" of ").int(len(g.set)).str(": ")
}

// panelLine draws line i of the panel under the ring (its top at y).
func panelLine(d drivers.Displayer, x, y int16, i int, s string, c color.RGBA) {
	ui.DrawText(d, fontSmall, x, y+8+int16(i)*11, s, c)
}

// mg is a force in mg (1 mg = 9.80665 mm/s²), rounded.
func mg(v float32) int {
	x := v / gravity * 1000
	if x < 0 {
		return int(x - 0.5)
	}
	return int(x + 0.5)
}

// guidedPanel draws the panel under the ring (four lines with its top at
// y): the faces still to do, or the result.
func (v *calibView) guidedPanel(d drivers.Displayer, x0, y int16) {
	g := &v.k.g
	x := x0 + 4
	switch g.stage {
	case gResult, gRefused, gInUse:
		f := &g.fit
		if g.stage == gInUse {
			f = &g.use
		}
		if g.stage == gRefused {
			panelLine(d, x, y, 0, g.txt[1].reset().str("spread ").fixed(f.Cond, 0).str(" (needs under ").int(calib.MaxCond).str(")").String(), calWarnC)
		} else {
			panelLine(d, x, y, 0, g.txt[1].reset().str("offset x").sint(mg(f.Offset[0])).str(" y").sint(mg(f.Offset[1])).str(" z").sint(mg(f.Offset[2])).str(" mg").String(), colFG)
			if f.Model == calib.OffsetScale {
				panelLine(d, x, y, 1, g.txt[2].reset().str("scale x").fixed(f.Scale[0]*100, 1).str(" y").fixed(f.Scale[1]*100, 1).str(" z").fixed(f.Scale[2]*100, 1).str(" %").String(), colFG)
				panelLine(d, x, y, 2, g.txt[3].reset().str("ok: fit ").fixed(f.RMS/gravity*1000, 1).str(" mg, spread ").fixed(f.Cond, 1).String(), calOK)
			} else {
				panelLine(d, x, y, 1, gNoScaleText, calChrome)
				panelLine(d, x, y, 2, gExactText, calOK)
			}
			panelLine(d, x, y, 3, gGyroSetText, calChrome) // the faces set the gyro too
		}
		hint := gUseText
		if g.stage != gResult {
			hint = gAgainText
		}
		ui.DrawText(d, fontBody, x, y+54, hint, calThr)
	case gDiscarded:
	default:
		panelLine(d, x, y, 0, gTodoText, calChrome)
		// the missing faces, two to a line (a done face drops off)
		ln, col := 1, 0
		t := &g.txt[1]
		t.reset()
		for _, f := range g.set {
			if g.covered(f) {
				continue
			}
			if col > 0 {
				t.str(", ")
			}
			t.str(calFaceShort[f])
			if col++; col == 2 {
				panelLine(d, x, y, ln, t.String(), colFG)
				ln, col = ln+1, 0
				t = &g.txt[1+ln]
				t.reset()
			}
		}
		if col > 0 {
			panelLine(d, x, y, ln, t.String(), colFG)
		}
	}
}
