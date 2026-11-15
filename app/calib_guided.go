package app

import (
	"errors"
	"image/color"
	"math"
	"time"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/widget"
	"tinygo.org/x/drivers"
)

// The orientation calibration (CAL-2, owner 2026-11-15): A on the
// Calibration page offers it. The owner picks one of the badge's six
// orientations with up/down, places the badge in it, and once it lies
// there still the page's still measurement takes it; the highlight then
// moves to the next one not done. Three orientations on different axes
// fit the accelerometer's offsets, all six (each axis both ways) its
// scales too, with EasyRobot's Solver; A uses the fit: from then until
// power-off every accelerometer sample is corrected (Fit.Apply) before the
// motion state, while the calibrator keeps the raw ones. A table shows, for
// every orientation done, how far gravity as measured is from 1 g, raw and
// fitted, and the spread across orientations.

// calFace is an orientation, by where gravity's reaction points in the
// badge's axes (x right, y up, z out of the screen).
type calFace uint8

const (
	faceUp       calFace = iota // flat, screen up: +z
	faceDown                    // screen down: −z
	faceUpright                 // standing on its A/B/C edge: +y
	faceUpside                  // on its top edge: −y
	faceArrows                  // on the edge with the UP/DOWN buttons: −x
	faceOpposite                // on the plain edge opposite them: +x
)

// what each orientation asks for (a landmark, not left/right; the A/B/C
// buttons are on the bottom edge, so the side edges are named by UP/DOWN)
var calFaceWords = [6]string{
	"lay it flat, screen up", "lay it screen down", "stand it upright",
	"stand it upside down", "lay it on the up/down-button edge", "lay it on the edge opposite up/down",
} // the top line (title case)

// the same, short, in the table and the messages
var calFaceShort = [6]string{"screen up", "screen down", "upright", "upside down", "up/down edge", "opposite edge"}

// faceOf is the orientation a still pose rests in: its force's largest axis.
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

// axisOf is the badge axis an orientation's force lies along (0 x, 1 y, 2 z).
func axisOf(f calFace) int { return 2 - int(f)/2 }

// gStage is the step the orientation calibration shows.
type gStage uint8

const (
	gOff       gStage = iota // not running: the single-pose page
	gPlace                   // the selected orientation isn't how the badge lies
	gHold                    // it lies so, not still yet
	gMeasure                 // measuring it
	gMoved                   // it moved: this orientation again
	gWrong                   // a run finished in another orientation than the selected one
	gNotStill                // the solver refused the pose
	gTaken                   // an orientation taken
	gAll                     // all six taken
	gDiscarded               // B on a fit
)

// gFit is what the fit shows.
type gFit uint8

const (
	fitNone    gFit = iota // not enough orientations yet
	fitShown               // a new fit: A uses it, B discards it
	fitRefused             // the orientations too alike
	fitInUse               // the fit shown is the one in use
)

const (
	gHoldFor  = 1500 * time.Millisecond // "wrong", "not still enough", "discarded" stay up this long
	gTakenFor = time.Second             // "<orientation> done"
)

// guided is the orientation calibration's state; it lives in the
// calibration (the Solver keeps its poses and scratch in itself: no
// allocation after setup).
type guided struct {
	stage  gStage
	sel    calFace // the highlighted orientation
	done   [6]bool // orientations taken
	pose   [6]calib.Pose
	last   calFace   // the orientation just taken, or the one a wrong run lay in
	at     time.Time // when a held message went up
	solver calib.Solver
	fitSt  gFit
	fit    calib.Fit // the fit shown
	inUse  bool      // a fit corrects the samples
	fitN   int       // fits solved so far (App.Update logs each new one: calReport)
	use    calib.Fit // the fit in use (a new fit is only shown until A)
	txt    [12]textBuf
}

// startGuided starts an orientation calibration from scratch; a fit in use
// stays in use until A on a new one.
func (k *calibration) startGuided() {
	g := &k.g
	g.done, g.stage, g.at, g.sel, g.fitSt = [6]bool{}, gPlace, time.Time{}, faceUp, fitNone
	if g.inUse {
		g.fit, g.fitSt = g.use, fitInUse
	}
	k.c.reset()
	k.movedAt = time.Time{}
}

// count is how many orientations are done, and how many axes they cover.
func (g *guided) count() (n, axes int) {
	var ax [3]bool
	for f, d := range g.done {
		if d {
			n++
			ax[axisOf(calFace(f))] = true
		}
	}
	for _, a := range ax {
		if a {
			axes++
		}
	}
	return n, axes
}

// step moves the highlight one orientation (up: back, down: on), wrapping.
// The run in progress starts again: it measures the badge as it lies, and
// the owner is about to move it.
func (k *calibration) step(up bool) {
	g := &k.g
	d := calFace(1)
	if up {
		d = 5
	}
	g.sel = (g.sel + d) % 6
	g.stage = gPlace
	k.c.reset()
}

// nextUndone is the first orientation after f not done (f itself when all
// are done).
func (g *guided) nextUndone(f calFace) calFace {
	for i := calFace(1); i <= 6; i++ {
		if c := (f + i) % 6; !g.done[c] {
			return c
		}
	}
	return f
}

// updateGuided moves the orientation calibration on (once per frame, from
// update): the calibrator's finished runs become poses.
func (k *calibration) updateGuided(now time.Time) {
	g := &k.g
	switch g.stage {
	case gDiscarded, gWrong, gNotStill:
		if now.Sub(g.at) < gHoldFor {
			return
		}
	case gTaken:
		if now.Sub(g.at) < gTakenFor {
			return
		}
	}
	placed := k.face == g.sel
	switch k.c.phase() {
	case calDone:
		k.takePose(now)
		return
	case calMoved:
		g.stage, k.pct = gMoved, 0
		return
	case calMeasuring:
		if placed {
			g.stage, k.pct = gMeasure, min(max(k.c.progress(), 0), 1)*100
			return
		}
	}
	if g.stage == gMoved && now.Sub(k.movedAt) < calMovedFor {
		return
	}
	k.pct = 0
	switch {
	case g.stage == gAll && g.done[g.sel]:
		// all six done and the highlight on a done one: nothing to ask
	case placed:
		g.stage = gHold
	default:
		g.stage = gPlace
	}
}

// takePose turns the calibrator's finished run into the selected
// orientation's pose (a run in another orientation is refused), fits once
// enough are done, and moves the highlight on.
func (k *calibration) takePose(now time.Time) {
	g := &k.g
	p, ok := k.c.pose()
	k.c.reset()
	if !ok {
		return
	}
	if f := faceOf(p.Accel); f != g.sel {
		g.stage, g.at, g.last = gWrong, now, f
		return
	}
	g.pose[g.sel], g.done[g.sel], g.last = p, true, g.sel
	if !k.refit() {
		g.done[g.sel] = false
		g.stage, g.at = gNotStill, now
		return
	}
	g.stage, g.at, k.pct = gTaken, now, 100
	if n, _ := g.count(); n == 6 {
		g.stage = gAll
	}
	g.sel = g.nextUndone(g.sel)
}

// refit feeds every orientation done to the Solver and fits when there are
// enough: three axes the offsets, all six the scales too. False when the
// Solver refuses a pose (not still enough).
func (k *calibration) refit() bool {
	g := &k.g
	g.solver.Reset()
	for f, d := range g.done {
		if d {
			if err := g.solver.AddPose(g.pose[f]); err != nil {
				return false
			}
		}
	}
	n, axes := g.count()
	if axes < 3 {
		return true
	}
	model := calib.OffsetOnly
	if n == 6 {
		model = calib.OffsetScale
	}
	fit, err := g.solver.SolveModel(model)
	g.fit, g.fitSt = fit, fitShown
	if err == nil {
		g.fitN++
	}
	if err != nil {
		g.fitSt = fitRefused
		if !errors.Is(err, calib.ErrIllPosed) {
			g.fit.Cond = 0
		}
	}
	return true
}

// useFit puts the shown fit in use (A on it).
func (k *calibration) useFit() {
	k.g.use, k.g.inUse, k.g.fitSt = k.g.fit, true, fitInUse
}

// discard drops the shown fit (B on it); a fit already in use stays.
func (k *calibration) discard(now time.Time) {
	g := &k.g
	g.stage, g.at, g.fitSt = gDiscarded, now, fitNone
	if g.inUse {
		g.fit, g.fitSt = g.use, fitInUse
	}
}

// correct is an accelerometer sample as the motion state gets it: with a
// fit in use, its offsets and scales taken out. The gyro's bias goes into
// the motion state once, when the page's still measurement completes
// (motion.seedBias), not here.
func (k *calibration) correct(acc [3]float32) [3]float32 {
	if k == nil || !k.g.inUse {
		return acc
	}
	return k.g.use.Apply(acc)
}

// calibChoices are the Calibration page's A (the highlight on the
// orientations).
var calibChoices = []string{"This pose (quick)", "Orientations: offsets, scale"}

// initCalibDialog builds the Calibration page's choice once, in setup.
func (a *App) initCalibDialog() {
	a.calDialog = widget.NewDialog(a.overlayStyle(), "Calibrate", nil, calibChoices, func(i int) {
		switch i {
		case 0:
			a.calib.again()
			a.log("calibration started again")
		case 1:
			a.calib.startGuided()
			a.log("calibration: orientations")
		}
	}, nil)
}

// The orientation texts (7-bit ASCII).
const (
	gHoldText      = "Hold still..."
	gNotStillText  = "Not still enough - this one again"
	gAllText       = "All six done"
	gRefusedText   = "Too alike - add a different orientation"
	gDiscardedText = "Discarded - back to self-learnt offsets"
	gGyroSetText   = "gyro: set"
	gUseText       = "A: use it   B: discard"
	gInUseText     = "in use until power-off"
	gNeedAxesText  = "fit: needs 3 on different axes"
	gNeedBothText  = "scale: needs each axis both ways"
)

// guidedLine is the orientation calibration's top line and its colour.
func (k *calibration) guidedLine() (string, color.RGBA) {
	g := &k.g
	t := g.txt[0].reset()
	switch g.stage {
	case gHold:
		return gHoldText, colFG
	case gMeasure:
		return t.str("Measuring ").str(calFaceShort[g.sel]).str(" - keep still").String(), colFG
	case gMoved:
		return t.str("Moved - ").str(calFaceShort[g.sel]).str(" again").String(), calWarnC
	case gWrong:
		return t.title(calFaceShort[g.last]).str(" now: choose it, or turn").String(), calWarnC
	case gNotStill:
		return gNotStillText, calWarnC
	case gTaken:
		return t.title(calFaceShort[g.last]).str(" done - choose the next").String(), calOK
	case gAll:
		return gAllText, calOK
	case gDiscarded:
		return gDiscardedText, colMuted
	}
	return t.title(calFaceWords[g.sel]).String(), colMuted
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

// norm3f is a vector's length.
func norm3f(v [3]float32) float32 {
	return float32(math.Sqrt(float64(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])))
}

// gDev is |g| measured minus 1 g, in m/s²: raw, or through the fit shown.
func (g *guided) gDev(f calFace, fitted bool) float32 {
	a := g.pose[f].Accel
	if fitted {
		a = g.fit.Apply(a)
	}
	return norm3f(a) - gravity
}

// The table's columns (x from the panel's left): the orientation, its
// state, gravity as measured in g, its deviation from 1 g in mg and %, and
// the fitted deviation in mg. The strips narrow for it (calPanelXO).
const (
	colState = 56
	colG     = 74
	colMg    = 98
	colPct   = 114
	colFit   = 134
)

// guidedPanel draws the orientation calibration in the ring's column (its
// top at y): a row per orientation (the highlight, its state, gravity's
// deviation raw and fitted), the spread across them, the gyro, then the
// fit.
func (v *calibView) guidedPanel(d drivers.Displayer, x0, y int16) {
	k := v.k
	g := &k.g
	x := x0 + 4
	fitted := g.fitSt == fitShown || g.fitSt == fitInUse
	cell(d, x, y, 0, "orientation", calChrome)
	cell(d, x+colG, y, 0, "|g|", calChrome)
	cell(d, x+colMg, y, 0, "mg", calChrome)
	cell(d, x+colPct, y, 0, "%", calChrome)
	if fitted {
		cell(d, x+colFit, y, 0, "fit", calChrome)
	}
	var lo, hi, flo, fhi float32
	n := 0
	for f := calFace(0); f < 6; f++ {
		row := int(1 + f)
		name := g.txt[1+f].reset()
		c := colFG
		if f == g.sel {
			name.str(">")
			c = calThr
		} else {
			name.str(" ")
		}
		cell(d, x, y, row, name.str(calFaceShort[f]).String(), c)
		if !g.done[f] {
			st := "--"
			switch {
			case f == g.sel && g.stage == gMeasure:
				st = g.txt[7].reset().str("meas ").int(int(k.pct)).str("%").String()
			case f == g.sel && g.stage == gHold:
				st = "wait"
			}
			cell(d, x+colState, y, row, st, colMuted)
			continue
		}
		dv := g.gDev(f, false)
		cell(d, x+colState, y, row, "ok", calOK)
		cell(d, x+colG, y, row, g.txt[8].reset().fixed((dv+gravity)/gravity, 3).String(), colFG)
		cell(d, x+colMg, y, row, g.txt[9].reset().sint(mg(dv)).String(), colFG)
		cell(d, x+colPct, y, row, g.txt[10].reset().fixed(dv/gravity*100, 1).String(), colFG)
		lo, hi = minmax(n, dv, lo, hi)
		if fitted {
			fv := g.gDev(f, true)
			cell(d, x+colFit, y, row, g.txt[11].reset().sint(mg(fv)).String(), calOK)
			flo, fhi = minmax(n, fv, flo, fhi)
		}
		n++
	}
	line := 7
	if n >= 2 {
		t := g.txt[7].reset().str("spread ").int(mg(hi - lo)).str(" mg")
		if fitted {
			t.str(" raw, ").int(mg(fhi - flo)).str(" mg fitted")
		} else {
			t.str(" (").fixed((hi-lo)/gravity*100, 1).str(" %)")
		}
		panelLine(d, x, y, line, t.String(), colFG)
	}
	line++
	switch g.fitSt {
	case fitNone:
		panelLine(d, x, y, line, gNeedAxesText, colMuted)
		return
	case fitRefused:
		panelLine(d, x, y, line, g.txt[0].reset().str("spread ").fixed(g.fit.Cond, 0).str(" (needs under ").int(calib.MaxCond).str(")").String(), calWarnC)
		return
	}
	f := &g.fit
	panelLine(d, x, y, line, g.txt[8].reset().str("offset x").sint(mg(f.Offset[0])).str(" y").sint(mg(f.Offset[1])).str(" z").sint(mg(f.Offset[2])).str(" mg").String(), colFG)
	line++
	if f.Model == calib.OffsetScale {
		panelLine(d, x, y, line, g.txt[9].reset().str("scale x").fixed(f.Scale[0]*100, 1).str(" y").fixed(f.Scale[1]*100, 1).str(" z").fixed(f.Scale[2]*100, 1).str(" %").String(), colFG)
	} else {
		panelLine(d, x, y, line, gNeedBothText, colMuted)
	}
	line++
	hint, hc := gUseText, calThr
	if g.fitSt == fitInUse {
		hint, hc = gInUseText, calOK
	}
	panelLine(d, x, y, line, hint, hc)
}

// cell draws one table cell: line i of the panel at x.
func cell(d drivers.Displayer, x, y int16, i int, s string, c color.RGBA) {
	panelLine(d, x, y, i, s, c)
}

// minmax widens [lo, hi] by v (the first value, n == 0, sets both).
func minmax(n int, v, lo, hi float32) (float32, float32) {
	if n == 0 {
		return v, v
	}
	return min(lo, v), max(hi, v)
}

// calReport logs the calibration as numbers, a line each (they fit the log
// line): the fit shown (its model, whether it is in use, the residual and
// the spread, the offsets, scales and gyro bias) and every measured
// orientation's raw pose (the accelerometer's and gyro's means and noise),
// from which the fit and gravity's size per orientation can be recomputed.
// The console's `cal` prints it, and each new fit logs it.
func (a *App) calReport() {
	if a.calib == nil {
		a.log("cal: no IMU")
		return
	}
	g := &a.calib.g
	t := &a.txtStat
	if g.fitSt == fitShown || g.fitSt == fitInUse {
		f := &g.fit
		a.log(t.reset().str("cal fit ").str(f.Model.String()).str(" in_use ").int(btoi(g.fitSt == fitInUse)).
			str(" rms ").fixed(f.RMS, 4).str(" cond ").fixed(f.Cond, 2).String())
		a.log(t.reset().str("cal offset ").fixed(f.Offset[0], 4).str(" ").fixed(f.Offset[1], 4).str(" ").fixed(f.Offset[2], 4).str(" m/s2").String())
		a.log(t.reset().str("cal scale ").fixed(f.Scale[0], 5).str(" ").fixed(f.Scale[1], 5).str(" ").fixed(f.Scale[2], 5).String())
		a.log(t.reset().str("cal gyro_bias ").fixed(f.GyroBias[0], 5).str(" ").fixed(f.GyroBias[1], 5).str(" ").fixed(f.GyroBias[2], 5).str(" rad/s").String())
	} else {
		a.log("cal fit none: needs 3 orientations on different axes")
	}
	n := 0
	for o := calFace(0); o < 6; o++ {
		if !g.done[o] {
			continue
		}
		n++
		p := &g.pose[o]
		a.log(t.reset().str("cal pose ").str(calFaceShort[o]).str(" acc ").fixed(p.Accel[0], 4).str(" ").fixed(p.Accel[1], 4).str(" ").fixed(p.Accel[2], 4).String())
		a.log(t.reset().str("cal pose ").str(calFaceShort[o]).str(" gyro ").fixed(p.Gyro[0], 5).str(" ").fixed(p.Gyro[1], 5).str(" ").fixed(p.Gyro[2], 5).String())
		a.log(t.reset().str("cal pose ").str(calFaceShort[o]).str(" noise ").fixed(norm3f(p.AccelNoise), 4).str(" ").fixed(norm3f(p.GyroNoise), 5).String())
	}
	if n == 0 {
		a.log("cal: no orientations measured (A, Orientations)")
	}
}
