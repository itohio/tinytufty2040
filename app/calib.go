package app

import (
	"image/color"
	"time"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
	"tinygo.org/x/drivers"
)

// The Calibration page (owner 2026-11-13, TG-3): the first page, only with an
// IMU. From power-on a calibrator measures the gyro's bias and the
// accelerometer's offsets while the badge lies still, flat or on a side; a
// ring shows its progress. The uncalibrated MPU6050's offsets (0.3–0.5 m/s²)
// make the first turns after power-on stray 10–15 cm (strapdown r8). Nothing
// waits for it: every page is there at once, the motion state runs
// uncalibrated until a calibration completes, and A on the page starts one
// afresh.
//
// Beside the ring, one strip per axis (accelerometer x/y/z, gyro x/y/z)
// shows its spread (the band's thickness) and drift (the bright line, the
// mean) over time against the stillness threshold (the shaded zone): the
// owner sees which axis keeps the calibration from starting.

// calPhase is what the calibrator is doing.
type calPhase uint8

const (
	calWaiting   calPhase = iota // waiting for the badge to be still
	calMeasuring                 // still: averaging (progress 0..1)
	calMoved                     // it moved while measuring: starting again
	calDone                      // the result is ready
)

// calResult is what calibration measured, in the badge's axes.
type calResult struct {
	gyroBias [3]float32     // rad/s: subtracted from every gyro sample
	forecast calib.Forecast // the drift to expect: heading °/min, position cm/min (for the done state)
}

// calibrator measures at rest, one IMU sample per add (EasyRobot's
// Calibrator behind calibAdapter). It keeps its statistics running after
// it is done, and stays done until reset.
type calibrator interface {
	add(acc, gyro [3]float32, dt float32)
	phase() calPhase
	progress() float32 // 0..1 while measuring
	result() calResult
	reset() // start a calibration afresh
	// stats is axis i's windowed mean and spread (standard deviation): 0..2
	// the accelerometer x/y/z (m/s²), 3..5 the gyro x/y/z (rad/s)
	stats(i int) (mean, spread float32)
	// thresholds are the spreads under which each sensor counts as still
	thresholds() (acc, gyro float32)
	// pose is a finished run as a still pose (the guided calibration)
	pose() (calib.Pose, bool)
}

// calStage is what the page shows: the calibrator's phase, held long
// enough to read.
type calStage uint8

const (
	calStartup calStage = iota // the instruction, the ring at 0 %
	calShowWaiting
	calShowMeasuring
	calShowMoved
	calShowDone
)

const (
	calStartupFor = 2 * time.Second         // the instruction alone, before "waiting"
	calMovedFor   = 1200 * time.Millisecond // "moved" stays up at least this long
	// the drift line is the mean against its own slow average (calDriftK per
	// frame: about 10 s at 15 fps), not against a fixed rest value: the
	// offsets and the gyro's bias are constant, they are what calibration
	// measures, and against a fixed value they read as permanent drift
	calDriftK = 1.0 / 150
)

// The page's geometry inside the page (304×186; it was laid out for the
// whole screen, the page sits under the tabs and above the footer).
const (
	calRingD    = 104 // the ring's diameter
	calRingX    = 4
	calRingY    = 22
	calPanelX   = 122 // the strips' panel: axis label, trace, state chip
	calTraceX   = calPanelX + 18
	calTraceW   = 132
	calTraceH   = 20
	calChipX    = calTraceX + calTraceW + 4
	calStripGap = 24 // strip to strip
)

// calStripY is the top of axis i's strip; a header row sits above each
// sensor's three.
func calStripY(i int) int16 {
	if i < 3 {
		return 32 + int16(i)*calStripGap
	}
	return 114 + int16(i-3)*calStripGap
}

// The page's palette.
var (
	calTrack  = color.RGBA{0x30, 0x36, 0x3D, 255}
	calFill   = color.RGBA{0x38, 0xBD, 0xF8, 255}
	calOK     = color.RGBA{0x34, 0xD3, 0x99, 255}
	calWarnC  = color.RGBA{0xFB, 0xBF, 0x24, 255}
	calBadC   = color.RGBA{0xF8, 0x71, 0x71, 255}
	calZone   = color.RGBA{0x0E, 0x2A, 0x22, 255}
	calThr    = color.RGBA{0x2D, 0xD4, 0xBF, 255}
	calDrift  = color.RGBA{0xE6, 0xED, 0xF3, 255}
	calChrome = color.RGBA{0x8B, 0x94, 0x9E, 255}
	calAccC   = color.RGBA{0x60, 0xA5, 0xFA, 255}
	calGyroC  = color.RGBA{0xC0, 0x84, 0xFC, 255}
)

// The page's texts (7-bit ASCII: the body font has no "…" or "—").
const (
	calInstr    = "Place it flat, or on its side. Hold it still."
	calWaitText = "Waiting for it to settle..."
	calMeasText = "Measuring - keep still"
	calMoveText = "Moved - starting again"
	calDoneText = "Gyro measured (this pose)"
	// what a single pose did and didn't, under the ring: its result is
	// shown, not applied (the motion state learns its own gyro bias)
	calDoneNote = "shown, not used: the|faces calibrate the|accelerometer and gyro"
	calPrompt   = "keep still"
	calAgain    = "A: calibrate"
)

var (
	calAxisNames  = [6]string{"ax", "ay", "az", "gx", "gy", "gz"}
	calStateWords = [5]string{"", "waiting", "measuring", "moved", "calibrated"}
	calChips      = [3]string{"ok", "drift", "noisy"}
)

// calibration runs the calibrator from power-on and tracks what its page
// shows. It lives as long as the App (the page is built around it).
type calibration struct {
	c        calibrator
	ring     *widget.Ring[float32] // filling
	ringDone *widget.Ring[float32] // the same, green: calibrated
	pct      float32               // the rings' value (0..100)
	stage    calStage
	start    time.Time // when this calibration started (its first update)
	movedAt  time.Time // the last time the calibrator reported a move
	applied  bool      // this calibration's result is in the motion state

	bands  [6]*widget.StatBand
	target [6]float32 // each axis's slow average: the strip's centre
	primed bool       // target holds a first mean
	face   calFace    // the face the strips' averages were primed on
	settle bool       // turned onto another face: re-prime until it lies still again
	state  [6]uint8   // 0 ok, 1 drift, 2 noisy (calChips; the band's colour)
	res    calResult  // the last completed calibration (its forecast on the done state)
	txt    [3]textBuf // the forecast's values and line
	g      guided     // the guided calibration on several faces
}

func newCalibration(c calibrator) *calibration {
	k := &calibration{c: c}
	s, err := calib.NewSolver(calib.DefaultConfig())
	if err != nil {
		panic("calib: " + err.Error()) // DefaultConfig is valid
	}
	k.g.solver = s
	style := widget.RingStyle{Track: calTrack, Fill: calFill, BG: colBG, Font: fontTitle, Label: colFG}
	opts := []widget.RingOption{widget.WithThickness(12), widget.WithCap(widget.RingCapRound), widget.WithPercentLabel()}
	k.ring = widget.NewRing(calRingD, calRingD, &k.pct, 0, 100, style, opts...)
	style.Fill = calOK
	k.ringDone = widget.NewRing(calRingD, calRingD, &k.pct, 0, 100, style, opts...)
	accT, gyroT := c.thresholds()
	for i := range k.bands {
		thr := accT
		if i >= 3 {
			thr = gyroT
		}
		// full scale 3× the threshold: the zone fills the middle third
		k.bands[i] = widget.NewStatBand(calTraceW, calTraceH, 3*thr, widget.StatBandStyle{
			BG: colBG, Zone: calZone, Threshold: calThr, Drift: calDrift,
			States: [3]color.RGBA{calOK, calWarnC, calBadC},
		}, widget.WithBandZone(thr))
	}
	return k
}

// add feeds one IMU sample to the calibrator.
func (k *calibration) add(acc, gyro [3]float32, dt float32) { k.c.add(acc, gyro, dt) }

// again starts a single-pose calibration afresh (the overlay's first
// choice): the page shows it; a guided result not used is dropped, a fit
// in use stays in use.
func (k *calibration) again() {
	k.c.reset()
	k.start, k.movedAt, k.applied = time.Time{}, time.Time{}, false
	k.g.stage = gOff
}

// update moves the page on to now (once per frame): the stage, and one
// column of every strip. It reports true once per calibration, when its
// result is ready to apply.
func (k *calibration) update(now time.Time) bool {
	k.pushStats()
	if k.start.IsZero() {
		k.start = now
	}
	p := k.c.phase()
	if p == calMoved {
		k.movedAt = now
	}
	if k.g.stage != gOff {
		k.updateGuided(now)
		return false
	}
	switch {
	case p == calDone:
		k.stage, k.pct = calShowDone, 100
		if !k.applied {
			k.applied = true
			k.res = k.c.result()
			return true
		}
	case !k.movedAt.IsZero() && now.Sub(k.movedAt) < calMovedFor:
		k.stage, k.pct = calShowMoved, 0
	case p == calMeasuring:
		k.stage, k.pct = calShowMeasuring, min(max(k.c.progress(), 0), 1)*100
	case now.Sub(k.start) < calStartupFor:
		k.stage, k.pct = calStartup, 0
	default:
		k.stage, k.pct = calShowWaiting, 0
	}
	return false
}

// pushStats adds this frame's column to every strip: the mean against its
// slow average (drift) and the spread, coloured by how they sit against
// the sensor's threshold.
func (k *calibration) pushStats() {
	accT, gyroT := k.c.thresholds()
	// turned onto another face, gravity is on another axis: the slow
	// averages start again there, or a turn reads as drift for seconds
	// (owner: "drift" on the strips after "Calibrated", standing it on an
	// edge)
	var f [3]float32
	for i := range f {
		f[i], _ = k.c.stats(i)
	}
	if face := faceOf(f); face != k.face {
		k.face, k.settle = face, true
	}
	if k.settle {
		// the averages follow the means until every axis is still: set down
		// on an edge, the last of the turn settles after the face changed
		k.primed = false
		still := true
		for i := range k.bands {
			thr := accT
			if i >= 3 {
				thr = gyroT
			}
			if _, spread := k.c.stats(i); spread >= thr {
				still = false
			}
		}
		k.settle = !still
	}
	for i, b := range k.bands {
		mean, spread := k.c.stats(i)
		if !k.primed {
			k.target[i] = mean
		}
		k.target[i] += (mean - k.target[i]) * calDriftK
		dev := mean - k.target[i]
		thr := accT
		if i >= 3 {
			thr = gyroT
		}
		var st uint8 // ok: the band inside the zone
		switch {
		case spread >= thr:
			st = 2 // noisy: a fat band
		case abs32f(dev)+spread >= thr:
			st = 1 // drift: thin, but off the centre
		}
		k.state[i] = st
		b.Push(dev, spread, st)
	}
	k.primed = true
}

func abs32f(v float32) float32 {
	if v < 0 {
		return -v
	}
	return v
}

// line is the sentence at the top and its colour.
func (k *calibration) line() (string, color.RGBA) {
	if k.guiding() {
		return k.guidedLine()
	}
	switch k.stage {
	case calShowWaiting:
		return calWaitText, colMuted
	case calShowMeasuring:
		return calMeasText, colFG
	case calShowMoved:
		return calMoveText, calWarnC
	case calShowDone:
		return calDoneText, calOK
	}
	return calInstr, colMuted
}

// guiding reports whether the page shows the guided calibration.
func (k *calibration) guiding() bool { return k.g.stage != gOff }

// pageCalib is the Calibration page: one view, which A acts on (act) and,
// on a guided result, B too (back).
func (a *App) pageCalib() *page {
	view := &calibView{WidgetBase: ui.NewWidgetBase(innerW, bodyH-12), k: a.calib, press: a.calibAct,
		onB: func() bool {
			if s := a.calib.g.stage; s != gResult && s != gRefused {
				return false
			}
			a.calib.discard(a.now)
			a.log("calibration discarded")
			return true
		}}
	return a.newPageWith("Calib", layout.VList(0), view)
}

// calibAct is A on the Calibration page: on a guided result it uses the
// fit; otherwise it offers the three calibrations (the highlight on
// 3 faces).
func (a *App) calibAct() {
	if a.calib.g.stage == gResult {
		a.calib.useFit()
		a.log2("calibration in use: ", a.calib.g.fit.Model.String())
		return
	}
	a.overlay.OpenCentered(a.calDialog)
	a.calDialog.Interact(ui.DOWN)
}

// calibView draws the page: the state line, the ring with the percentage
// and the state word, and the six strips with their labels and chips. Fixed
// strings and ui.DrawText: no allocation per frame.
type calibView struct {
	ui.WidgetBase
	k     *calibration
	press func()
	onB   func() bool
	ctx   ui.ContextImpl    // the ring's context (fields: no allocation per draw)
	bctx  [6]ui.ContextImpl // the strips'
}

// activate is A on the page: it acts at once, with no edit mode, like a
// demo tile (App.enter).
func (v *calibView) activate() { v.press() }

// backs is B on the page: true when the page took it (a guided result
// discarded), so it doesn't go back a page.
func (v *calibView) backs() bool { return v.onB() }

func (v *calibView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	k := v.k
	s, c := k.line()
	ui.DrawText(d, fontBody, x0+(int16(v.Width)-textWidth(fontBody, s))/2, y0+9, s, c)
	fillRect(d, x0, y0+15, int16(v.Width), 1, calTrack)

	// the ring: the main cue
	ring := k.ring
	guided := k.guiding()
	if k.stage == calShowDone && !guided || guided && (k.g.stage >= gResult || k.g.stage == gFaceDone) {
		ring = k.ringDone
	}
	v.ctx = ui.NewContext(d, calRingD, calRingD, x0+calRingX, y0+calRingY)
	ring.Draw(&v.ctx)
	cx := x0 + calRingX + calRingD/2
	w := calStateWords[k.stage]
	if guided {
		w = ""
		if int(k.g.stage) < len(gWords) {
			w = gWords[k.g.stage]
		}
	}
	if w != "" {
		ui.DrawText(d, fontSmall, cx-textWidth(fontSmall, w)/2, y0+calRingY+calRingD/2+18, w, calChrome)
	}
	if guided {
		v.guidedPanel(d, x0, y0+calRingY+calRingD+6)
	} else if k.stage == calShowDone {
		v.doneNote(d, x0, y0+calRingY+calRingD+6)
	} else {
		ui.DrawText(d, fontBody, cx-textWidth(fontBody, calPrompt)/2, y0+calRingY+calRingD+22, calPrompt, calThr)
	}

	// the strips: a legend, then accelerometer and gyro, three each
	fillRect(d, x0+calPanelX-6, y0+20, 1, int16(v.Height)-20, calTrack)
	v.legend(d, x0+calPanelX, y0+27)
	// the strips plot the deviation from each axis's slow average against
	// the stillness threshold (the zone is ±thr): no absolute unit
	ui.DrawText(d, fontSmall, x0+calChipX-16, y0+27, "acc +-thr", calChrome)
	ui.DrawText(d, fontSmall, x0+calChipX-20, y0+calStripY(3)-4, "gyro +-thr", calChrome)
	fillRect(d, x0+calTraceX, y0+calStripY(3)-7, calTraceW-20, 1, calTrack)
	fillRect(d, x0+calPanelX-3, y0+calStripY(0), 2, calStripY(2)+calTraceH-calStripY(0), calAccC)
	fillRect(d, x0+calPanelX-3, y0+calStripY(3), 2, calStripY(5)+calTraceH-calStripY(3), calGyroC)
	for i, b := range k.bands {
		y := y0 + calStripY(i)
		ui.DrawText(d, fontBody, x0+calPanelX, y+calTraceH/2+4, calAxisNames[i], calChrome)
		v.bctx[i] = ui.NewContext(d, calTraceW, calTraceH, x0+calTraceX, y)
		b.Draw(&v.bctx[i])
		st := k.state[i]
		col := [3]color.RGBA{calOK, calWarnC, calBadC}[st]
		fillRect(d, x0+calChipX, y+calTraceH/2-5, 4, 4, col)
		ui.DrawText(d, fontSmall, x0+calChipX, y+calTraceH/2+6, calChips[st], col)
	}
}

// legend: "spread" (a band swatch), "drift" (a line), "zone" (shaded).
func (v *calibView) legend(d drivers.Displayer, x, y int16) {
	fillRect(d, x, y-5, 6, 5, calOK)
	ui.DrawText(d, fontSmall, x+8, y, "spread", calChrome)
	fillRect(d, x+36, y-3, 6, 1, calDrift)
	ui.DrawText(d, fontSmall, x+44, y, "drift", calChrome)
	fillRect(d, x+68, y-5, 6, 5, calZone)
	ui.DrawText(d, fontSmall, x+76, y, "zone", calChrome)
}

// The done state's forecast (owner: "calibration should be able to measure
// it for me (spread and drift) so I know what to expect"): what the
// Calibrator expects from motion, as two bars on fixed scales, each a solid
// drift segment and a lighter noise cap over the green zone up to its OK
// line, with the value beside it, and one line.
const (
	calFcBarX   = 22 // the bars' left edge (from the view's left)
	calFcBarW   = 64 // their length
	calFcBarH   = 6  // their height
	calFcHeadFS = 6  // heading scale, °/min
	calFcPosFS  = 10 // position scale, cm/min
)

// doneNote is the single pose's done panel with its top at y: what it set
// and what it didn't (calDoneNote, its lines split at |),
// the forecast's one line, and A.
func (v *calibView) doneNote(d drivers.Displayer, x0, y int16) {
	s, i := calDoneNote, 0
	for len(s) > 0 {
		j := 0
		for j < len(s) && s[j] != '|' {
			j++
		}
		panelLine(d, x0+4, y-2, i, s[:j], calChrome)
		if j < len(s) {
			j++
		}
		s, i = s[j:], i+1
	}
	t, c := v.forecastLine()
	panelLine(d, x0+4, y-2, i, t, c)
	ui.DrawText(d, fontBody, x0+4, y+54, calAgain, calThr)
}

// forecastLine is the forecast's sentence: the drift to expect and its
// verdict.
func (v *calibView) forecastLine() (string, color.RGBA) {
	f := v.k.res.forecast
	t := v.k.txt[2].reset().str("Expect ").fixed(f.PosDrift+f.PosNoise, 1).str("cm/m ").fixed(f.HeadingDrift+f.HeadingNoise, 1).str("d/m")
	if f.PosDrift+f.PosNoise <= f.PosOK && f.HeadingDrift+f.HeadingNoise <= f.HeadingOK {
		return t.str(": fine").String(), calOK
	}
	return t.str(": watch").String(), calWarnC
}

// forecast draws the expectation block with its top at y.
func (v *calibView) forecast(d drivers.Displayer, x0, y int16) {
	f := v.k.res.forecast
	ui.DrawText(d, fontSmall, x0+4, y+5, "expected", calChrome)
	headOK := v.fcBar(d, x0, y+9, "hd", f.HeadingDrift, f.HeadingNoise, f.HeadingOK, calFcHeadFS, &v.k.txt[0], "d/m")
	posOK := v.fcBar(d, x0, y+19, "pos", f.PosDrift, f.PosNoise, f.PosOK, calFcPosFS, &v.k.txt[1], "cm/m")
	t := v.k.txt[2].reset().str("Expect ").fixed(f.PosDrift+f.PosNoise, 1).str("cm/m ").fixed(f.HeadingDrift+f.HeadingNoise, 1).str("d/m")
	c := calOK
	if headOK && posOK {
		t.str(": fine")
	} else {
		t.str(": watch")
		c = calWarnC
	}
	ui.DrawText(d, fontSmall, x0+4, y+36, t.String(), c)
	ui.DrawText(d, fontBody, x0+4, y+48, calAgain, calThr)
}

// fcBar draws one forecast bar (drift + noise against ok on a 0..fs scale)
// with its label and value; it reports whether the total is within ok.
func (v *calibView) fcBar(d drivers.Displayer, x0, y int16, label string, drift, noise, ok, fs float32, txt *textBuf, unit string) bool {
	px := func(val float32) int16 { return int16(min(max(val/fs, 0), 1) * calFcBarW) }
	bx := x0 + calFcBarX
	ui.DrawText(d, fontSmall, x0+4, y+calFcBarH-1, label, calChrome)
	fillRect(d, bx, y, calFcBarW, calFcBarH, calTrack)
	fillRect(d, bx, y, px(ok), calFcBarH, calZone)
	total := drift + noise
	within := total <= ok
	col := calOK
	switch {
	case total > 2*ok:
		col = calBadC
	case !within:
		col = calWarnC
	}
	dw, tw := px(drift), px(total)
	fillRect(d, bx, y+1, dw, calFcBarH-2, col)
	fillRect(d, bx+dw, y+1, tw-dw, calFcBarH-2, blend40(col, calZone)) // the noise cap, lighter
	fillRect(d, bx+px(ok), y-1, 1, calFcBarH+2, calThr)                // the OK line
	ui.DrawText(d, fontSmall, bx+calFcBarW+3, y+calFcBarH-1, txt.reset().fixed(total, 1).str(unit).String(), col)
	return within
}

// blend40 is c at 40 % over bg.
func blend40(c, bg color.RGBA) color.RGBA {
	m := func(a, b uint8) uint8 { return uint8((uint16(a)*2 + uint16(b)*3) / 5) }
	return color.RGBA{m(c.R, bg.R), m(c.G, bg.G), m(c.B, bg.B), 255}
}
