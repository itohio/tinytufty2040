package app

import (
	"image/color"
	"math"

	"github.com/itohio/EasyRobot/x/math/vec"
	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
	"tinygo.org/x/drivers"
)

// The Motion page (only with an IMU): the path the badge has moved along in
// the real world (the shared motion state's world position), seen from the
// badge (owner 2026-11-13: "you integrate real world path, then mahoney gives
// orientation and orientation is used to rotate the view. the pov is not at
// 0.0.0 ... but a bit further, so that more of the path could be visible"):
// a perspective camera at the latest position, set back along the view axis
// (into the screen, the way you look) far enough to see the whole path, and
// turned with the badge, so turning the badge turns the path around where
// you are. A small faint inset shows the world's axes as the camera sees
// them.

const (
	pathPoints  = 128  // path points kept (a new one per millimetre moved)
	pathStepMM  = 1    // mm between recorded points
	pathBackMin = 0.15 // m: the camera's least set-back behind the latest position
	pathBackMax = 3    // m
	pathBackK   = 1.6  // set-back per metre of the path's reach from the latest position
	pathNearMM  = 10   // mm: nearer than this to the camera is not drawn
	pathFocal   = 0.8  // focal length, of the view's smaller side
	insetSize   = 52   // px: the axes inset's square
	insetScale  = 0.36 // inset axis length, of insetSize
)

// the inset's colours: faint (owner: axes "LESS prominent")
var (
	axisColors = [3]color.RGBA{{110, 55, 55, 255}, {55, 100, 62, 255}, {55, 75, 115, 255}}
	pathScale  = color.RGBA{60, 66, 78, 255}
)

// pathTrack is the recorded world path, shared by the page's and the
// full-screen views: a new point per millimetre moved.
type pathTrack struct {
	m    *motion
	pts  [pathPoints][3]int16 // millimetres (±32 m): half the RAM of float32 metres
	n, i int
}

// record adds the badge's position when it has moved a millimetre from the
// last point (once a frame). Compared in the stored millimetres: compared
// with the metres, the rounding alone exceeded the step at rest and every
// frame stored the same point again, pushing the path out of the ring.
func (p *pathTrack) record() {
	mm := toMM(p.m.pos)
	if p.n > 0 {
		l := p.pts[(p.i-1+pathPoints)%pathPoints]
		dx, dy, dz := int32(mm[0])-int32(l[0]), int32(mm[1])-int32(l[1]), int32(mm[2])-int32(l[2])
		if dx*dx+dy*dy+dz*dz < pathStepMM*pathStepMM {
			return
		}
	}
	p.pts[p.i] = mm
	p.i = (p.i + 1) % pathPoints
	p.n = min(p.n+1, pathPoints)
}

// toMM converts a position in metres to the stored millimetres (rounded;
// clamped to ±32.767 m).
func toMM(p [3]float32) (mm [3]int16) {
	for i, v := range p {
		mm[i] = int16(math.Round(float64(min(max(v*1000, -32767), 32767))))
	}
	return mm
}

// reset starts a new path here.
func (p *pathTrack) reset() {
	p.m.resetPath()
	p.n, p.i = 0, 0
}

type pathView struct {
	ui.WidgetBase
	t    *pathTrack
	full bool // the full-screen view: it shows the settling notice
	// this frame's camera (set by Draw; fields, not a closure: no
	// allocation): the attitude, the camera's world position, the view's
	// centre on screen, the focal length in pixels and the view's rectangle
	// (lines are clipped to it)
	q      vec.Quaternion
	cam    [3]float32
	ox, oy int16
	f      int32 // px
	clip   viewRect
	// the camera for the integer projection (the RP2040 has no FPU): its
	// axes in the world (Q14) and its position (mm)
	ax, ay, az [3]int32
	camMM      [3]int32
}

// viewRect is a rectangle on the screen: x, y, width, height.
type viewRect struct{ x, y, w, h int16 }

// setCamera places this frame's camera: at the latest position, set back
// along the view axis (the badge's −Z: into the screen) by back metres.
func (v *pathView) setCamera(back float32) {
	m := v.t.m
	v.q = m.ahrs.Output()
	look := toWorld(v.q, [3]float32{0, 0, -1})
	p := m.pos
	v.cam = [3]float32{p[0] - back*look[0], p[1] - back*look[1], p[2] - back*look[2]}
	v.ax = toQ14(toWorld(v.q, [3]float32{1, 0, 0}))
	v.ay = toQ14(toWorld(v.q, [3]float32{0, 1, 0}))
	v.az = toQ14(look)
	for i, c := range v.cam {
		v.camMM[i] = int32(min(max(c*1000, -1<<20), 1<<20))
	}
}

// project maps a recorded point (mm) to the screen through this frame's
// camera, in integers; ok is false for a point behind it (or too near). The
// coordinates aren't limited to the view (a point near the camera's plane
// lands far off it): line clips them.
func (v *pathView) project(mm [3]int16) (x, y int32, ok bool) {
	var d [3]int32 // from the camera, mm (±32 m: the products fit int32)
	for i := range 3 {
		d[i] = min(max(int32(mm[i])-v.camMM[i], -32767), 32767)
	}
	depth := dotQ(d, v.az) // the camera looks along the badge's −Z
	if depth < pathNearMM {
		return 0, 0, false
	}
	return int32(v.ox) + v.f*dotQ(d, v.ax)/depth, int32(v.oy) - v.f*dotQ(d, v.ay)/depth, true
}

// back is the camera's set-back for the path now: far enough that the whole
// path, as far as it reaches from the latest position, fits the view.
func (v *pathView) back() float32 {
	t := v.t
	p := toMM(t.m.pos)
	var reach int64 // mm²
	for k := range t.n {
		reach = max(reach, dist2MM(t.pts[k], p))
	}
	return min(max(pathBackK*float32(math.Sqrt(float64(reach)))/1000, pathBackMin), pathBackMax)
}

// dist2MM is the squared distance between two recorded points (mm²).
func dist2MM(a, b [3]int16) int64 {
	var s int64
	for i := range 3 {
		d := int64(a[i]) - int64(b[i])
		s += d * d
	}
	return s
}

func (v *pathView) Draw(ctx ui.Context) {
	d := ctx.D()
	x0, y0 := ctx.DisplayPos()
	w, h := int16(v.Width), int16(v.Height)
	fillRect(d, x0, y0, w, h, boidsViewBG)
	t := v.t
	if v.full {
		drawSettling(d, t.m, x0, y0, w, h)
	}
	back := v.back()
	v.setCamera(back)
	v.ox, v.oy = x0+w/2, y0+h/2
	v.f = int32(pathFocal * float32(min(w, h)))
	v.clip = viewRect{x0, y0, w, h}
	// scale: 10 cm at the latest position's distance, bottom left
	bar := int16(float32(v.f) * 0.10 / back)
	fillRect(d, x0+6, y0+h-6, min(bar, w-12), 1, pathScale)
	ui.DrawText(d, fontBody, x0+8, y0+h-9, "10 cm", pathScale) // tinyfont.WriteLine allocates
	// the path, oldest dimmest, ending at the badge (the view's centre)
	var lx, ly int32
	lok := false
	for k := range t.n {
		x, y, ok := v.project(t.pts[(t.i-t.n+k+pathPoints)%pathPoints])
		if ok && lok {
			f := uint8(60 + k*195/t.n)
			line(d, v.clip, lx, ly, x, y, color.RGBA{f / 3, f, f, 255})
		}
		lx, ly, lok = x, y, ok
	}
	if lok {
		line(d, v.clip, lx, ly, int32(v.ox), int32(v.oy), colFG)
	}
	fillRect(d, v.ox-2, v.oy-2, 5, 5, colFG) // the badge, now
	v.drawInset(d, x0+w-insetSize-2, y0+2)
}

// drawInset draws the world's axes (x red, y green, z blue: up) as the camera
// sees them, small and faint, in an insetSize square at (x, y).
func (v *pathView) drawInset(d drivers.Displayer, x, y int16) {
	const s = insetSize * insetScale
	cx, cy := int32(x+insetSize/2), int32(y+insetSize/2)
	for i := range 3 {
		var ax [3]float32
		ax[i] = 1
		b := toBadge(v.q, ax)
		line(d, v.clip, cx, cy, cx+int32(b[0]*s), cy-int32(b[1]*s), axisColors[i])
	}
}

// outcode is where (x, y) lies against a rectangle: 1 left, 2 right, 4
// above, 8 below; 0 inside.
func outcode(x, y, xmin, ymin, xmax, ymax int32) (k uint8) {
	if x < xmin {
		k |= 1
	} else if x > xmax {
		k |= 2
	}
	if y < ymin {
		k |= 4
	} else if y > ymax {
		k |= 8
	}
	return k
}

// line draws the part of the segment (x0, y0)–(x1, y1) inside r, one pixel
// wide, all in integers. It is clipped first (Cohen–Sutherland; the
// crossing in int64) and then drawn (Bresenham): a runaway position projects
// the path's points tens of thousands of pixels off the view, and an int16
// Bresenham over such a segment overflowed (2·err) and never ended: the
// Motion page froze (TG-6).
func line(d drivers.Displayer, r viewRect, x0, y0, x1, y1 int32, c color.RGBA) {
	xmin, ymin := int32(r.x), int32(r.y)
	xmax, ymax := xmin+int32(r.w)-1, ymin+int32(r.h)-1
	k0, k1 := outcode(x0, y0, xmin, ymin, xmax, ymax), outcode(x1, y1, xmin, ymin, xmax, ymax)
	for range 8 { // each pass puts one end on an edge: 4 per end at most
		if k0|k1 == 0 {
			break
		}
		if k0&k1 != 0 {
			return // wholly outside
		}
		k := k0
		if k == 0 {
			k = k1
		}
		dx, dy := int64(x1-x0), int64(y1-y0)
		var x, y int32
		switch {
		case k&4 != 0:
			x, y = x0+int32(dx*int64(ymin-y0)/dy), ymin
		case k&8 != 0:
			x, y = x0+int32(dx*int64(ymax-y0)/dy), ymax
		case k&1 != 0:
			x, y = xmin, y0+int32(dy*int64(xmin-x0)/dx)
		default:
			x, y = xmax, y0+int32(dy*int64(xmax-x0)/dx)
		}
		if k == k0 {
			x0, y0, k0 = x, y, outcode(x, y, xmin, ymin, xmax, ymax)
		} else {
			x1, y1, k1 = x, y, outcode(x, y, xmin, ymin, xmax, ymax)
		}
	}
	if k0|k1 != 0 {
		return
	}
	ex, ey := x1-x0, y1-y0
	sx, sy := int32(1), int32(1)
	if ex < 0 {
		ex, sx = -ex, -1
	}
	if ey < 0 {
		ey, sy = -ey, -1
	}
	err := ex - ey
	for {
		d.SetPixel(int16(x0), int16(y0), c)
		if x0 == x1 && y0 == y1 {
			return
		}
		e2 := 2 * err
		if e2 > -ey {
			err -= ey
			x0 += sx
		}
		if e2 < ex {
			err += ex
			y0 += sy
		}
	}
}

// motionDemo is the Motion page's demo: it records the path each frame and
// shows it full screen.
type motionDemo struct {
	track      pathTrack
	page, full *pathView
	fullOn     bool
}

func newMotionDemo(m *motion) *motionDemo {
	md := &motionDemo{track: pathTrack{m: m}}
	md.page = &pathView{WidgetBase: ui.NewWidgetBase(192, 144), t: &md.track}
	md.full = &pathView{WidgetBase: ui.NewWidgetBase(ScreenW, ScreenH), t: &md.track, full: true}
	return md
}

func (m *motionDemo) tick() { m.track.record() }

func (m *motionDemo) fullView() ui.Widget {
	if m.fullOn {
		return m.full
	}
	return nil
}

// fullPress: A starts a new path here, B back to the page, C leaves.
func (m *motionDemo) fullPress(b Button) bool {
	switch b {
	case BtnA:
		m.track.reset()
	case BtnB:
		m.fullOn = false
	case BtnC:
		m.fullOn = false
		return false
	}
	return true
}

func (a *App) pageMotion() *page {
	md := a.motionDemo
	m := a.motion
	deg := func(r float32) int { return int(math.Round(float64(r) * 180 / math.Pi)) }
	cm := func(v float32) int { return int(math.Round(float64(v) * 100)) }
	full := widget.NewToggle(44, 12, fontBody, colFG, "SHOW", "SHOW", colTrack, colTrack,
		func() bool { return false }, func(bool) { md.fullOn = true; a.log("motion full screen") })
	reset := widget.NewToggle(44, 12, fontBody, colFG, "HERE", "HERE", colTrack, colTrack,
		func() bool { return false }, func(bool) { md.track.reset(); a.log("motion path reset") })
	p := a.newPageWith("Motion", layout.VFlow(4, 144),
		still(md.page),
		newField("Full", 34, 104, full),
		newField("Path", 34, 104, reset),
		still(a.text(104, func() string {
			return m.txt[0].reset().str("x ").int(cm(m.pos[0])).str(" y ").int(cm(m.pos[1])).str(" z ").int(cm(m.pos[2])).str(" cm").String()
		}, colMuted)),
		still(a.text(104, func() string {
			return m.txt[1].reset().str("roll ").int(deg(m.rpy[0])).str(" pitch ").int(deg(m.rpy[1])).String()
		}, colMuted)),
		still(a.text(104, func() string { return m.txt[2].reset().str("yaw ").int(deg(m.rpy[2])).String() }, colMuted)),
		still(a.text(104, func() string {
			t := m.txt[4].reset()
			if m.settling() {
				return t.str("settling").String()
			}
			if m.nav.Still {
				return t.str("at rest").String()
			}
			return t.str("moving").String()
		}, colMuted)),
	)
	p.demo = md
	return p
}
