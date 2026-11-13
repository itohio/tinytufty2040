// Package app is the Tufty 2040 showcase UI, written against TinyGUI only.
//
// It has no board dependencies: the badge firmware (../main_tufty2040.go)
// supplies a framebuffer displayer and a Hardware implementation; the host
// simulator (../sim) supplies fakes and writes PNG screenshots.
package app

import (
	"image/color"
	"time"
	"unsafe"

	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/animation"
	"github.com/itohio/tinygui/container"
	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Hardware is everything the UI reads from or drives on the board.
type Hardware interface {
	Light() uint16 // raw light-sensor ADC value, 0..65535
	Battery() (volts float32, usb bool)
	SetLED(percent int) // 0 = off
	SetBacklight(percent int)
}

// Config injects board-only helpers and output.
type Config struct {
	// DurationAdd/Sub step a duration by its own magnitude (ui.DurationAdd on
	// TinyGo targets; the host simulator passes an equivalent).
	DurationAdd, DurationSub func(d, delta time.Duration) time.Duration
	// Log mirrors log lines, e.g. to the USB serial console. May be nil.
	Log func(line string)
	// IMU: a motion sensor feeds SetIMU; adds the Motion page (owner: the
	// Mahony page only when there is an IMU).
	IMU bool
	// BuildID names the firmware (git commits, dirty flag, build time):
	// `version` and `stats` report it (tools/build sets it).
	BuildID string
}

// Button is a physical badge button.
type Button uint8

const (
	BtnUp Button = iota
	BtnDown
	BtnA
	BtnB
	BtnC
	BtnUser
)

// Every press is logged (Log page and serial console). Serial-injected and
// physical presses enter through the same Press call and log the same line.
var (
	pressLog     = [...]string{"press up", "press down", "press a", "press b", "press c", "press user"}
	pressLongLog = [...]string{"press up long", "press down long", "press a long", "press b long", "press c long", "press user long"}
)

type page struct {
	name string
	root *container.Base[ui.Widget]
	nav  *ui.Navigator
	demo demo // nil for pages that don't animate
}

// App holds the model, the widget tree and the input state.
type App struct {
	d   ui.Displayer
	hw  Hardware
	cfg Config

	ctx     ui.ContextImpl
	tabsCtx ui.ContextImpl
	footCtx ui.ContextImpl
	root    *container.Base[ui.Widget]
	tabs    *tabBar
	// pages: every page built in setup (container.Pages.BuildAll), so showing
	// one allocates nothing (owner: no allocations after setup); built[i] is
	// page i, demoOf[i] its demo
	pages    *container.Pages
	built    []*page
	demoOf   []demo
	footer   *widget.Label
	cur      int
	editing  bool
	saver    bool
	saverCtx ui.RandomContext
	saverW   ui.Widget
	saverPos ui.Context // RandomContext clone taken once per frame in Update

	idleTimeout time.Duration

	// model
	start, now, lastSample time.Time
	light, battPct, vbat   float32
	lightRaw               uint16
	lightFilt              lightFilter // median, log scale, EMA (light.go)
	lightShown             int         // the readout: light %, moving by whole steps
	usb                    bool
	presses                [6]int
	ledOn                  bool
	ledPct, backlight      int
	samplePeriod           time.Duration
	level, gain            float32
	mix                    []float32
	accent                 int
	pngIdx                 int
	bitmapIdx              int
	menu                   *menu
	optLabels              []string // the Demos options dialog's labels (first: Pause/Resume all)
	optDialog, resetDialog *widget.Dialog
	aboutDialog            *widget.Dialog
	motion                 *motion // IMU fusion; nil without an IMU
	// demo models: they outlive their pages (the Demos page's tiles run them)
	life                   *life
	water                  *water
	fire                   *fire
	pong                   *pong
	boids                  *boids
	motionDemo             *motionDemo
	tilters                []tilter
	lightForce             int // light <raw>: forced sensor reading, -1: the real sensor
	health                 health
	txtStat                textBuf
	allocU, allocD, allocP int     // allocations in the last frame (ReportAllocs)
	tiltDir                int     // gravity from the motion sensor (SetTilt)
	tiltX, tiltY           float32 // the last in-plane gravity (screen x right, y down; m/s²), the tilt command
	tilted                 bool    // SetTilt has given a direction
	solid                  int     // grey level of a uniform test screen, -1: off
	overlay                *container.Overlay
	imagePage              int       // index of the Image page (the plasma animates only there)
	full                   ui.Widget // a demo drawn full screen instead of the UI
	fullCtx                ui.ContextImpl
	speedMS                int

	curves     [4]float32
	curveAnims [4]animation.Animator
	curveFrom  [4][1]float32
	curveTo    [4][1]float32

	plasma   [plasmaW * plasmaH]uint16
	gradient [plasmaW * plasmaH]uint8
	phase    int

	// widgets the app updates
	spark      *sparkline
	battGauge  *widget.VolumeGauge[float32]
	lightGauge *widget.SolidGauge[float32]
	hGauge     *widget.Gauge[float32]
	multi      *widget.MultiGauge[float32]
	plasmaBmp  *widget.Bitmap16
	logView    *widget.InteractiveLog
	info       *widget.MultilineLabel
	infoLines  [2]string

	// stats
	frames         int
	fps            int
	fpsT           time.Time
	drawUS, pushUS int64
	nodes          int

	logRing [logCapacity + 1][logLineLen]byte
	logNext int

	cmds    chan command
	dumpReq bool

	// zero-allocation text buffers, one per live label
	txtUptime, txtLight, txtBatt, txtFrame, txtFoot textBuf
	txtSpeed, txtLED, txtBL, txtGain, txtSel        textBuf
	txtSaver, txtInfo, txtLog                       textBuf
	listNames, listVals                             [listRows]textBuf
}

const (
	// the Image page's Bitmap16 plasma and Bitmap8 gradient: 48×30 (2.9 KB
	// and 1.4 KB); 64×40 cost 3.4 KB more of a heap that needs 30 KB free
	plasmaW = 48
	plasmaH = 30
	// how far B can go back through the pages shown before
	pagesHistory = 8
)

// initDemos makes the demo models and the Demos page's tiles that run them:
// they live as long as the App, their pages are built around them on demand.
func (a *App) initDemos(cfg Config) {
	imu := cfg.IMU
	// one liquid engine for Water and Fire's fuel (fluid.go): Fire first, so
	// Water holds it at boot
	share := &liquidShare{fl: newFlipFluid()}
	a.fire = newFire(share)
	a.life, a.water, a.pong, a.boids = newLife(), newWater(share), newPong(), newBoids()
	a.life.log2, a.water.log2, a.fire.log2 = a.log2, a.log2, a.log2
	a.pong.log, a.boids.log = a.log, a.log
	a.tilters = []tilter{a.water, a.fire} // the boids follow the whole motion state (applyMotion)
	a.menu = a.newMenu([]string{"Life", "Water", "Fire", "Pong", "Boids"},
		[]previewer{a.life, a.water, a.fire, a.pong, a.boids})
	if imu {
		a.motion = newMotion()
		a.motionDemo = newMotionDemo(a.motion)
		a.pong.motion = a.motion // your paddle follows the badge up and down
	}
}

// New builds the whole UI. d must stay valid for the app's lifetime.
func New(d ui.Displayer, hw Hardware, cfg Config, now time.Time) *App {
	a := &App{
		d: d, hw: hw, cfg: cfg,
		start: now, now: now, fpsT: now,
		idleTimeout:  30 * time.Second,
		ledPct:       40,
		backlight:    80,
		samplePeriod: 200 * time.Millisecond,
		level:        65,
		mix:          []float32{20, 30, 25},
		speedMS:      1500,
		cmds:         make(chan command, 16),
		solid:        -1,
		lightForce:   -1,
	}
	if _, usb := hw.Battery(); usb {
		// on USB the badge is usually on the desk under the camera: full
		// backlight is DC, no PWM to beat with the camera's rolling shutter
		a.backlight = 100
	}
	a.restartCurves()
	a.initLive()
	a.initDemos(cfg)
	// The pages: all built at the end of setup (BuildAll below).
	type pageDef struct {
		name  string
		build func() *page
		demo  demo
	}
	defs := []pageDef{
		{"Demos", a.pageDemos, a.menu},
		{"Dash", a.pageDashboard, nil}, {"Gauge", a.pageGauges, nil}, {"Ctrl", a.pageControls, nil},
		{"Image", a.pagePictures, nil}, {"Flow", a.pageLayouts, nil}, {"List", a.pageList, nil}, {"Log", a.pageLog, nil},
		{"Life", a.pageLife, a.life}, {"Water", a.pageWater, a.water}, {"Fire", a.pageFire, a.fire},
		{"Pong", a.pagePong, a.pong}, {"Boids", a.pageBoids, a.boids},
	}
	if cfg.IMU {
		defs = append(defs, pageDef{"Motion", a.pageMotion, a.motionDemo})
	}
	names := make([]string, len(defs))
	specs := make([]container.PageSpec, len(defs))
	a.built = make([]*page, len(defs))
	a.demoOf = make([]demo, len(defs))
	for i, d := range defs {
		names[i] = d.name
		a.demoOf[i] = d.demo
		specs[i] = container.PageSpec{Name: d.name, Build: func() ui.Widget {
			p := d.build()
			p.demo = d.demo
			a.built[i] = p
			return p.root
		}}
	}
	a.pages = container.NewPages(ScreenW, bodyH, specs, container.WithHistory(pagesHistory)) // default: every page kept
	a.imagePage = a.pageIndex("Image")
	a.tabs = newTabBar(ScreenW, tabsH, names)
	a.footer = widget.NewLabel(ScreenW-8, footerH-3, fontBody, a.footerText, colMuted)
	a.root = container.New[ui.Widget](ScreenW, ScreenH,
		container.WithLayout[ui.Widget](layout.VList(0)),
		container.WithChildren[ui.Widget](a.tabs, a.pages, a.footer),
	)
	a.overlay = container.NewOverlay(a.root)
	a.ctx = ui.NewContext(d, ScreenW, ScreenH, 0, 0)
	a.tabsCtx = ui.NewContext(d, ScreenW, tabsH, 0, 0)
	a.footCtx = ui.NewContext(d, ScreenW, footerH, 4, ScreenH-footerH)

	a.saverCtx = ui.NewRandomContext(d, 2*time.Second, 150, 40)
	a.saverW = newHeading(150, 30, func() string {
		s := int(a.now.Sub(a.start).Seconds())
		return a.txtSaver.reset().str("tinygui ").pad2(s / 60 % 60).str(":").pad2(s % 60).String()
	}, &colAccent)

	a.initMenuDialogs()
	a.pages.BuildAll() // every page now: nothing is built (allocated) later
	a.showPage(0)
	a.applyLED()
	hw.SetBacklight(a.backlight)
	a.log("tinygui showcase ready")
	return a
}

// Update advances time-driven state: sensors, animations, idle timeouts and
// queued commands. Call it once per frame before Draw.
func (a *App) Update(now time.Time) {
	a.now = now
	for {
		select {
		case c := <-a.cmds:
			a.exec(c)
			continue
		default:
		}
		break
	}
	us := now.UnixMicro()
	if now.Sub(a.lastSample) >= a.samplePeriod {
		a.lastSample = now
		a.lightRaw = a.hw.Light()
		if a.lightForce >= 0 { // the light command: a reading forced for tests
			a.lightRaw = uint16(a.lightForce)
		}
		a.light = a.lightFilt.add(a.lightRaw)
		a.lightShown = steadyPct(a.lightShown, a.light)
		a.vbat, a.usb = a.hw.Battery()
		a.battPct = clampf((a.vbat-3.0)/(4.2-3.0)*100, 0, 100)
		a.spark.Push(a.light)
	}
	done := true
	for i, an := range a.curveAnims {
		if !an.Update(a.curves[i:i+1], us) {
			done = false
		}
	}
	if done { // ping-pong
		a.curveFrom, a.curveTo = a.curveTo, a.curveFrom
		for i, an := range a.curveAnims {
			an.Start(a.curveFrom[i][:], a.curveTo[i][:], us)
		}
	}
	a.tabs.tick(us)
	if a.saver {
		a.saverPos = a.saverCtx.Clone(a.saverW, 150, 40)
	}
	a.phase = (a.phase + 3) & 255
	a.full = nil
	if d := a.demoOf[a.cur]; d != nil {
		d.tick()
		a.full = d.fullView()
	}
	if a.full != nil {
		w, h := a.full.Size()
		a.fullCtx = ui.NewContext(a.d, ScreenW, ScreenH, int16(ScreenW-int(w))/2, int16(ScreenH-int(h))/2)
	}
	if a.cur == a.imagePage { // only animate the plasma while it is visible
		renderPlasma(a.plasma[:], plasmaW, plasmaH, a.phase)
	}
	if !a.editing {
		a.page().root.Interact(ui.IDLE) // container idle timeout clears focus
	}

	a.frames++
	if dt := now.Sub(a.fpsT); dt >= time.Second {
		a.fps = int(int64(a.frames) * int64(time.Second) / int64(dt))
		a.sampleHeap()
		a.frames = 0
		a.fpsT = now
		a.infoLines[0] = a.txtInfo.reset().str("Navigator depth ").int(a.page().nav.Depth()).
			str(", pages built ").int(a.pages.Built()).String()
		a.infoLines[1] = "Serial: send 'help'. USER hold >1s: screensaver."
		a.info.SetLines(a.infoLines[:])
	}
}

// Draw renders the frame into the displayer (call Display afterwards). With a
// banded framebuffer the board calls Draw once per band; Draw must then paint
// the same frame every time, so all per-frame state changes live in Update.
func (a *App) Draw() {
	if a.solid >= 0 { // `solid <level>`: the whole screen one grey, nothing else
		v := uint8(a.solid)
		a.d.FillScreen(color.RGBA{v, v, v, 255})
		return
	}
	a.d.FillScreen(colBG)
	if a.saver {
		a.saverW.Draw(a.saverPos)
	} else if a.full != nil {
		a.full.Draw(&a.fullCtx)
	} else {
		a.root.Draw(&a.ctx)
		// TinyGUI does not clip: scrolled list rows can spill over the
		// chrome, so the tab bar and footer are painted again on top.
		a.tabs.Draw(&a.tabsCtx)
		fillRect(a.d, 0, ScreenH-footerH, ScreenW, footerH, colBG)
		a.footer.Draw(&a.footCtx)
		a.overlay.DrawLayers(&a.ctx) // popups, dialogs, dropdown lists on top
	}
}

// ReportAllocs records the heap allocations of the last frame's Update,
// Draw (all bands) and push; `stats` prints them.
func (a *App) ReportAllocs(update, draw, push int) {
	a.allocU, a.allocD, a.allocP = update, draw, push
}

// ReportTiming records how long the board spent drawing and pushing a frame.
func (a *App) ReportTiming(draw, push time.Duration) {
	a.drawUS, a.pushUS = draw.Microseconds(), push.Microseconds()
}

// TakeDump reports (once) that a framebuffer dump was requested over serial.
func (a *App) TakeDump() bool {
	r := a.dumpReq
	a.dumpReq = false
	return r
}

func (a *App) footerText() string {
	t := a.txtFoot.reset()
	switch {
	case a.saver:
		t.str("screensaver - any key")
	case a.overlay.Top() != nil:
		t.str("up/dn choose  A ok  B cancel")
	case a.editing:
		t.str("EDIT up/dn  A ok  B cancel")
	default:
		t.str("up/dn focus  A ok  B back  C page")
	}
	// heartbeat: seconds since boot (a frozen number means a stuck badge);
	// the hints are short so it fits the 48-byte footer text
	t.str("  ").int(a.fps).str("fps ").int(int(a.now.Sub(a.start) / time.Second)).str("s")
	if h := &a.health; h.memShow {
		t.str(" ").int(h.heapUsedKB).str("/").int(h.heapSysKB).str("K")
	}
	return t.String()
}

// ---- input ----

// Press handles a button press; long is a held or auto-repeated press.
func (a *App) Press(b Button, long bool) {
	a.presses[b]++
	if long {
		a.log(pressLongLog[b])
	} else {
		a.log(pressLog[b])
	}
	if a.saver {
		a.saver = false
		return
	}
	if a.full != nil && a.demoOf[a.cur].fullPress(b) {
		return
	}
	if a.overlay.Top() != nil { // a dialog or list is open: it gets the buttons
		if cmd, ok := overlayCmd[b]; ok {
			a.overlay.Interact(cmd)
		}
		return
	}
	p := a.page()
	// Base's idle timer only sees Interact calls, not navigator moves; any
	// non-navigation command (USER is ignored by containers) marks activity.
	p.root.Interact(ui.USER)
	switch b {
	case BtnC:
		if a.editing {
			a.cancelEdit()
		}
		step := 1
		if long {
			step = a.pages.Count() - 1
		}
		a.showPage((a.cur + step) % a.pages.Count())
	case BtnUp, BtnDown:
		if a.editing {
			cmd := ui.UP
			if b == BtnDown {
				cmd = ui.DOWN
			}
			if long {
				cmd += ui.LONG_UP - ui.UP
			}
			p.nav.Current().Interact(cmd)
			return
		}
		if b == BtnUp {
			p.nav.Prev()
		} else {
			p.nav.Next()
		}
	case BtnA:
		if a.editing {
			cur := p.nav.Current()
			if !cur.Interact(ui.ENTER) || !childSelected(cur) {
				a.endEdit()
			}
			return
		}
		a.enter(p)
	case BtnB:
		if a.editing {
			a.cancelEdit()
			return
		}
		if p.nav.Depth() == 1 && !p.root.Active() { // nothing to leave inside the page
			if p.demo == a.menu {
				a.menuOptions() // the home page: its options
			} else if a.pages.Back() {
				a.afterShow() // the page shown before (bounded history)
			}
			return
		}
		p.nav.Back()
	}
}

// UserHold reports how long the USER button was held (ui.PeekButton on the board).
func (a *App) UserHold(d time.Duration) {
	a.presses[BtnUser]++
	a.logf("user held ms", int(d/time.Millisecond))
	if d > time.Second {
		a.saver = !a.saver
	}
}

func (a *App) enter(p *page) {
	if p.root.Index() < 0 && p.nav.Depth() == 1 {
		// The idle timeout cleared the focus: the first A only brings the
		// focus back, so it never edits a row the user cannot see selected.
		p.nav.Focus(0)
		return
	}
	cur := p.nav.Current()
	if dd, ok := cur.(*widget.Dropdown); ok {
		dd.Open() // the list opens in the overlay and takes the buttons
		return
	}
	if t, ok := cur.(interface{ activate() }); ok {
		t.activate() // e.g. a demo tile: opens the demo full screen
		return
	}
	if f, ok := cur.(*field); ok {
		if _, isToggle := f.child.(*widget.Toggle); isToggle {
			f.Interact(ui.ENTER) // toggles act immediately, no edit mode
			return
		}
	}
	depth := p.nav.Depth()
	if !p.nav.Enter() {
		return
	}
	if p.nav.Depth() != depth {
		return // descended into a container (e.g. the list)
	}
	if f, ok := p.nav.Current().(*field); ok {
		if _, isLabel := f.child.(*widget.Label); isLabel {
			a.log3(p.name, ": picked ", f.Name()) // plain rows: select, no edit
			a.endEdit()
			return
		}
	}
	a.editing = true // a leaf editor was activated
}

func (a *App) endEdit() {
	p := a.page()
	top := a.top(p)
	idx := top.Index()
	p.nav.Back() // deactivates (OnDeactivate, OnExit) and clears the index
	p.nav.Focus(idx)
	a.editing = false
}

func (a *App) cancelEdit() {
	a.page().nav.Current().Interact(ui.ESC)
	a.endEdit()
}

// top returns the container at the top of the page's navigator stack.
func (a *App) top(p *page) ui.Navigable {
	path := p.nav.Path()
	for i := len(path) - 1; i >= 0; i-- {
		if n, ok := path[i].Widget.(ui.Navigable); ok {
			return n
		}
	}
	return p.root
}

// page is the page on screen (always built: container.Pages builds it).
func (a *App) page() *page { return a.built[a.cur] }

// pageIndex is the index of the page named name, -1 if there is none.
func (a *App) pageIndex(name string) int {
	for i := range a.pages.Count() {
		if a.pages.Name(i) == name {
			return i
		}
	}
	return -1
}

func (a *App) showPage(i int) {
	a.pages.Show(i)
	a.afterShow()
}

// afterShow brings the rest of the UI to the page container.Pages shows.
func (a *App) afterShow() {
	a.cur = a.pages.Index()
	// the last page's full-screen view is stale until the next Update: a
	// button in between went to the new page's fullPress (nil demo state:
	// crash; review R-TG-1 T1, e.g. console "c" then "a" in one Update)
	a.full = nil
	a.tabs.setActive(a.cur, a.now.UnixMicro())
	p := a.page()
	// Pages that were off screen still have the idle clock of their last
	// visit; mark activity so the next Update's IDLE doesn't clear the focus.
	p.root.Interact(ui.USER)
	if p.root.Index() < 0 && p.nav.Depth() == 1 {
		p.nav.Focus(0) // a page shown after an idle timeout starts focused
	}
	if c, ok := a.demoOf[a.cur].(interface{ claim() }); ok {
		c.claim() // Water or Fire shown: it takes the shared liquid engine
	}
	if u, ok := a.demoOf[a.cur].(interface{ unpin() }); ok {
		u.unpin() // a gravity picked on the last visit gives way to the badge's
	}
	a.log2("page ", p.name)
}

func childSelected(w ui.Widget) bool {
	if f, ok := w.(*field); ok {
		return f.child.Selected()
	}
	return w.Selected()
}

// ---- model actions ----

func (a *App) restartCurves() {
	us := a.now.UnixMicro()
	dur := int64(a.speedMS) * 1000
	a.curveAnims = [4]animation.Animator{
		animation.NewLinear(dur), animation.NewEaseIn(dur), animation.NewEaseOut(dur), animation.NewEaseInOut(dur),
	}
	for i, an := range a.curveAnims {
		a.curveFrom[i][0], a.curveTo[i][0] = 0, 100
		an.Start(a.curveFrom[i][:], a.curveTo[i][:], us)
	}
}

func (a *App) applyLED() {
	if a.ledOn {
		a.hw.SetLED(a.ledPct)
	} else {
		a.hw.SetLED(0)
	}
}

func (a *App) setAccent(i int) {
	colAccent = accents[i]
	a.lightGauge.Foreground = colAccent
	a.hGauge.Foreground = colAccent
	a.log2("accent ", accentNames[i])
}

// ---- log ----

// logCapacity is how many lines the Log page keeps; logRing has one spare
// slot so a line is never overwritten while the log still shows it.
const (
	logCapacity = 16 // lines kept (56 B each): the heap needs 30 KB free after setup (Leader 2026-11-13)
	logLineLen  = 56
)

// log copies line into a fixed ring of byte arrays and hands the Log widget a
// string view of the copy. The log's lines then live in one block of the App
// instead of as 64 small strings scattered over the heap, which fragmented
// it until the PNG decoder's ≈ 13 KB allocation no longer fit (badge OOM).
func (a *App) log(line string) { a.log3(line, "", "") }

func (a *App) log2(p1, p2 string) { a.log3(p1, p2, "") }

// log3 logs p1+p2+p3 without building the string: the parts are copied
// into the ring's slot (truncated to logLineLen).
func (a *App) log3(p1, p2, p3 string) {
	slot := &a.logRing[a.logNext]
	a.logNext = (a.logNext + 1) % len(a.logRing)
	n := copy(slot[:], p1)
	n += copy(slot[n:], p2)
	n += copy(slot[n:], p3)
	line := unsafe.String(&slot[0], n)
	a.logView.Append(line)
	if a.cfg.Log != nil {
		a.cfg.Log(line)
	}
}

func (a *App) logf(what string, v int) { a.log3(what, " ", a.txtLog.reset().int(v).String()) }

// navLogger is a ui.NavigatorObserver writing focus changes to the log.
type navLogger struct {
	a    *App
	page *page
}

func (n navLogger) OnNavigatorEvent(ev ui.NavigatorEvent) {
	w := ev.Path.Current()
	name := ""
	if f, ok := w.(*field); ok {
		name = f.Name()
	}
	switch ev.Type {
	case ui.NavigatorEventFocusChanged:
		if name != "" {
			n.a.log3(n.page.name, ": focus ", name)
		}
	case ui.NavigatorEventActivated:
		n.a.log3(n.page.name, ": edit ", name)
	case ui.NavigatorEventDeactivated:
		n.a.log2(n.page.name, ": done")
	}
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}

func clampf(v, lo, hi float32) float32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
