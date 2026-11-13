package app

import (
	"time"

	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/container"
	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// The Demos page: every demo runs live in a tile, drawn by its own view at
// 64×48 px, 128×96 for the focused tile, which grows, so the rows are laid
// out again (layout.Wrap with centred rows). A opens the focused demo full
// screen, B there comes back.

// previewer is a demo that can be shown in a tile.
type previewer interface {
	demo
	// tileViews are the tile's widgets: tileW×tileH, and twice that for
	// the focused tile.
	tileViews() (small, big ui.Widget)
	showFull()
	reset()
}

const (
	tileW, tileH = 64, 48
	tileBorder   = 2
	tileLabelH   = 12
)

type tile struct {
	ui.WidgetBase
	name           string
	d              previewer
	small, big     ui.Widget
	tickUS, drawUS int64
	drawAcc        int64 // summed over the bands of one frame
	open           func(*tile)
}

func newTile(name string, d previewer, open func(*tile)) *tile {
	t := &tile{name: name, d: d, open: open}
	t.small, t.big = d.tileViews()
	t.WidgetBase = ui.NewWidgetBase(t.sizeFor(false))
	return t
}

func (t *tile) sizeFor(big bool) (uint16, uint16) {
	g := t.small
	if big {
		g = t.big
	}
	w, h := g.Size()
	return w + 2*tileBorder, h + 2*tileBorder + tileLabelH
}

// SetSelected grows the focused tile; the page's arranger lays out again.
func (t *tile) SetSelected(s bool) {
	t.WidgetBase.SetSelected(s)
	t.Width, t.Height = t.sizeFor(s)
}

func (t *tile) activate() { t.open(t) }

func (t *tile) Draw(ctx ui.Context) {
	t0 := time.Now()
	d := ctx.D()
	x, y := ctx.DisplayPos()
	g := t.small
	frame := colTrack
	if t.Selected() {
		g, frame = t.big, colAccent
	}
	gw, gh := g.Size()
	w, h := int16(gw)+2*tileBorder, int16(gh)+2*tileBorder
	fillRect(d, x, y, w, tileBorder, frame)
	fillRect(d, x, y+h-tileBorder, w, tileBorder, frame)
	fillRect(d, x, y, tileBorder, h, frame)
	fillRect(d, x+w-tileBorder, y, tileBorder, h, frame)
	ctx.AddPos(tileBorder, tileBorder) // the arranger sets each child's position anew
	g.Draw(ctx)
	ctx.AddPos(-tileBorder, -tileBorder)
	c := colMuted
	if t.Selected() {
		c = colFG
	}
	ui.DrawText(d, fontBody, x+1, y+h+tileLabelH-3, t.name, c)
	t.drawAcc += time.Since(t0).Microseconds()
}

// menu is the Demos page's demo: it ticks every tile's demo, or only the one
// shown full screen.
type menu struct {
	tiles  []*tile
	full   *tile
	log3   func(p1, p2, p3 string) // App.log3: parts copied into the log, no string built
	speed  int                     // index into menuSpeeds
	paused bool
	frame  int
}

var menuSpeeds = []string{"speed 1/2", "speed x1", "speed x2"}

func (m *menu) tick() {
	if m.full != nil {
		m.full.d.tick()
		return
	}
	m.frame++
	steps := m.speed // 0: every other frame, 1: once, 2: twice
	if steps == 0 {
		steps = m.frame & 1
	}
	if m.paused {
		steps = 0
	}
	for _, t := range m.tiles {
		t0 := time.Now()
		for range steps {
			t.d.tick()
		}
		t.tickUS = time.Since(t0).Microseconds()
		t.drawUS, t.drawAcc = t.drawAcc, 0
	}
}

func (m *menu) fullView() ui.Widget {
	if m.full == nil {
		return nil
	}
	g := m.full.d.fullView()
	if g == nil { // the demo left full screen (B)
		m.full = nil
	}
	return g
}

func (m *menu) fullPress(b Button) bool { return m.full.d.fullPress(b) }

func (m *menu) open(t *tile) {
	t.d.showFull()
	m.full = t
	m.log3("demo ", t.name, " full screen")
}

// report prints each tile's frame budget on the console (`tiles`).
func (m *menu) report(log func(string), b *textBuf) {
	for _, t := range m.tiles {
		log(b.reset().str("tile ").str(t.name).str(" tick_us ").int(int(t.tickUS)).str(" draw_us ").int(int(t.drawUS)).String())
	}
}

// newMenu makes the Demos page's model: a tile per demo, running it live.
func (a *App) newMenu(names []string, demos []previewer) *menu {
	m := &menu{log3: a.log3, speed: 1}
	for i, d := range demos {
		m.tiles = append(m.tiles, newTile(names[i], d, m.open))
	}
	return m
}

// pageDemos builds the Demos page around the menu's tiles.
func (a *App) pageDemos() *page {
	m := a.menu
	children := make([]ui.Widget, 0, len(m.tiles)+1)
	for _, t := range m.tiles {
		children = append(children, t)
	}
	// a combo box among the tiles: its list opens over the live previews
	children = append(children, widget.NewDropdown(76, a.overlayStyle(), menuSpeeds, &m.speed,
		func(_ int, s string) { a.log2("demos ", s) }))
	p := a.newPageOpt("Demos", container.WithArranger[ui.Widget](&layout.Wrap{
		Gap: 4, Main: layout.Center, Cross: layout.Center, Block: layout.Center}), children...)
	p.demo = m
	return p
}

// ---- overlays on the Demos page ----

// overlayCmd maps the badge's buttons to commands for an open dialog or list.
var overlayCmd = map[Button]ui.UserCommand{BtnUp: ui.UP, BtnDown: ui.DOWN, BtnA: ui.ENTER, BtnB: ui.BACK}

func (a *App) overlayStyle() widget.OverlayStyle {
	return widget.OverlayStyle{Font: fontBody, LineH: 13, FG: colFG, BG: colPanel, Border: colMuted, Accent: colAccent}
}

// menuOptions is the Demos page's B: a choice question over the live tiles.
// initMenuDialogs builds the Demos options dialogs once, in setup: opening
// them later allocates nothing (owner: no allocations after setup).
func (a *App) initMenuDialogs() {
	m := a.menu
	// sized for the longer label; the first option reads Pause or Resume
	a.optLabels = []string{"Resume all", "Reset all...", "About", "Close"}
	a.optDialog = widget.NewDialog(a.overlayStyle(), "Demos", nil, a.optLabels, func(i int) {
		switch i {
		case 0:
			m.paused = !m.paused
			a.log2("demos ", a.optLabels[0])
		case 1:
			a.overlay.OpenCentered(a.resetDialog)
		case 2:
			a.overlay.OpenCentered(a.aboutDialog)
		}
	}, nil)
	a.resetDialog = widget.NewDialog(a.overlayStyle(), "Reset all demos?",
		[]string{"Life, water, fire, pong and", "boids start over."}, []string{"Reset", "Cancel"},
		func(i int) {
			if i == 0 {
				for _, t := range m.tiles {
					t.d.reset()
				}
				a.log("demos reset")
			}
		}, nil)
	a.aboutDialog = widget.NewDialog(a.overlayStyle(), "About",
		[]string{"tinygui on a Tufty 2040:", "five live demos in tiles,", "laid out by layout.Wrap.", "A or B closes this."}, nil, nil, nil)
}

// menuOptions opens the Demos options (built in initMenuDialogs).
func (a *App) menuOptions() {
	a.optLabels[0] = "Pause all"
	if a.menu.paused {
		a.optLabels[0] = "Resume all"
	}
	a.overlay.OpenCentered(a.optDialog)
}
