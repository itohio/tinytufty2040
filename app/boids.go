package app

import (
	ui "github.com/itohio/tinygui"
	"image/color"
	"math"
	"time"

	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Boids (Reynolds: separation, alignment, cohesion) in a 64×48-cell box,
// drawn in real pixels by boidsView. Positions and velocities are fixed
// point, 1/256 of a cell. Neighbours are found by a plain scan with an early
// reject on |dx| and |dy|: at most 128 boids, so a spatial grid isn't needed
// (the step time is shown on the page).
const (
	boidsW, boidsH = 64, 48
	boidsFix       = 256
	boidsMax       = 128
	boidsPitch     = 3
	boidsR         = 6 * boidsFix // neighbour radius
	boidsMargin    = 4 * boidsFix // walls push back inside this margin
	boidsVMax      = 128
	boidsVMin      = 56
	boidsTrail     = 3 // trail palette indices 1..3, heads from 4
	boidsHist      = 4 // positions kept per boid for the full-resolution trail
)

var (
	boidsCounts = []string{"32", "64", "128"}
	boidsN      = [3]int{32, 64, 128}
)

var boidsPalette = []color.RGBA{
	{14, 14, 22, 255},
	{22, 34, 46, 255}, {32, 60, 78, 255}, {50, 96, 120, 255}, // trail, fading
	// heads, one hue per heading octant
	{255, 90, 90, 255}, {255, 170, 60, 255}, {240, 240, 80, 255}, {120, 240, 90, 255},
	{70, 230, 200, 255}, {80, 160, 255, 255}, {160, 110, 255, 255}, {250, 100, 220, 255},
}

type boids struct {
	x, y, vx, vy [boidsMax]int32
	nvx, nvy     [boidsMax]int32
	hist         [boidsMax][boidsHist][2]int16 // recent positions (ring), from the box centre then (histC): int16 halves the 4 KB, wherever the box goes
	histC        [boidsHist][2]int32           // the box centre when each history slot was taken
	feltX, feltY float32                       // the apparent acceleration the flock feels (screen axes, m/s²; feel)
	histI        int
	count        int // index into boidsN
	pose         boxPose
	stepUS       int64
	grid, full   *boidsView
	tiles        [2]*boidsView
	fullOn       bool
	txt          textBuf
	log          func(string)
	// the window's travel in the boids' world per metre the badge moves,
	// at the shared motion sensitivity (sens.go), and where it was when
	// that last changed: the box's base then and the badge's displacement
	unitsPerM  float32
	anchorBase [2]int32
	anchorDisp [2]float32
}

func newBoids() *boids {
	b := &boids{count: 1, unitsPerM: cellsPerM(sensMM[sensDefault]) * boidsFix,
		anchorBase: [2]int32{boidsW * boidsFix / 2, boidsH * boidsFix / 2}}
	b.reset()
	b.grid = newBoidsView(b, boidsW*boidsPitch, boidsH*boidsPitch)
	b.full = newBoidsView(b, ScreenW, ScreenH)
	b.tiles = [2]*boidsView{newBoidsView(b, tileW, tileH), newBoidsView(b, 2*tileW, 2*tileH)}
	return b
}

func abs32(v int32) int32 {
	if v < 0 {
		return -v
	}
	return v
}

// limit keeps the speed between boidsVMin and boidsVMax, measured with the
// octagonal approximation max + 3/8 min (no square root).
func limit(vx, vy int32) (int32, int32) {
	ax, ay := abs32(vx), abs32(vy)
	mag := max(ax, ay) + min(ax, ay)>>2 + min(ax, ay)>>3
	switch {
	case mag > boidsVMax:
		return vx * boidsVMax / mag, vy * boidsVMax / mag
	case mag < boidsVMin && mag > 0:
		return vx * boidsVMin / mag, vy * boidsVMin / mag
	case mag == 0:
		return boidsVMin, 0
	}
	return vx, vy
}

func (b *boids) step() {
	n := boidsN[b.count]
	const r2 = (boidsR >> 4) * (boidsR >> 4)               // in (1/16 cell)²
	const sep2 = (2 * boidsFix >> 4) * (2 * boidsFix >> 4) // closer than 2 cells
	for i := range n {
		xi, yi := b.x[i], b.y[i]
		var cnt, sx, sy, svx, svy, px, py int32
		for j := range n {
			if j == i {
				continue
			}
			dx, dy := b.x[j]-xi, b.y[j]-yi
			if dx > boidsR || dx < -boidsR || dy > boidsR || dy < -boidsR {
				continue
			}
			// /16, not >>4: the shift rounds negative offsets away from zero,
			// so neighbours up and left counted as closer and separation
			// pushed the whole flock into the top-left corner
			d2 := (dx/16)*(dx/16) + (dy/16)*(dy/16)
			if d2 > r2 {
				continue
			}
			cnt++
			sx += dx
			sy += dy
			svx += b.vx[j]
			svy += b.vy[j]
			if d2 < sep2 {
				px -= dx
				py -= dy
			}
		}
		vx, vy := b.vx[i], b.vy[i]
		// steering divides rather than shifts: >> on a negative value rounds
		// towards -inf and drifted the whole flock up and left
		if cnt > 0 {
			vx += (svx/cnt - vx) / 8 // alignment
			vy += (svy/cnt - vy) / 8
			vx += sx / cnt / 64 // cohesion: towards the neighbours' centre
			vy += sy / cnt / 64
		}
		vx += px / 8 // separation
		vy += py / 8
		b.nvx[i], b.nvy[i] = limit(vx, vy)
	}
	// move in the world, then meet the box's walls (walls): a boid past one
	// turns inwards (the minimum speed would otherwise undo the push for a
	// boid flying straight at a wall), carried by the wall or gliding back.
	for i := range n {
		b.vx[i], b.vy[i] = b.nvx[i], b.nvy[i]
		b.x[i] += b.vx[i]
		b.y[i] += b.vy[i]
		b.walls(i)
	}
}

// boidsCarry is how far past a wall (per axis) the wall still carries a boid
// (puts it on the wall): a boid's own flight in a tick (0.5 cell) plus a
// shake or a turn of the box (about a cell a tick). Farther out, the window has outrun the
// boid (a hand stroke: up to 44 cells a tick, TG-6 #5a): it is left outside
// and glides back, 1/boidsReturn of the way to the wall each tick.
const (
	boidsCarry  = 3 * boidsFix / 2
	boidsReturn = 6
)

// walls applies the box's walls to boid i. A boid near a wall is pushed
// inwards. One past it turns inwards; the wall carries it if it is just
// past, otherwise it glides back. Never reflected across: a boid the window
// left 40 cells behind used to land 40 cells inside, a jump across the
// window (the owner's "they jump around"). Only boids a wall touches are
// converted back, so the others keep their world heading.
func (b *boids) walls(i int) {
	const push = 16
	const hiX, hiY = boidsW*boidsFix - 1, boidsH*boidsFix - 1
	p := &b.pose
	bx, by := p.toBox(b.x[i], b.y[i])
	near := bx < boidsMargin || bx > hiX-boidsMargin || by < boidsMargin || by > hiY-boidsMargin
	if !near {
		return
	}
	vx, vy := p.vecToBox(b.vx[i], b.vy[i])
	switch {
	case bx < boidsMargin:
		vx += push
	case bx > hiX-boidsMargin:
		vx -= push
	}
	switch {
	case by < boidsMargin:
		vy += push
	case by > hiY-boidsMargin:
		vy -= push
	}
	nx, vx := wall(bx, vx, hiX)
	ny, vy := wall(by, vy, hiY)
	if nx != bx || ny != by {
		b.x[i], b.y[i] = p.toWorld(nx, ny)
	}
	b.vx[i], b.vy[i] = limit(p.vecToWorld(vx, vy))
}

// wall meets one axis's walls (0 and hi, box units): a boid past one turns
// inwards; just past (boidsCarry) the wall carries it, farther it glides
// back a part of the way.
func wall(p, v, hi int32) (int32, int32) {
	var over int32 // how far past the wall, signed outwards
	switch {
	case p < 0:
		over, v = p, abs32(v)
	case p > hi:
		over, v = p-hi, -abs32(v)
	default:
		return p, v
	}
	if abs32(over) <= boidsCarry {
		return p - over, v
	}
	return p - over/boidsReturn, v
}

// octant of a heading: 0 east, counting clockwise on screen (y down).
func octant(vx, vy int32) uint8 {
	ax, ay := abs32(vx), abs32(vy)
	// steep / shallow / diagonal by the tan(22.5°) ≈ 0.414 ≈ 106/256 ratio
	var o uint8
	switch {
	case ay*256 < ax*106:
		o = 0 // horizontal
	case ax*256 < ay*106:
		o = 2 // vertical
	default:
		o = 1 // diagonal
	}
	// map to the 8 directions by signs
	switch {
	case vx >= 0 && vy >= 0:
		return o // 0 E, 1 SE, 2 S
	case vx < 0 && vy >= 0:
		return 4 - o // 4 W, 3 SW, 2 S
	case vx < 0 && vy < 0:
		return (4 + o) & 7 // 4 W, 5 NW, 6 N
	default:
		return (8 - o) & 7 // 0 E, 7 NE, 6 N
	}
}

// boidsFeel is the velocity a boid gains per tick per m/s² of apparent
// acceleration (1/256 cell per tick); boidsFeelMin (m/s²) is noise.
const (
	boidsFeel    = 4
	boidsFeelMin = 0.5
)

// turnTo turns the box (the window) to the badge's roll (radians,
// clockwise as the viewer sees it): from the motion state's attitude.
func (b *boids) turnTo(roll float32) {
	d := roll - b.pose.angle
	d -= 2 * math.Pi * float32(math.Round(float64(d/(2*math.Pi))))
	b.pose.setAngle(b.pose.angle + d)
}

// moveTo puts the box where the badge is: right and up (m), the motion
// state's displacement in the badge's own frame, at the sensitivity.
func (b *boids) moveTo(right, up float32) {
	b.pose.baseX = b.anchorBase[0] + int32((right-b.anchorDisp[0])*b.unitsPerM)
	b.pose.baseY = b.anchorBase[1] - int32((up-b.anchorDisp[1])*b.unitsPerM)
}

// setScale changes the window's travel per metre of badge motion (u, world
// units) from where it is now, the badge's displacement being right, up.
func (b *boids) setScale(u, right, up float32) {
	b.anchorBase = [2]int32{b.pose.baseX, b.pose.baseY}
	b.anchorDisp = [2]float32{right, up}
	b.unitsPerM = u
}

// feel sets the apparent acceleration (screen axes, m/s²) the flock feels
// from the badge's own acceleration: a shake stirs it.
func (b *boids) feel(ax, ay float32) { b.feltX, b.feltY = ax, ay }

// push gives every boid the velocity the felt acceleration adds this tick.
func (b *boids) push() {
	ax, ay := b.feltX, b.feltY
	if ax*ax+ay*ay < boidsFeelMin*boidsFeelMin {
		return
	}
	vx, vy := b.pose.vecToWorld(int32(ax*boidsFeel), int32(ay*boidsFeel))
	for i := range boidsN[b.count] {
		b.vx[i], b.vy[i] = limit(b.vx[i]+vx, b.vy[i]+vy)
	}
}

func (b *boids) tick() {
	t0 := time.Now()
	b.pose.advance()
	b.push()
	b.step()
	cx, cy := b.pose.cx, b.pose.cy
	b.histC[b.histI] = [2]int32{cx, cy}
	for i := range boidsN[b.count] {
		b.hist[i][b.histI] = histPoint(b.x[i]-cx, b.y[i]-cy)
	}
	b.histI = (b.histI + 1) % boidsHist
	b.stepUS = time.Since(t0).Microseconds()
}

// histPoint is a trail point relative to the box centre, saturated to
// int16: a boid the window left behind may be farther than int16 holds
// (128 cells), and a wrapped point would show inside the window.
func histPoint(dx, dy int32) [2]int16 {
	return [2]int16{int16(min(max(dx, math.MinInt16), math.MaxInt16)), int16(min(max(dy, math.MinInt16), math.MaxInt16))}
}

func (b *boids) fullView() ui.Widget {
	if b.fullOn {
		return b.full
	}
	return nil
}

// fullPress: UP/DOWN turn the box 22.5° (the walls sweep through the
// flock), A shakes it, B back to the page, C leaves and changes page.
func (b *boids) fullPress(btn Button) bool {
	switch btn {
	case BtnUp:
		b.pose.target += boxTurnStep
		b.log("box turns")
	case BtnDown:
		b.pose.target -= boxTurnStep
		b.log("box turns")
	case BtnA:
		b.pose.shake()
	case BtnB:
		b.fullOn = false
	case BtnC:
		b.fullOn = false
		return false
	}
	return true
}

// scatter sends every boid away from the box's centre at top speed.
func (b *boids) scatter() {
	for i := range boidsN[b.count] {
		b.vx[i], b.vy[i] = limit(b.x[i]-b.pose.cx, b.y[i]-b.pose.cy)
	}
}

func (a *App) pageBoids() *page {
	b := a.boids
	count := widget.NewInteractiveLabelChoice(52, 11, boidsCounts,
		widget.WithLabelChoiceIndex(&b.count), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	spin := widget.NewToggle(44, 12, fontBody, colFG, "ON", "OFF", colGood, colTrack,
		func() bool { return b.pose.spin }, func(v bool) { b.pose.spin = v })
	full := widget.NewToggle(44, 12, fontBody, colFG, "SHOW", "SHOW", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.fullOn = true; a.log("boids full screen") })
	gather := widget.NewToggle(44, 12, fontBody, colFG, "GO", "GO", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.gather(); a.log("boids gather") })
	scatter := widget.NewToggle(44, 12, fontBody, colFG, "GO", "GO", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.scatter(); a.log("boids scatter") })
	// explicit height: findings F-33
	p := a.newPageWith("Boids", layout.VFlow(4, boidsH*boidsPitch),
		still(b.grid),
		newField("Boids", 42, 104, count),
		newField("Gather", 42, 104, gather),
		newField("Scatter", 42, 104, scatter),
		newField("Spin", 42, 104, spin),
		newField("Full", 42, 104, full),
		still(a.text(104, func() string { return b.txt.reset().str("step ").int(int(b.stepUS)).str(" us").String() }, colMuted)),
	)
	p.demo = b
	return p
}

func (b *boids) tileViews() (ui.Widget, ui.Widget) { return b.tiles[0], b.tiles[1] }
func (b *boids) showFull()                         { b.fullOn = true }

// reset straightens the box at the world's centre and gathers the flock
// into it.
func (b *boids) reset() {
	b.pose = newBoxPose()
	b.gather()
}

// gather puts every boid at a random place and heading inside the window
// where it is now, moved and turned (owner, TG-6 #5: a control that resets
// the flock into the window), with no trail.
func (b *boids) gather() {
	for i := range boidsMax {
		b.x[i], b.y[i] = b.pose.toWorld(int32(rand32()%(boidsW*boidsFix)), int32(rand32()%(boidsH*boidsFix)))
		b.vx[i] = int32(rand32()%(2*boidsVMax)) - boidsVMax
		b.vy[i] = int32(rand32()%(2*boidsVMax)) - boidsVMax
	}
	for k := range boidsHist { // every history slot is now
		b.histC[k] = [2]int32{b.pose.cx, b.pose.cy}
		for i := range boidsMax {
			b.hist[i][k] = histPoint(b.x[i]-b.pose.cx, b.y[i]-b.pose.cy)
		}
	}
}
