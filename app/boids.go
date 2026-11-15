package app

import (
	"image/color"
	"math"
	"time"

	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/layout"
	"github.com/itohio/tinygui/widget"
)

// Boids fly on the inside of a sphere around the badge, and the badge is the
// camera at its centre: turning the badge turns the view across the sphere.
// The view is the badge's attitude only (no position). Each boid is a unit
// vector on the sphere with a velocity tangent to it (rad/s); separation,
// alignment and cohesion work by chord distance between neighbours, the
// velocity is kept tangent and the position renormalised every step. A boid
// outside the view is pulled back toward the view's direction (it wants to
// be seen), a boid inside only faintly. The screen shows the sphere around
// the view direction by gnomonic projection over boidsFOV across. Fixed
// arrays: no allocation after setup.
const (
	boidsMax   = 128
	boidsPitch = 3       // the page's view: 64×48 cells' worth of pixels at this pitch
	boidsGridW = 64      // the page's view, in cells (boidsPitch px each)
	boidsGridH = 48      //
	boidsDT    = 1. / 30 // s per step (one per frame)
	boidsVMin  = 0.12    // rad/s
	boidsVMax  = 0.40    // rad/s
	boidsNear  = 0.14    // rad: neighbours within this see each other (8°)
	boidsSep   = 0.05    // rad: closer than this they push apart (3°)
	boidsKSep  = 1.5     // 1/s: separation weight
	boidsKAli  = 0.6     // 1/s: alignment
	boidsKCoh  = 0.4     // 1/s: cohesion
	boidsKOut  = 0.8     // 1/s: outside the view, the pull toward its direction (seek)
	boidsBand  = 0.3     // inside the view's cone, the border's soft band: the outer 30 % of its angle
	boidsTrail = 3       // trail palette indices 1..3, heads from 4
	boidsHist  = 4       // positions kept per boid for the trail
	boidsHistQ = 127     // the trail's unit vectors as int8 (0.45°: about 2 px at the 60° view)
	boidsQ     = 1 << 14 // positions and velocities in Q14: the step and the projection are integer (no FPU on the M0+)
	boidsScrQ  = 1 << 12 // projected points in Q12 of the view's half width
	boidsOff   = -32768  // a projected point off the view
)

// the field of view across the view: 40–150° (owner), 110° at first
const boidsFOVDefault = 7

var (
	boidsCounts = []string{"32", "64", "128"}
	boidsN      = [3]int{32, 64, 128}
	// the view's field across the screen (the projection's width): a choice
	boidsFOVs      = [...]float32{40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150}
	boidsFOVLabels = []string{"40°", "50°", "60°", "70°", "80°", "90°", "100°", "110°", "120°", "130°", "140°", "150°"}
)

var boidsPalette = []color.RGBA{
	{14, 14, 22, 255},
	{22, 34, 46, 255}, {32, 60, 78, 255}, {50, 96, 120, 255}, // trail, fading
	// heads, one hue per on-screen heading octant
	{255, 90, 90, 255}, {255, 170, 60, 255}, {240, 240, 80, 255}, {120, 240, 90, 255},
	{70, 230, 200, 255}, {80, 160, 255, 255}, {160, 110, 255, 255}, {250, 100, 220, 255},
}

type boids struct {
	p, v   [boidsMax][3]int16 // unit position on the sphere (world: z up) and tangent velocity (rad/s), Q14 (boidsQ)
	hist   [boidsMax][boidsHist][3]int8
	histI  int
	count  int       // index into boidsN
	fov    int       // index into boidsFOVs
	fovAt  time.Time // when Up/Down in the full view last changed it: the readout fades from then
	stepUS int64
	// this frame's camera (camera): the view direction, the screen's right
	// and up, all world unit vectors, and the projection's scale
	look, right, up  [3]int32 // the camera's axes in the world, Q14
	tanHalf          int32    // tan of half the field across, Q14
	cosView, cosBand int32    // the view's cone and where its border band starts: cos of their angles from the view's direction (Q14)
	bandPeak         int32    // the band's push at the cone's edge (Q14 rad/s²)
	free             int      // ticks left in which Scatter's flock flies out: no border band, no seek
	// the frame's projection, once per tick (Draw runs once per band):
	// heads and trail points in Q12 of the view's half width (boidsOff: not
	// in front), the heading's octant
	head       [boidsMax][2]int16
	trail      [boidsMax][boidsHist - 1][2]int16
	oct        [boidsMax]uint8
	grid, full *boidsView
	tiles      [2]*boidsView
	fullOn     bool
	txt        textBuf
	log        func(string)
	motion     *motion // nil without an IMU: the camera stays put
}

func newBoids() *boids {
	b := &boids{count: 1, fov: boidsFOVDefault}
	b.camera()
	b.reset()
	b.grid = newBoidsView(b, boidsGridW*boidsPitch, boidsGridH*boidsPitch)
	b.full = newBoidsView(b, ScreenW, ScreenH)
	b.tiles = [2]*boidsView{newBoidsView(b, tileW, tileH), newBoidsView(b, 2*tileW, 2*tileH)}
	return b
}

func dot3(a, b [3]float32) float32 { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }

func unit3(a [3]float32) [3]float32 {
	n := float32(math.Sqrt(float64(dot3(a, a))))
	if n == 0 {
		return [3]float32{0, 0, 1}
	}
	return [3]float32{a[0] / n, a[1] / n, a[2] / n}
}

// tangent is a with its component along the unit p removed.
func tangent(a, p [3]float32) [3]float32 {
	d := dot3(a, p)
	return [3]float32{a[0] - d*p[0], a[1] - d*p[1], a[2] - d*p[2]}
}

// pos and vel are boid i's position and velocity as floats (the tests);
// set stores them (Q14).
func (b *boids) pos(i int) [3]float32 { return fromQ14(b.p[i]) }
func (b *boids) vel(i int) [3]float32 { return fromQ14(b.v[i]) }
func (b *boids) set(i int, p, v [3]float32) {
	b.p[i], b.v[i] = q14of(toQ14(p)), q14of(toQ14(v))
}

func floatQ(q [3]int32) [3]float32 {
	return [3]float32{float32(q[0]) / boidsQ, float32(q[1]) / boidsQ, float32(q[2]) / boidsQ}
}

func fromQ14(q [3]int16) [3]float32 {
	return [3]float32{float32(q[0]) / boidsQ, float32(q[1]) / boidsQ, float32(q[2]) / boidsQ}
}

// toQ14 rounds a float vector to Q14 (setup and the camera, once a frame).
func toQ14(v [3]float32) (q [3]int32) {
	for a := range 3 {
		x := v[a] * boidsQ
		if x < 0 {
			x -= 0.5
		} else {
			x += 0.5
		}
		q[a] = int32(x)
	}
	return q
}

func q14of(v [3]int32) (q [3]int16) {
	for a := range 3 {
		q[a] = int16(min(max(v[a], -boidsQ), boidsQ-1))
	}
	return q
}

func wide(q [3]int16) [3]int32 { return [3]int32{int32(q[0]), int32(q[1]), int32(q[2])} }

// the Q14 vector maths: |components| ≤ 2^15, so a dot product (Q28) fits
// int32. Every shift rounds to the nearest (rsh): a plain >> floors, always
// toward −∞, and that bias bent every boid's path the same way (the flock
// went round in circles).
func rsh(x int32, k uint) int32 { return (x + 1<<(k-1)) >> k }

// divr is x/d rounded to the nearest (d > 0): Go's / truncates toward
// zero, which shrinks each component on its own and turns a vector toward
// the axes.
func divr(x, d int32) int32 {
	if x < 0 {
		return -((-x + d/2) / d)
	}
	return (x + d/2) / d
}

func dotQ(a, b [3]int32) int32 { return rsh(a[0]*b[0]+a[1]*b[1]+a[2]*b[2], 14) }

func tangentQ(a, p [3]int32) [3]int32 {
	d := dotQ(a, p)
	return [3]int32{a[0] - rsh(d*p[0], 14), a[1] - rsh(d*p[1], 14), a[2] - rsh(d*p[2], 14)}
}

func crossQ(a, b [3]int32) [3]int32 {
	return [3]int32{rsh(a[1]*b[2]-a[2]*b[1], 14), rsh(a[2]*b[0]-a[0]*b[2], 14), rsh(a[0]*b[1]-a[1]*b[0], 14)}
}

// lenQ is |a| in Q14 (an integer square root of the Q28 dot product).
func lenQ(a [3]int32) int32 { return int32(isqrt(uint32(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]))) }

// scaleTo gives a the length l (Q14); a zero vector stays zero.
func scaleTo(a [3]int32, l int32) [3]int32 {
	n := lenQ(a)
	if n == 0 {
		return a
	}
	return [3]int32{divr(a[0]*l, n), divr(a[1]*l, n), divr(a[2]*l, n)}
}

func isqrt(x uint32) uint32 {
	var r uint32
	for bit := uint32(1 << 30); bit != 0; bit >>= 2 {
		if x >= r+bit {
			x -= r + bit
			r = r>>1 + bit
		} else {
			r >>= 1
		}
	}
	return r
}

// camera takes this frame's view from the badge's attitude: it looks along
// the badge's −Z (into the screen), the screen's right is the badge's +X and
// its up +Y. Without an IMU, or until the attitude has settled, it looks
// along the world's −Y.
func (b *boids) camera() {
	th := math.Tan(float64(boidsFOVs[b.fov]) * math.Pi / 360)
	b.tanHalf = int32(th*boidsQ + 0.5)
	r := math.Atan(th * boidsGridH / boidsGridW) // the view's cone: the field across the screen's height
	b.cosView = int32(math.Cos(r)*boidsQ + 0.5)
	b.cosBand = int32(math.Cos((1-boidsBand)*r)*boidsQ + 0.5)
	b.bandPeak = bandPush(boidsBand * r)
	if m := b.motion; m != nil && m.att.settled {
		q := m.att.q()
		b.look = toQ14(toWorld(q, [3]float32{0, 0, -1}))
		b.right = toQ14(toWorld(q, [3]float32{1, 0, 0}))
		b.up = toQ14(toWorld(q, [3]float32{0, 1, 0}))
		return
	}
	b.look, b.right, b.up = [3]int32{0, -boidsQ, 0}, [3]int32{boidsQ, 0, 0}, [3]int32{0, 0, boidsQ}
}

// project is p's place on the view (gnomonic: straight lines stay
// straight) in Q12 of the half width, x right, y up; ok false when p is not
// in front.
func (b *boids) project(p [3]int32) (x, y int32, ok bool) {
	z := dotQ(p, b.look)
	if z <= boidsQ/20 {
		return 0, 0, false
	}
	zt := z * b.tanHalf >> 14
	return (dotQ(p, b.right) << 12) / zt, (dotQ(p, b.up) << 12) / zt, true
}

// inCone reports whether p is in the view's cone: within its angle of the
// view's direction (the dot product: no screen coordinates).
func (b *boids) inCone(p [3]int32) bool { return dotQ(p, b.look) > b.cosView }

// pairGap is 1 − cos of the great-circle angle between two boids, Q28:
// every distance between boids is this angle (neighbours, separation). It
// is computed as |p − q|²/2, the same number for unit vectors: 1 − p·q
// itself cancels to a few bits near 1 (±0.1° at the separation's 3°).
func pairGap(p, q [3]int32) int32 {
	d0, d1, d2 := p[0]-q[0], p[1]-q[1], p[2]-q[2]
	return (d0*d0 + d1*d1 + d2*d2) >> 1
}

// the step's constants in Q14 (lengths, speeds) and Q12 (gains, 1/s),
// rounded once
const gainQ = 1 << 12

var (
	kSepQ  = roundQ(boidsKSep * gainQ)
	kAliQ  = roundQ(boidsKAli * gainQ)
	kCohQ  = roundQ(boidsKCoh * gainQ)
	kOutQ  = roundQ(boidsKOut * gainQ)
	vMaxQ  = roundQ(boidsVMax * boidsQ)
	vMinQ  = roundQ(boidsVMin * boidsQ)
	dtQ16  = roundQ(boidsDT * (1 << 16))
	vMax2Q = vMaxQ * vMaxQ // Q28
	vMin2Q = vMinQ * vMinQ
)

// roundQ rounds a positive constant to the nearest integer (math.Round's
// NaN test doesn't run in TinyGo's init-time interpreter).
func roundQ(x float64) int32 { return int32(x + 0.5) }

// step moves the flock one boidsDT on the sphere, all in integers (Q14
// unit vectors; no FPU on the M0+): the neighbours within boidsNear (chord²,
// a Q28 dot product) align and cohere, those within boidsSep push apart;
// every boid is pulled toward the view's direction, turning when it heads
// away; the speed stays within boidsVMin..boidsVMax; one Newton step keeps
// the position on the sphere (a step moves it at most 0.013 rad).
func (b *boids) step() {
	n := boidsN[b.count]
	nearGap := int32((1 - math.Cos(boidsNear)) * boidsQ * boidsQ) // 1 − cos(boidsNear), Q28
	sepGap := int32((1 - math.Cos(boidsSep)) * boidsQ * boidsQ)   // 1 − cos(boidsSep), Q28
	for i := range n {
		pi, vi := wide(b.p[i]), wide(b.v[i])
		var sepF [3]float32
		var ali, coh [3]int32
		k := int32(0)
		for j := range n {
			if j == i {
				continue
			}
			pj := wide(b.p[j])
			gap := pairGap(pi, pj) // 1 − cos of their angle on the sphere
			if gap >= nearGap {
				continue
			}
			k++
			ali[0] += int32(b.v[j][0])
			ali[1] += int32(b.v[j][1])
			ali[2] += int32(b.v[j][2])
			coh[0] += int32(b.p[j][0])
			coh[1] += int32(b.p[j][1])
			coh[2] += int32(b.p[j][2])
			if gap < sepGap && gap > 0 { // close: rare; the push grows as 1/(1 − cos), float keeps its range
				f := float32(sepGap) / float32(gap) // away along the great circle: the chord, laid on the tangent plane below
				sepF[0] += float32(pi[0]-pj[0]) * f
				sepF[1] += float32(pi[1]-pj[1]) * f
				sepF[2] += float32(pi[2]-pj[2]) * f
			}
		}
		var s [3]int32 // the steering, Q14 per s
		if k > 0 {
			sep := q14clamp(sepF)
			for a := range 3 {
				s[a] = rsh(kSepQ*sep[a]+kAliQ*(divr(ali[a], k)-vi[a])+kCohQ*(divr(coh[a], k)-pi[a]), 12)
			}
		}
		// inside the view: kept off its border by a soft push back inward that
		// grows across the outer band (nothing in the middle: nothing pulls
		// a boid along the border). Outside: it seeks the view's direction,
		// and heading away it turns (a pull straight back would only slow
		// it to boidsVMin, and the speed limit keeps a velocity from passing
		// through zero): sideways, toward the side the view is on
		toView := tangentQ([3]int32{b.look[0] - pi[0], b.look[1] - pi[1], b.look[2] - pi[2]}, pi) // along the great circle to the view's direction
		if c := dotQ(pi, b.look); c > b.cosView {
			if c < b.cosBand && b.free == 0 { // in the band
				e := min((b.cosBand-c)*gainQ/(b.cosBand-b.cosView), gainQ) // depth into the band, Q12
				f := rsh(b.bandPeak*rsh(e*e, 12), 12)                      // Q14 rad/s²
				d := scaleTo(toView, boidsQ)
				for a := range 3 {
					s[a] += rsh(f*d[a], 14)
				}
			}
		} else if b.free == 0 { // outside: seek (not while Scatter sends them out)
			pull := toView
			for a := range 3 {
				s[a] += rsh(kOutQ*pull[a], 12)
			}
			if dotQ(vi, pull) < 0 {
				side := crossQ(pi, scaleTo(vi, boidsQ)) // tangent, across the heading
				if dotQ(side, pull) < 0 {
					side = [3]int32{-side[0], -side[1], -side[2]}
				}
				kl := rsh(kOutQ*lenQ(pull), 14) // Q12
				for a := range 3 {
					s[a] += rsh(kl*side[a], 12)
				}
			}
		}
		// the velocity is steered in place: later boids see it already
		// steered, which flocking doesn't mind; the positions all move after
		acc := tangentQ(s, pi)
		for a := range 3 {
			vi[a] += rsh(acc[a]*dtQ16, 16)
		}
		b.v[i] = q14of(tangentQ(vi, pi))
	}
	for i := range n {
		p, v := wide(b.p[i]), wide(b.v[i])
		switch sp2 := v[0]*v[0] + v[1]*v[1] + v[2]*v[2]; {
		case sp2 > vMax2Q:
			v = scaleTo(v, vMaxQ)
		case sp2 == 0:
			v = scaleTo(tangentQ(b.right, p), vMinQ)
		case sp2 < vMin2Q:
			v = scaleTo(v, vMinQ)
		}
		var q [3]int32
		for a := range 3 {
			q[a] = p[a] + rsh(v[a]*dtQ16, 16)
		}
		// back onto the sphere: q·(3 − |q|²)/2, the factor in Q16
		f := rsh(3<<28-(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]), 13)
		for a := range 3 {
			q[a] = rsh(q[a]*f, 16)
		}
		b.p[i], b.v[i] = q14of(q), q14of(tangentQ(v, q))
	}
}

// bandSafety scales the border band's push over what just stops a boid at
// full speed heading straight out within the band (a variable: the tests
// switch it off to see a boid fly free).
var bandSafety = 1.5

// bandPush is the band's peak push (Q14 rad/s²) for a band w rad wide: a
// push growing as the square of the depth into the band averages a third of
// its peak, and stopping boidsVMax within w takes vMax²/2w on average.
func bandPush(w float64) int32 {
	return int32(bandSafety*3*boidsVMax*boidsVMax/(2*w)*boidsQ + 0.5)
}

// q14clamp turns the separation's float sum into Q14 (a sum of chords
// times sep²/chord²: kept to ±2).
func q14clamp(v [3]float32) (q [3]int32) {
	for a := range 3 {
		q[a] = int32(min(max(v[a], -2*boidsQ), 2*boidsQ))
	}
	return q
}

// frame projects the flock for Draw, once per tick: the heads, the trail
// points and the heading's octant.
func (b *boids) frame() {
	for i := range boidsN[b.count] {
		p, v := wide(b.p[i]), wide(b.v[i])
		b.head[i] = b.scr(p)
		// the heading on screen: where the boid will be shortly
		ahead := [3]int32{p[0] + v[0]/5, p[1] + v[1]/5, p[2] + v[2]/5}
		if h := b.scr(ahead); b.head[i][0] != boidsOff && h[0] != boidsOff {
			b.oct[i] = octant(float32(h[0]-b.head[i][0]), float32(b.head[i][1]-h[1]))
		}
		for k := 1; k < boidsHist; k++ {
			slot := (b.histI - 1 - k + 2*boidsHist) % boidsHist
			hq := b.hist[i][slot]
			b.trail[i][k-1] = b.scr([3]int32{int32(hq[0]) * boidsQ / boidsHistQ, int32(hq[1]) * boidsQ / boidsHistQ, int32(hq[2]) * boidsQ / boidsHistQ})
		}
	}
}

// scr is p projected, Q12 of the half width (y up), or boidsOff.
func (b *boids) scr(p [3]int32) [2]int16 {
	x, y, ok := b.project(p)
	if !ok || x <= -2*boidsScrQ || x >= 2*boidsScrQ || y <= -2*boidsScrQ || y >= 2*boidsScrQ {
		return [2]int16{boidsOff, boidsOff}
	}
	return [2]int16{int16(x), int16(y)}
}

func (b *boids) tick() {
	t0 := time.Now()
	b.camera()
	b.step()
	for i := range boidsN[b.count] {
		for a := range 3 {
			b.hist[i][b.histI][a] = int8(int32(b.p[i][a]) * boidsHistQ / boidsQ)
		}
	}
	b.histI = (b.histI + 1) % boidsHist
	b.free = max(b.free-1, 0)
	b.frame()
	b.stepUS = time.Since(t0).Microseconds()
}

func (b *boids) fullView() ui.Widget {
	if b.fullOn {
		return b.full
	}
	return nil
}

// fullPress: UP/DOWN widen and narrow the field of view (its readout
// fades: fovReadout), A gathers the flock into the view, B back to the page, C
// leaves and changes page.
func (b *boids) fullPress(btn Button) bool {
	switch btn {
	case BtnUp, BtnDown: // the field of view, shown briefly
		if btn == BtnUp {
			b.fov = min(b.fov+1, len(boidsFOVs)-1)
		} else {
			b.fov = max(b.fov-1, 0)
		}
		b.fovAt = time.Now()
	case BtnA:
		b.gather()
	case BtnB:
		b.fullOn = false
	case BtnC:
		b.fullOn = false
		return false
	}
	return true
}

// randIn is a random number in [−1, 1).
func randIn() float32 { return float32(rand32()%2048)/1024 - 1 }

// scatter sends every boid away from the view's direction at top speed.
func (b *boids) scatter() {
	// the border band and the seek stay off until a boid at full speed has
	// got past the screen's corner, and half a second more (the cone spans
	// only the screen's height: past its rim a boid is still on the
	// screen's sides): the flock flies off the screen,
	// then the seek brings it back (a fixed 1.5 s, and seeking meanwhile,
	// kept most of a wide view's flock in: 110° is a 47° cone, 150° a 70°
	// one)
	t := float64(b.tanHalf) / boidsQ
	r := math.Atan(t * math.Hypot(1, boidsGridH/float64(boidsGridW))) // to the screen's corner
	b.free = int((r/boidsVMax + 0.5) * 30)
	for i := range boidsN[b.count] {
		p, look := b.pos(i), floatQ(b.look)
		d := unit3(tangent([3]float32{p[0] - look[0], p[1] - look[1], p[2] - look[2]}, p))
		b.set(i, p, [3]float32{d[0] * boidsVMax, d[1] * boidsVMax, d[2] * boidsVMax})
	}
}

func (a *App) pageBoids() *page {
	b := a.boids
	count := widget.NewInteractiveLabelChoice(52, 11, boidsCounts,
		widget.WithLabelChoiceIndex(&b.count), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	fov := widget.NewInteractiveLabelChoice(52, 11, boidsFOVLabels,
		widget.WithLabelChoiceIndex(&b.fov), widget.WithLabelChoiceFont(fontBody), widget.WithLabelChoiceColor(colFG))
	full := widget.NewToggle(44, 12, fontBody, colFG, "SHOW", "SHOW", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.fullOn = true; a.log("boids full screen") })
	gather := widget.NewToggle(44, 12, fontBody, colFG, "GO", "GO", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.gather(); a.log("boids gather") })
	scatter := widget.NewToggle(44, 12, fontBody, colFG, "GO", "GO", colTrack, colTrack,
		func() bool { return false }, func(bool) { b.scatter(); a.log("boids scatter") })
	// explicit height: findings F-33
	p := a.newPageWith("Boids", layout.VFlow(4, boidsGridH*boidsPitch),
		still(b.grid),
		newField("Boids", 42, 104, count),
		newField("View", 42, 104, fov),
		newField("Gather", 42, 104, gather),
		newField("Scatter", 42, 104, scatter),
		newField("Full", 42, 104, full),
		still(a.text(104, func() string { return b.txt.reset().str("step ").int(int(b.stepUS)).str(" us").String() }, colMuted)),
	)
	p.demo = b
	return p
}

func (b *boids) tileViews() (ui.Widget, ui.Widget) { return b.tiles[0], b.tiles[1] }
func (b *boids) showFull()                         { b.fullOn = true }

// reset gathers the flock into the view.
func (b *boids) reset() { b.gather() }

// gather puts every boid at a random place inside the view and a random
// heading along the sphere, with no trail.
func (b *boids) gather() {
	b.camera()
	look, right, up := floatQ(b.look), floatQ(b.right), floatQ(b.up)
	cv := float64(b.cosView) / boidsQ
	for i := range boidsMax {
		// uniform over the cone's cap: cos of the angle uniform, the bearing too
		c := 1 - (float64(randIn())+1)/2*(1-cv)
		sn := math.Sqrt(1 - c*c)
		az := float64(randIn()) * math.Pi
		x, y := float32(sn*math.Cos(az)), float32(sn*math.Sin(az))
		p := unit3([3]float32{float32(c)*look[0] + x*right[0] + y*up[0], float32(c)*look[1] + x*right[1] + y*up[1], float32(c)*look[2] + x*right[2] + y*up[2]})
		d := unit3(tangent([3]float32{randIn(), randIn(), randIn()}, p))
		sp := boidsVMin + (boidsVMax-boidsVMin)*(randIn()+1)/2
		b.set(i, p, [3]float32{d[0] * sp, d[1] * sp, d[2] * sp})
		for k := range boidsHist {
			for a := range 3 {
				b.hist[i][k][a] = int8(int32(b.p[i][a]) * boidsHistQ / boidsQ)
			}
		}
	}
}
