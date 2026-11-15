package app

import (
	"math"
	"time"
)

// Serial console. The board feeds ui.CommandStreamMux with these handlers;
// handlers only enqueue, and Update executes them on the UI goroutine, so the
// widget tree is touched by one goroutine only.

type command struct {
	name   string
	text   string // the argument as sent (page takes a name)
	arg    int
	hasArg bool
	long   bool
}

// helpLines lists the commands in lines that fit a log line (logLineLen):
// one long line was cut off after `led`.
var helpLines = [...]string{
	"commands: up|down|a|b|c [long], user <ms>, page <n|name>",
	"led <0-100>, bl <10-100>, saver [0|1], solid [0-255]",
	"stats, tiles, frames, mem, memshow [0|1], light [raw|-1], tilt, motion",
	"sens [2.5|5|10|20|40]: motion sensitivity, 1 px = N mm; dump, version, help",
	"cal: the calibration fit and its orientations' poses",
}

// Handlers returns the command table for ui.NewCommandStreamMux.
func (a *App) Handlers() map[string]func([]byte) {
	m := map[string]func([]byte){}
	for _, name := range []string{"up", "down", "a", "b", "c", "user", "page", "led", "bl", "saver", "solid", "stats", "tiles", "frames", "mem", "memshow", "light", "tilt", "motion", "sens", "dump", "version", "help", "cal"} {
		m[name] = func(arg []byte) { a.Command(name, string(arg)) }
	}
	return m
}

// Command queues one console command (safe to call from another goroutine).
func (a *App) Command(name, arg string) {
	c := command{name: name, text: arg, long: arg == "long"}
	c.arg, c.hasArg = atoi(arg)
	if name == "sens" && arg != "" { // a choice, 2.5 isn't an int: the index, -1 for none
		c.arg, c.hasArg = sensArg(arg), true
	}
	select {
	case a.cmds <- c:
	default: // queue full: drop
	}
}

func (a *App) exec(c command) {
	switch c.name {
	case "up":
		a.Press(BtnUp, c.long)
	case "down":
		a.Press(BtnDown, c.long)
	case "a":
		a.Press(BtnA, c.long)
	case "b":
		a.Press(BtnB, c.long)
	case "c":
		a.Press(BtnC, c.long)
	case "user":
		a.UserHold(time.Duration(c.arg) * time.Millisecond)
	case "page": // an index or a page's name; anything else stays and says so
		if i := a.pageArg(c); i >= 0 {
			a.showPage(i)
		} else {
			a.log(a.txtStat.reset().str("page: a number 0-").int(a.pages.Count() - 1).str(" or a page name").String())
		}
	case "led":
		a.ledOn, a.ledPct = c.arg > 0, clampi(c.arg, 0, 100)
		a.applyLED()
	case "bl":
		a.backlight = clampi(c.arg, 10, 100)
		a.hw.SetBacklight(a.backlight)
	case "saver": // no argument toggles; 1 on, 0 off
		if c.hasArg {
			a.saver = c.arg != 0
		} else {
			a.saver = !a.saver
		}
	case "solid": // a uniform grey screen for camera tests; no argument: back to the UI
		a.solid = -1
		if c.hasArg {
			a.solid = clampi(c.arg, 0, 255)
		}
	case "dump":
		a.dumpReq = true
	case "stats":
		a.log2("build ", a.cfg.BuildID)
		a.log(a.txtStat.reset().str("fps ").int(a.fps).str(" draw_us ").int(int(a.drawUS)).str(" push_us ").int(int(a.pushUS)).
			str(" page ").int(a.cur).str(" edit ").int(btoi(a.editing)).String())
		a.memReport() // live and free after a collection: heap_alloc counts uncollected garbage
		a.log(a.txtStat.reset().str("frame allocs update ").int(a.allocU).str(" draw ").int(a.allocD).str(" push ").int(a.allocP).String())
		a.profileReport()
	case "tiles": // frame budget of each demo tile on the Demos page
		a.menu.report(a.log, &a.txtStat)
	case "light": // the phototransistor's last reading; light <raw> forces a reading (0: darkest), light -1 the real sensor again
		if c.hasArg {
			a.lightForce = clampi(c.arg, -1, 65535)
			a.lightFilt = lightFilter{} // the forced (or real) reading applies at the next sample
		}
		a.log(a.txtStat.reset().str("light raw ").int(int(a.lightRaw)).str(" median ").int(int(a.lightFilt.med)).
			str(" pct ").int(a.lightShown).str(" backlight ").int(a.backlight).String())
	case "tilt": // the in-plane gravity the demos use, as an angle from screen-down
		a.logTilt()
	case "frames": // frame-time histogram and GCs since the last call
		a.framesReport()
	case "mem": // live heap after a collection
		a.memReport()
	case "memshow": // heap use in the footer
		a.health.memShow = !a.health.memShow
		if c.hasArg {
			a.health.memShow = c.arg != 0
		}
	case "cal": // the calibration fit and its poses, as numbers
		a.calReport()
	case "version": // the firmware's build ID (tools/build)
		a.log2("build ", a.cfg.BuildID)
	case "motion": // the shared motion state as the demos read it (measurable at rest)
		a.motionReport()
	case "sens": // the motion sensitivity: mm of badge motion per px of the paddle and the boids' window
		switch {
		case !c.hasArg:
		case c.arg < 0:
			a.log("sens: one of 2.5 5 10 20 40 (1 px = that many mm)")
			return
		default:
			a.setSens(c.arg)
		}
		a.log2("sens: 1 px = ", sensLabels[a.sens])
	case "help":
		for _, l := range helpLines {
			a.log(l)
		}
	}
}

// pageArg is the page a page command names: its index, or a page's name
// in any case; -1 for neither.
func (a *App) pageArg(c command) int {
	if c.hasArg {
		if c.arg >= 0 && c.arg < a.pages.Count() {
			return c.arg
		}
		return -1
	}
	for i := range a.pages.Count() {
		if c.text != "" && asciiFold(a.pages.Name(i), c.text) {
			return i
		}
	}
	return -1
}

// asciiFold reports whether a and b match ignoring ASCII case (page names
// are ASCII; strings.EqualFold brings Unicode tables: 8 KB of flash).
func asciiFold(a, b string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := 0; i < len(a); i++ {
		x, y := a[i], b[i]
		if x >= 'A' && x <= 'Z' {
			x += 'a' - 'A'
		}
		if y >= 'A' && y <= 'Z' {
			y += 'a' - 'A'
		}
		if x != y {
			return false
		}
	}
	return true
}

func atoi(s string) (int, bool) {
	n, neg, ok := 0, false, false
	for i := 0; i < len(s); i++ {
		c := s[i]
		switch {
		case c == '-' && i == 0:
			neg = true
		case c >= '0' && c <= '9':
			n = n*10 + int(c-'0')
			ok = true
		default:
			return 0, false
		}
	}
	if neg {
		n = -n
	}
	return n, ok
}

func clampi(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// logTilt logs the in-plane gravity the demos use: the angle from screen-down
// (tenths of a degree), the side it leans to, and its size, e.g. "gravity 4.3
// deg toward screen left, in-plane 9.8 m/s2" — which side of the badge is
// lower; a small in-plane size means the badge lies nearly flat and the
// angle means little.
func (a *App) logTilt() {
	if !a.tilted {
		a.log("gravity: no direction (flat, or no motion sensor)")
		return
	}
	deg := math.Atan2(float64(a.tiltX), float64(a.tiltY)) * 180 / math.Pi
	side := " deg toward screen right"
	if deg < 0 {
		deg, side = -deg, " deg toward screen left"
	}
	tenths := int(math.Round(deg * 10))
	mag := int(math.Round(math.Hypot(float64(a.tiltX), float64(a.tiltY)) * 10))
	a.log3(a.txtStat.reset().str("gravity ").int(tenths/10).str(".").int(tenths%10).String(), side,
		a.txtLog.reset().str(", in-plane ").int(mag/10).str(".").int(mag%10).str(" m/s2").String())
}

// motionReport logs the shared motion state the demos read: the world
// position and the badge-frame displacement in µm (the Motion page shows
// whole cm; the paddle moves 0.13 mm a pixel), how many samples the strapdown called rest since the last
// report, and what moves on screen: Pong's paddle row (px) and the boids'
// window offset (tenths of a cell).
func (a *App) motionReport() {
	m := a.motion
	if m == nil {
		a.log("motion: no IMU")
		return
	}
	um := func(v float32) int { return int(v * 1e6) }
	a.log(a.txtStat.reset().str("motion P ").int(um(m.nav.P[0])).str(" ").int(um(m.nav.P[1])).str(" ").int(um(m.nav.P[2])).
		str(" um disp ").int(um(m.disp[0])).str(" ").int(um(m.disp[1])).str(" ").int(um(m.disp[2])).str(" um").String())
	rest := 0
	if m.sampleN > 0 {
		rest = m.restN * 100 / m.sampleN
	}
	b := a.boids
	a.log(a.txtStat.reset().str("rest ").int(rest).str("% of ").int(m.sampleN).str(" settled ").int(btoi(m.settled)).
		str(" pad ").int(int(a.pong.pad[0] / (pongFix / 5))).
		str(" win ").int(int((b.pose.baseX - boidsW*boidsFix/2) * 10 / boidsFix)).str(" ").int(int((b.pose.baseY - boidsH*boidsFix/2) * 10 / boidsFix)).
		str(" moves ").int(m.movesN).String())
	m.sampleN, m.restN, m.movesN = 0, 0, 0
}
