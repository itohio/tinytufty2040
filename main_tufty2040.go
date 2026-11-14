//go:build tufty2040

// Command tufty2040 is the TinyGUI showcase firmware for the Pimoroni Tufty 2040.
//
//	tinygo flash -target=tufty2040 .
//
// Board code lives here; the UI is in ./app and runs unchanged in ./sim.
package main

import (
	"device/arm"
	"io"
	"machine"
	"runtime"
	"time"

	ui "github.com/itohio/tinygui"
	"github.com/itohio/tinygui/examples/framebuffer"
	"github.com/itohio/tinytufty2040/app"
	"github.com/itohio/tinytufty2040/board"
	"github.com/itohio/tinytufty2040/console"
)

// The frame is drawn in three 80-row bands into a 51.2 KB buffer: widgets draw
// at memory speed, each band is pushed in one burst (no clear-then-draw
// flicker), and the rest of the frame's 153.6 KB stays free for the heap. A
// full-frame buffer left ≈ 45 KB of heap and ran out of memory; two 120-row
// bands still ran out when the PNG decoder (≈ 13 KB per decode) met a
// fragmented heap on the Images page after a tour of the other pages.
const bandRows = app.ScreenH / 3

var pixels [app.ScreenW * bandRows]uint16

// buildID names this firmware; tools/build sets it (-ldflags -X main.buildID=…)
// to <tinytufty2040 commit>[+]/<tinygui commit>[+]@<MMDD-HHMM>, + marking
// uncommitted changes.
var buildID = "unknown"

const frameTime = 33 * time.Millisecond

func main() {
	board.SetUSBSerial()
	go serialWriter() // the one writer to USB (console.Ring)
	waitForConsole(3 * time.Second)
	boot("start")
	var panel board.LCD
	panel.Configure()
	boot("lcd")
	hw := newBadge()
	hw.SetBacklight(0)
	boot("badge")

	dumping := false
	fb := framebuffer.NewBand(app.ScreenW, app.ScreenH, bandRows, pixels[:], func(f *framebuffer.RGB565) error {
		rows := f.Pix[:int(min(f.H, f.ScreenH-f.Y0))*int(f.W)]
		panel.Push(f.Y0, rows)
		if dumping {
			dumpRows(rows)
		}
		return nil
	})
	imuOK := setupIMU()
	boot("imu")
	a := app.New(fb, hw, app.Config{
		IMU:         imuOK,
		DurationAdd: ui.DurationAdd,
		DurationSub: ui.DurationSub,
		Log:         func(s string) { var l console.Line; l.S(s).Send(&con) },
		BuildID:     buildID,
	}, time.Now())
	boot("app")
	if !imuOK {
		reportIMU() // another part, or none: gravity from the buttons
	}

	// USB serial console: line commands, see app.Handlers ("help").
	handlers := a.Handlers()
	handlers["madctl"] = func(arg []byte) { // orientation probe: madctl <decimal>
		v := 0
		for _, c := range arg {
			v = v*10 + int(c-'0')
		}
		panel.SetMADCTL(uint8(v))
	}
	handlers["reboot"] = func([]byte) { arm.SystemReset() } // fresh state for scripted tours
	handlers["pins"] = func([]byte) {
		var l console.Line
		l.S("A").B(machine.BUTTON_A.Get()).S("B").B(machine.BUTTON_B.Get()).S("C").B(machine.BUTTON_C.Get()).
			S("UP").B(machine.BUTTON_UP.Get()).S("DOWN").B(machine.BUTTON_DOWN.Get()).S("USER").B(machine.BUTTON_USER.Get()).
			S("VBUS").B(machine.VBUS_DETECT.Get()).Send(&con)
	}
	handlers["imu"] = func([]byte) { // the sensor in use, or probe again after plugging one in
		if mpu != nil {
			var l console.Line
			l.S("imu: MPU6050 at 0x68 drives gravity and the Motion page").Send(&con)
			return
		}
		reportIMU()
	}
	handlers["imuread"] = func(arg []byte) { // imuread [n]: n samples at 10 Hz
		n := 10
		if len(arg) > 0 {
			n = 0
			for _, c := range arg {
				n = n*10 + int(c-'0')
			}
		}
		imuRead(n)
	}
	handlers["imuint"] = func([]byte) { imuINTCheck() }
	mux := ui.NewCommandStreamMux(idleReader{ui.NewSerialReader(machine.Serial)}, handlers)
	go mux.Run()

	buttons := []*button{
		newButton(machine.BUTTON_UP, app.BtnUp, true),
		newButton(machine.BUTTON_DOWN, app.BtnDown, true),
		newButton(machine.BUTTON_A, app.BtnA, false),
		newButton(machine.BUTTON_B, app.BtnB, false),
		newButton(machine.BUTTON_C, app.BtnC, false),
	}
	machine.BUTTON_USER.Configure(machine.PinConfig{Mode: machine.PinInputPullup}) // active low

	first := true
	var last time.Time
	for {
		now := time.Now()
		for _, b := range buttons {
			b.poll(a, now)
		}
		if !machine.BUTTON_USER.Get() {
			a.UserHold(ui.PeekButton(machine.BUTTON_USER)) // blocks until release
		}
		if now.Sub(last) >= frameTime {
			period := now.Sub(last) // start to start: everything the loop did since the last frame
			last = now
			m0 := mallocs()
			bus, samples := imuDrain(a) // every sample since the last frame (100 Hz): the shared motion state
			tIMU := time.Now()
			a.Update(now)
			tUpd := time.Now()
			m1 := mallocs()
			if dumping = a.TakeDump(); dumping {
				con.Stream() // header, pixels, END: nothing else in between
				var l console.Line
				l.S("FB").I(app.ScreenW).I(app.ScreenH)
				con.Write(l.Bytes())
				con.Write(lineEnd[:])
			}
			var draw, push time.Duration
			var md, mp uint64
			for i := range fb.Bands() {
				fb.SetY0(int16(i * bandRows))
				t0 := time.Now()
				d0 := mallocs()
				a.Draw()
				d1 := mallocs()
				t1 := time.Now()
				_ = fb.Display()
				md += d1 - d0
				mp += mallocs() - d1
				draw += t1.Sub(t0)
				push += time.Since(t1)
			}
			a.ReportTiming(draw, push)
			a.ReportProfile(period, tIMU.Sub(now), bus, samples, tUpd.Sub(tIMU), draw, push)
			a.ReportFrame(time.Since(now))
			a.ReportAllocs(int(m1-m0), int(md), int(mp))
			if dumping {
				con.Write(dumpEnd[:])
				con.EndStream()
			}
			if first { // light the panel only after the first full frame
				first = false
				hw.SetBacklight(80)
				boot("first frame")
			}
		}
		// the next frame already due (every page is slower than frameTime):
		// only yield, so the console goroutines still run; otherwise wait
		// 2 ms between button polls (TG-5 5f: the fixed sleep cost 2 ms a frame)
		if time.Since(last) >= frameTime {
			runtime.Gosched()
		} else {
			time.Sleep(2 * time.Millisecond)
		}
	}
}

// idleReader sleeps when the serial port has nothing buffered, so the mux
// goroutine (which yields with runtime.Gosched on every read) stops spinning.
type idleReader struct{ r io.Reader }

func (i idleReader) Read(b []byte) (int, error) {
	n, err := i.r.Read(b)
	if n == 0 && err == nil {
		time.Sleep(5 * time.Millisecond)
	}
	return n, err
}

var memStats runtime.MemStats

// mallocs is the runtime's allocation count (ReadMemStats doesn't allocate).
func mallocs() uint64 {
	runtime.ReadMemStats(&memStats)
	return memStats.Mallocs
}

// dumpRows streams pixels over USB serial as big-endian RGB565. The frame
// loop frames a full dump as: line "FB 320 240", 153 600 bytes (all three bands),
// newline, line "END" — a raw screenshot without a camera.
func dumpRows(pix []uint16) {
	var buf [512]byte
	n := 0
	for _, p := range pix {
		buf[n], buf[n+1] = byte(p>>8), byte(p)
		n += 2
		if n == len(buf) {
			con.Write(buf[:n]) // within the dump's stream (frame loop)
			n = 0
		}
	}
	if n > 0 {
		con.Write(buf[:n])
	}
}

// con is the one serial writer's queue: every byte the firmware sends goes
// through it (review R-TG-1 T4, T5).
var con console.Ring

var (
	lineEnd = [1]byte{'\n'}
	dumpEnd = [5]byte{'\n', 'E', 'N', 'D', '\n'}
)

// serialWriter drains con into the USB serial port: the only goroutine that
// writes to it, so a host holding the port open without reading stalls this
// goroutine, not the frame loop, and nothing interleaves. Lines dropped
// because the queue was full are reported once it has room again.
func serialWriter() {
	var buf [256]byte
	for {
		if n := con.Drain(buf[:]); n > 0 {
			_, _ = machine.Serial.Write(buf[:n])
			continue
		}
		if d := con.Dropped(); d > 0 {
			var l console.Line
			l.S("console: lines dropped (queue full)").U(uint64(d)).Send(&con)
			continue
		}
		time.Sleep(2 * time.Millisecond)
	}
}

// waitForConsole gives a host up to d to open the USB serial port, so the
// boot log below is not lost.
func waitForConsole(d time.Duration) {
	for end := time.Now().Add(d); time.Now().Before(end) && !machine.Serial.DTR(); {
		time.Sleep(10 * time.Millisecond)
	}
}

// boot prints a boot stage with the heap state.
func boot(stage string) {
	var m runtime.MemStats
	runtime.ReadMemStats(&m)
	var l console.Line
	l.S("boot:").S(stage).S("heap_alloc").U(m.HeapAlloc).S("heap_sys").U(m.HeapSys).Send(&con)
}

// reportIMU logs a motion sensor found without a driver here (or none):
// gravity then follows the buttons.
func reportIMU() {
	imu, ok := board.ProbeIMU()
	if !ok {
		var l console.Line
		l.S("imu: none found: gravity from the buttons").Send(&con)
		return
	}
	var l console.Line
	l.S("imu:").S(imu.Name).S("at").U(uint64(imu.Addr)).S("WHO_AM_I").U(uint64(imu.ID)).S("(no driver for it: gravity from the buttons)").Send(&con)
}
