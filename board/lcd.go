//go:build tufty2040

// Package board drives the Pimoroni Tufty 2040's panel: an ST7789 320×240 on
// an 8-bit parallel bus (8080 write cycles). Commands are bit-banged through
// the RP2040 SIO set/clear registers; PushRect's pixel stream goes out
// through PIO0 fed by DMA (lcdPIOSetup). Init sequence and orientation follow
// Pimoroni's driver (pimoroni-pico drivers/st7789, DISPLAY_TUFTY_2040:
// ROTATE_180 because the panel is mounted upside down).
package board

import (
	"device/arm"
	"device/rp"
	"machine"
	"runtime/volatile"
	"time"
	"unsafe"
)

const (
	Width  = 320
	Height = 240

	lcdDataShift = 14 // LCD_DB0..DB7 = GPIO14..21
	lcdDataMask  = uint32(0xFF) << lcdDataShift
	lcdWRMask    = uint32(1) << 12 // LCD_WR = GPIO12

	// MADCTLRotate180 is ROW_ORDER | SWAP_XY | SCAN_ORDER (landscape, upside down).
	MADCTLRotate180 = 0x80 | 0x20 | 0x10

	// The pixel stream (TG-5 5c): bit-banged by the CPU it took 126 ns a
	// byte, 19.3 ms a frame. PIO0's state machine 0 clocks it out at
	// 200 MHz / lcdPIOClkDiv: 20 ns a PIO cycle, two per phase, so 80 ns a
	// byte with WR low 40 ns and high 40 ns (ST7789: cycle ≥ 66 ns, each
	// phase ≥ 15 ns); a DMA channel feeds it from the band.
	lcdPIOClkDiv = 4
	lcdSM        = 0  // PIO0 state machine
	lcdDMA       = 10 // DMA channel (TinyGo's drivers use the low ones; the light capture 11)
	funcSIO      = 5  // GPIO function select: SIO (bit-banged commands)
	funcPIO0     = 6  // GPIO function select: PIO0 (the pixel stream)
	lcdWRPin     = 12 // LCD_WR (side-set); LCD_RD (GPIO13) stays SIO, high
)

// lcdProgram is PIO0's pixel loop (side-set: one pin, WR):
//
//	out pins, 8  side 0 [1]  ; the next byte on DB0..7, WR low (a stalled out holds WR low: harmless, the panel latches on the rising edge)
//	nop          side 1 [1]  ; WR high latches it
//
// Autopull every 16 bits, shifting left: each RGB565 pixel (a halfword the
// DMA writes, which the bus repeats in both halves of the FIFO word) goes
// out MSB first.
var lcdProgram = [...]uint16{0x6108, 0xB142}

// dmaChannel is one DMA channel's first four registers (RP2040 2.5.7).
type dmaChannel struct {
	read, write, count, ctrl volatile.Register32
}

func dmaCh(n uintptr) *dmaChannel {
	return (*dmaChannel)(unsafe.Pointer(uintptr(unsafe.Pointer(rp.DMA)) + n*0x40))
}

// gpioCtrl is GPIO pin's IO_BANK0 control register (function select).
func gpioCtrl(pin uintptr) *volatile.Register32 {
	return (*volatile.Register32)(unsafe.Pointer(uintptr(unsafe.Pointer(&rp.IO_BANK0.GPIO0_CTRL)) + pin*8))
}

// lcdPinsTo selects fn (funcSIO or funcPIO0) for WR and DB0..7.
func lcdPinsTo(fn uint32) {
	gpioCtrl(lcdWRPin).Set(fn)
	for pin := uintptr(lcdDataShift); pin < lcdDataShift+8; pin++ {
		gpioCtrl(pin).Set(fn)
	}
}

// lcdPIOSetup loads the pixel loop into PIO0 and starts state machine 0,
// stalled with an empty FIFO; the pins stay on SIO until a push.
func lcdPIOSetup() {
	p := rp.PIO0
	p.CTRL.ClearBits(1 << (rp.PIO0_CTRL_SM_ENABLE_Pos + lcdSM))
	for i, ins := range lcdProgram {
		(*volatile.Register32)(unsafe.Pointer(uintptr(unsafe.Pointer(&p.INSTR_MEM0)) + uintptr(i)*4)).Set(uint32(ins))
	}
	p.SM0_CLKDIV.Set(lcdPIOClkDiv << rp.PIO0_SM0_CLKDIV_INT_Pos)
	p.SM0_EXECCTRL.Set(uint32(len(lcdProgram)-1) << rp.PIO0_SM0_EXECCTRL_WRAP_TOP_Pos) // wrap bottom 0
	p.SM0_SHIFTCTRL.Set(rp.PIO0_SM0_SHIFTCTRL_FJOIN_TX | rp.PIO0_SM0_SHIFTCTRL_AUTOPULL |
		16<<rp.PIO0_SM0_SHIFTCTRL_PULL_THRESH_Pos) // OUT_SHIFTDIR 0: left, MSB first
	p.SM0_PINCTRL.Set(1<<rp.PIO0_SM0_PINCTRL_SIDESET_COUNT_Pos | 1<<rp.PIO0_SM0_PINCTRL_SET_COUNT_Pos |
		8<<rp.PIO0_SM0_PINCTRL_OUT_COUNT_Pos | lcdWRPin<<rp.PIO0_SM0_PINCTRL_SIDESET_BASE_Pos |
		lcdWRPin<<rp.PIO0_SM0_PINCTRL_SET_BASE_Pos | lcdDataShift<<rp.PIO0_SM0_PINCTRL_OUT_BASE_Pos)
	p.SM0_INSTR.Set(0xF001)                                    // set pins, 1 side 1: WR high
	p.SM0_INSTR.Set(0xF081)                                    // set pindirs, 1 side 1: WR an output
	p.SM0_INSTR.Set(0xB0EB)                                    // mov osr, ~null side 1
	p.SM0_INSTR.Set(0x7088)                                    // out pindirs, 8 side 1: DB0..7 outputs
	p.CTRL.SetBits(1 << (rp.PIO0_CTRL_SM_RESTART_Pos + lcdSM)) // clear the OSR's shift count
	p.SM0_INSTR.Set(0x1000)                                    // jmp 0 side 1
	p.CTRL.SetBits(1 << (rp.PIO0_CTRL_SM_ENABLE_Pos + lcdSM))
}

// LCD is the Tufty's ST7789 panel.
type LCD struct{}

// Configure sets up the bus pins and runs the panel init sequence.
func (l LCD) Configure() {
	for _, p := range []machine.Pin{
		machine.LCD_CS, machine.LCD_DC, machine.LCD_WR, machine.LCD_RD,
		machine.LCD_DB0, machine.LCD_DB1, machine.LCD_DB2, machine.LCD_DB3,
		machine.LCD_DB4, machine.LCD_DB5, machine.LCD_DB6, machine.LCD_DB7,
	} {
		p.Configure(machine.PinConfig{Mode: machine.PinOutput})
	}
	machine.LCD_CS.High()
	machine.LCD_RD.High() // never read
	machine.LCD_WR.High()

	l.Command(0x01) // SWRESET
	time.Sleep(150 * time.Millisecond)
	l.Command(0x35)       // TEON
	l.Command(0x3A, 0x05) // COLMOD: 16 bpp
	l.Command(0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33)
	l.Command(0xC0, 0x2C)
	l.Command(0xC2, 0x01)
	l.Command(0xC3, 0x12)
	l.Command(0xC4, 0x20)
	l.Command(0xD0, 0xA4, 0xA1)
	l.Command(0xC6, 0x0F)
	l.Command(0xB0, 0x00, 0xC0) // RAMCTRL
	l.Command(0xB7, 0x35)       // GCTRL (320×240)
	l.Command(0xBB, 0x1F)       // VCOMS (320×240)
	l.Command(0xE0, 0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D)
	l.Command(0xE1, 0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31)
	l.Command(0x21) // INVON
	l.Command(0x11) // SLPOUT
	l.Command(0x29) // DISPON
	time.Sleep(100 * time.Millisecond)
	l.SetMADCTL(MADCTLRotate180)
	lcdPIOSetup()
}

// SetMADCTL sets the memory access control (orientation) register.
func (l LCD) SetMADCTL(v uint8) { l.Command(0x36, v) }

// write clocks one byte out: data and WR low, then WR high latches it.
// ST7789 8080 write timing: WR low ≥ 15 ns, WR high ≥ 15 ns, cycle ≥ 66 ns.
// TinyGo 0.43 runs the RP2040 at 200 MHz (machine.CPUFrequency, measured), so
// one cycle is 5 ns: with two nops per phase a fill loop measured 42 ns per
// byte and the WR-low pulse was ~15 ns — out of spec. Six nops per phase
// measured 83 ns per byte (WR low ≈ 35 ns).
func write(b uint8) {
	rp.SIO.GPIO_OUT_CLR.Set(lcdDataMask | lcdWRMask)
	rp.SIO.GPIO_OUT_SET.Set(uint32(b) << lcdDataShift)
	arm.Asm("nop\nnop\nnop\nnop\nnop\nnop")
	rp.SIO.GPIO_OUT_SET.Set(lcdWRMask)
	arm.Asm("nop\nnop\nnop\nnop\nnop\nnop")
}

// Command sends a command byte followed by its parameters.
func (l LCD) Command(cmd uint8, args ...uint8) {
	machine.LCD_CS.Low()
	machine.LCD_DC.Low()
	write(cmd)
	machine.LCD_DC.High()
	for _, a := range args {
		write(a)
	}
	machine.LCD_CS.High()
}

// window selects the rectangle x0…x1, y0…y1 (inclusive) and starts a RAM write.
func (l LCD) window(x0, y0, x1, y1 int) {
	l.Command(0x2A, uint8(x0>>8), uint8(x0), uint8(x1>>8), uint8(x1)) // CASET
	l.Command(0x2B, uint8(y0>>8), uint8(y0), uint8(y1>>8), uint8(y1)) // RASET
	machine.LCD_CS.Low()
	machine.LCD_DC.Low()
	write(0x2C) // RAMWR
	machine.LCD_DC.High()
}

// Push sends full-width rows starting at y0 (RGB565 pixels, MSB first).
func (l LCD) Push(y0 int16, pix []uint16) {
	l.PushRect(0, y0, Width, int16(len(pix)/Width), pix)
}

// PushRect sends a w×h block of RGB565 pixels to (x, y): partial updates
// cost only the pixels of the rectangle. The window commands are bit-banged;
// the pixels go out through PIO0 fed by DMA, and PushRect returns once the
// last byte is latched (the caller then draws into the same buffer).
func (l LCD) PushRect(x, y, w, h int16, pix []uint16) {
	n := int(w) * int(h)
	l.window(int(x), int(y), int(x+w)-1, int(y+h)-1)
	if n > 0 {
		p := rp.PIO0
		lcdPinsTo(funcPIO0)
		ch := dmaCh(lcdDMA)
		ch.read.Set(uint32(uintptr(unsafe.Pointer(&pix[0]))))
		ch.write.Set(uint32(uintptr(unsafe.Pointer(&p.TXF0))))
		ch.count.Set(uint32(n))
		ch.ctrl.Set(rp.DMA_CH0_CTRL_TRIG_INCR_READ |
			rp.DMA_CH0_CTRL_TRIG_DATA_SIZE_SIZE_HALFWORD<<rp.DMA_CH0_CTRL_TRIG_DATA_SIZE_Pos |
			rp.DREQ_PIO0_TX0<<rp.DMA_CH0_CTRL_TRIG_TREQ_SEL_Pos |
			lcdDMA<<rp.DMA_CH0_CTRL_TRIG_CHAIN_TO_Pos | // chained to itself: no chain
			rp.DMA_CH0_CTRL_TRIG_EN)
		for ch.ctrl.Get()&rp.DMA_CH0_CTRL_TRIG_BUSY != 0 {
		}
		// the FIFO drained, then a fresh stall: the last byte's WR rise is done
		empty := uint32(1) << (rp.PIO0_FSTAT_TXEMPTY_Pos + lcdSM)
		for p.FSTAT.Get()&empty == 0 {
		}
		stall := uint32(1) << (rp.PIO0_FDEBUG_TXSTALL_Pos + lcdSM)
		p.FDEBUG.Set(stall)
		for p.FDEBUG.Get()&stall == 0 {
		}
	}
	machine.LCD_CS.High() // before WR goes back to SIO (high): no stray latch
	lcdPinsTo(funcSIO)
}

// PushIndexed sends a w×h block of 8-bit palette indices to (x, y), mapping
// each through lut to RGB565 on the way out (half the RAM of RGB565).
func (l LCD) PushIndexed(x, y, w, h int16, pix []uint8, lut *[256]uint16) {
	l.window(int(x), int(y), int(x+w)-1, int(y+h)-1)
	for _, i := range pix[:int(w)*int(h)] {
		p := lut[i]
		write(uint8(p >> 8))
		write(uint8(p))
	}
	machine.LCD_CS.High()
}

// PushIndexed4 sends a w×h block of 4-bit palette indices (two pixels per
// byte, high nibble first) to (x, y) through lut (a quarter of RGB565's RAM).
func (l LCD) PushIndexed4(x, y, w, h int16, pix []uint8, lut *[16]uint16) {
	l.window(int(x), int(y), int(x+w)-1, int(y+h)-1)
	for _, b := range pix[:(int(w)*int(h)+1)/2] {
		p := lut[b>>4]
		write(uint8(p >> 8))
		write(uint8(p))
		p = lut[b&15]
		write(uint8(p >> 8))
		write(uint8(p))
	}
	machine.LCD_CS.High()
}

// Fill paints the whole panel one RGB565 colour without a buffer.
func (l LCD) Fill(c uint16) {
	l.window(0, 0, Width-1, Height-1)
	for range Width * Height {
		write(uint8(c >> 8))
		write(uint8(c))
	}
	machine.LCD_CS.High()
}
