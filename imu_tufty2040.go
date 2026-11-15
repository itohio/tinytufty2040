//go:build tufty2040

package main

import (
	"machine"
	"sync"
	"time"

	"github.com/itohio/EasyRobot/x/devices/mpu6050"
	"github.com/itohio/tinytufty2040/app"
	"github.com/itohio/tinytufty2040/console"
	"github.com/itohio/tinytufty2040/mpufifo"
)

// The owner's MPU6050 sits on the debug pads JP1 (3V3, SDA pad 9, SCL pad 8,
// INT pad 7?, GND): the same I2C nets as Qw/ST, I2C0 on GPIO4/GPIO5; JP1's
// I2C_INT is GPIO3.

var (
	mpu    *mpu6050.Device
	mpuCfg = mpu6050.Config{
		SampleRateDiv:      9, // 1 kHz / 10 = 100 Hz
		DLPF:               mpu6050.DLPF44Hz,
		Accel:              mpu6050.Accel8G,     // a hand's shake and taps reach past ±2 g (the M5 capture clipped there)
		Gyro:               mpu6050.Gyro2000DPS, // ±500 clipped a brisk pick-up (TG-6 capture 0); 0.06 °/s a step, far under the rest thresholds
		DataReadyInterrupt: true,

		// INT_STATUS bit 4 is set on an overflow only with this on (MATH-1
		// D1); any read clears it, so the drain's FIFO count check stays
		FIFOOverflowInterrupt: true,
	}
	imuINT = machine.GPIO3
)

// setupIMU configures the MPU6050 if it answers on I2C0 (GPIO4 SDA, GPIO5
// SCL, the Qw/ST and JP1 nets). The bus runs at 400 kHz (the MPU6050's fast
// mode): at 100 kHz each 12-byte sample took 1.3 ms of bus time, 13 % of the
// CPU at 100 Hz (TG-5 #4's profile).
func setupIMU() bool {
	i2cRecover(machine.GPIO4, machine.GPIO5)
	if err := machine.I2C0.Configure(machine.I2CConfig{SDA: machine.GPIO4, SCL: machine.GPIO5, Frequency: 400_000}); err != nil {
		return false
	}
	d := mpu6050.New(machine.I2C0, 0)
	id, ok, err := d.Probe()
	if err != nil || !ok {
		_ = id
		return false
	}
	if err := d.Configure(mpuCfg); err != nil {
		var l console.Line
		l.S("imu: mpu6050 configure:").S(err.Error()).Send(&con)
		return false
	}
	imuINT.Configure(machine.PinConfig{Mode: machine.PinInputPulldown})
	if err := fifoStart(); err != nil {
		var l console.Line
		l.S("imu: fifo:").S(err.Error()).Send(&con)
		return false
	}
	mpu = d
	// the ranges as read back from the sensor: AFS_SEL and FS_SEL in bits 4:3
	var ac, gc [1]byte
	_ = mpuRead(regAccelConfig, ac[:])
	_ = mpuRead(regGyroConfig, gc[:])
	var l console.Line
	l.S("imu: MPU6050 at 0x68, 100 Hz, DLPF 44 Hz, full scale").U(uint64(2) << (ac[0] >> 3 & 3)).S("g and").U(uint64(250) << (gc[0] >> 3 & 3)).S("°/s: gravity from the tilt").Send(&con)
	return true
}

// i2cRecover frees a bus a device still holds. A reset in the middle of a
// read (a 1200-baud touch, a flash) leaves the MPU6050 driving SDA low and
// waiting for clocks, and the reset doesn't power it down (JP1 pads), so
// the first transaction after boot never completes: the badge hung in
// setupIMU after a firmware that read it at 100 Hz (2026-11-13). The fix is
// the standard bus recovery: up to 9 clocks on SCL until SDA goes high,
// then a STOP (SDA rising while SCL is high).
func i2cRecover(sda, scl machine.Pin) {
	const half = 5 * time.Microsecond // 100 kHz
	sda.Configure(machine.PinConfig{Mode: machine.PinInputPullup})
	scl.Configure(machine.PinConfig{Mode: machine.PinOutput})
	scl.High()
	time.Sleep(half)
	for i := 0; i < 9 && !sda.Get(); i++ {
		scl.Low()
		time.Sleep(half)
		scl.High()
		time.Sleep(half)
	}
	if !sda.Get() {
		return // still held: nothing more to do without a power cycle
	}
	sda.Configure(machine.PinConfig{Mode: machine.PinOutput})
	scl.Low()
	sda.Low()
	time.Sleep(half)
	scl.High()
	time.Sleep(half)
	sda.High() // STOP
	time.Sleep(half)
}

// The sensor's FIFO (MPU-6000/6050 register map 4.6, 4.7, 4.17, 4.18):
// the demo reads every sample (100 Hz) though a frame takes up to 100 ms,
// so the motion state integrates at the sensor's rate (owner 2026-11-13:
// at one sample a frame the attitude lagged a tilt and gravity leaked into
// the path). Accel then gyro, 12 bytes a sample, big-endian; 1024 bytes
// hold 0.85 s. The driver (EasyRobot x/devices/mpu6050) has no FIFO API
// yet: these registers are read directly on its bus.
const (
	mpuAddr        = 0x68
	regGyroConfig  = 0x1B // GYRO_CONFIG: FS_SEL in bits 4:3
	regAccelConfig = 0x1C // ACCEL_CONFIG: AFS_SEL in bits 4:3
	regFIFOEn      = 0x23 // FIFO_EN: which outputs go into the FIFO
	regIntStatus   = 0x3A // INT_STATUS: bit 4 FIFO_OFLOW_INT
	regUserCtrl    = 0x6A // USER_CTRL: bit 6 FIFO_EN, bit 2 FIFO_RESET
	regFIFOCount   = 0x72 // FIFO_COUNT_H, _L
	regFIFORW      = 0x74 // FIFO_R_W
	fifoAccGyro    = 0x78 // XG, YG, ZG, ACCEL
	fifoSample     = mpufifo.Sample
	fifoBatch      = 10 // samples per burst read
)

var (
	fifoW   [2]byte
	fifoR   [fifoSample * fifoBatch]byte
	fifoCnt [2]byte
)

func mpuWrite(reg, v byte) error {
	fifoW[0], fifoW[1] = reg, v
	return machine.I2C0.Tx(mpuAddr, fifoW[:2], nil)
}

func mpuRead(reg byte, dst []byte) error {
	fifoW[0] = reg
	return machine.I2C0.Tx(mpuAddr, fifoW[:1], dst)
}

// fifoStart empties the FIFO and starts filling it with accel and gyro.
func fifoStart() error {
	if err := mpuWrite(regUserCtrl, 0x04); err != nil { // reset
		return err
	}
	if err := mpuWrite(regFIFOEn, fifoAccGyro); err != nil {
		return err
	}
	return mpuWrite(regUserCtrl, 0x40) // enable
}

// i2cMu gives I2C0 to one goroutine at a time. TinyGo's I2C driver yields
// inside a transfer (gosched while it waits) and starts every transfer by
// disabling the peripheral, so a console IMU command reading while the main
// loop drained broke the transfer ("i2c: peripheral timeout in disable":
// every imuread failed once the loop yielded instead of sleeping, TG-5 5f).
// The drain holds it for its whole pass, the console commands for each read
// (never across their sleeps: a drain held off past 0.85 s loses samples).
var i2cMu sync.Mutex

// mpuSample and mpuRaw are the console commands' locked reads.
func mpuSample(s *mpu6050.Sample) error {
	i2cMu.Lock()
	err := mpu.Read(s)
	i2cMu.Unlock()
	return err
}

func mpuRaw(r *mpu6050.Raw) error {
	i2cMu.Lock()
	err := mpu.ReadRaw(r)
	i2cMu.Unlock()
	return err
}

// imuDrain feeds every sample waiting in the FIFO to the app, at the
// sensor's own period. After an overflow (a stall longer than 0.85 s) the
// FIFO restarts: samples are lost, none are misread. For the frame profile
// it returns its time on the I2C bus and the samples fed: the rest of its
// time is the motion maths (SetIMU).
func imuDrain(a *app.App) (bus time.Duration, samples int) {
	if mpu == nil {
		return 0, 0
	}
	i2cMu.Lock()
	bus, samples = imuDrainLocked(a)
	i2cMu.Unlock()
	return bus, samples
}

// imuErrs and imuOvf count the drain's I2C errors and FIFO overflow
// restarts since boot (stats: prof imu err … ovf …): both stay 0 while the
// bus is shared safely and the loop keeps up.
var imuErrs, imuOvf int

func imuDrainLocked(a *app.App) (bus time.Duration, samples int) {
	t0 := time.Now()
	var st [1]byte
	switch err := mpuRead(regIntStatus, st[:]); {
	case err != nil:
		imuErrs++ // read the count anyway
	case st[0]&mpu6050.IntStatusFIFOOverflow != 0:
		imuOvf++
		_ = fifoStart()
		return time.Since(t0), 0
	}
	if mpuRead(regFIFOCount, fifoCnt[:]) != nil {
		imuErrs++
		return time.Since(t0), 0
	}
	bus = time.Since(t0)
	// the count too, not only the status bit: a full FIFO means an
	// overflow, and reading on would start mid-sample (a fake spin at rest)
	n, whole := mpufifo.Samples(int(fifoCnt[0])<<8 | int(fifoCnt[1]))
	if !whole {
		imuOvf++
		_ = fifoStart()
		return bus, 0
	}
	cfg := mpu.Config()
	dt := 1 / cfg.SampleRate()
	for n > 0 {
		k := min(n, fifoBatch)
		t1 := time.Now()
		err := mpuRead(regFIFORW, fifoR[:k*fifoSample])
		bus += time.Since(t1)
		if err != nil {
			imuErrs++
			return bus, samples
		}
		for i := range k {
			b := fifoR[i*fifoSample:]
			var r mpu6050.Raw
			for j := range 3 {
				r.Accel[j] = int16(uint16(b[2*j])<<8 | uint16(b[2*j+1]))
				r.Gyro[j] = int16(uint16(b[6+2*j])<<8 | uint16(b[7+2*j]))
			}
			s := cfg.Scale(r)
			// the chip faces away from the screen: badge right = −X, up = +Y, out = −Z
			a.SetIMU([3]float32{-s.Accel[0], s.Accel[1], -s.Accel[2]}, [3]float32{-s.Gyro[0], s.Gyro[1], -s.Gyro[2]}, dt)
			samples++
		}
		n -= k
	}
	return bus, samples
}

// imuRead prints n samples, 10 per second: accel m/s², gyro rad/s ×1000,
// temperature.
func imuRead(n int) {
	if mpu == nil {
		var l console.Line
		l.S("imu: none").Send(&con)
		return
	}
	var s mpu6050.Sample
	for range n {
		var l console.Line
		if err := mpuSample(&s); err != nil {
			l.S("imu: read:").S(err.Error()).Send(&con)
			return
		}
		l.S("imu a").F2(s.Accel[0]).F2(s.Accel[1]).F2(s.Accel[2]).
			S("g").I(int(s.Gyro[0] * 1000)).I(int(s.Gyro[1] * 1000)).I(int(s.Gyro[2] * 1000)).S("t").I(int(s.TempC)).Send(&con)
		time.Sleep(100 * time.Millisecond)
	}
}

// imuINTCheck tells whether JP1's INT (pad 7, GPIO3) carries the
// MPU6050's data-ready signal. Phase 1, 0.5 s without reading samples: a
// wired, latched INT rises once and stays high. Phase 2, 1 s reading a
// sample at each rise (a read clears the latch): about 100 rises if wired.
// An unconnected GPIO3 with its pull-down stays low in both.
func imuINTCheck() {
	if mpu == nil {
		var l console.Line
		l.S("imu: none").Send(&con)
		return
	}
	var r mpu6050.Raw
	_ = mpuRaw(&r) // clear a pending INT
	rises, highs, polls := 0, 0, 0
	prev := imuINT.Get()
	end := time.Now().Add(500 * time.Millisecond)
	for time.Now().Before(end) {
		v := imuINT.Get()
		polls++
		if v {
			highs++
		}
		if v && !prev {
			rises++
		}
		prev = v
	}
	var l console.Line
	l.S("imu: INT phase 1 (no reads, 0.5 s): rises").I(rises).S("high").I(highs).S("of").I(polls).S("polls").Send(&con)
	_ = mpuRaw(&r)
	rises, highs, polls = 0, 0, 0
	prev = imuINT.Get()
	end = time.Now().Add(time.Second)
	for time.Now().Before(end) {
		v := imuINT.Get()
		polls++
		if v {
			highs++
		}
		if v && !prev {
			rises++
			_ = mpuRaw(&r)
			v = imuINT.Get()
		}
		prev = v
	}
	l.Reset()
	l.S("imu: INT phase 2 (read on each rise, 1 s): rises").I(rises).S("high").I(highs).S("of").I(polls).S("polls (wired: ~100)").Send(&con)
}

func itoa(v int) string {
	if v == 0 {
		return "0"
	}
	var b [12]byte
	i := len(b)
	for v > 0 {
		i--
		b[i] = byte('0' + v%10)
		v /= 10
	}
	return string(b[i:])
}
