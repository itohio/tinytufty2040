package app

import (
	"encoding/binary"
	"errors"
	"math"
	"strconv"

	"github.com/itohio/EasyRobot/x/math/nav/calib"
)

// The calibration survives power-off (owner: "I don't want to recalibrate
// every time i turn it on"). The MPU6050 keeps nothing (its offset
// registers reset at power-off, and it has no scale registers), so the fit
// is one record in the board's own storage (the RP2040's flash on the
// badge, CalStore): saved when a full fit is put in use and by `cal set`,
// erased by `cal clear`, and put in use at boot when it is whole and was
// made at the sensor's present ranges.

// CalStore keeps one calibration record (calRecordSize bytes).
type CalStore interface {
	Read(p []byte) error  // the record as stored (erased storage reads 0xff)
	Write(p []byte) error // replace it
	Erase() error         // forget it
}

// The sensor's ranges as the firmware sets them (imu_tufty2040.go: ±2 g,
// ±2000 °/s): a record made at other ranges isn't used.
const (
	AccelFullScale = 2 * gravity
	GyroFullScale  = 2000 * math.Pi / 180
)

// The record: magic, version, model, the ranges it was made at, the fit,
// and a CRC-32 over everything before it (little-endian throughout).
const (
	calRecordSize    = 64
	calRecordVersion = 1
	calMagic         = 0x4c414354 // "TCAL"
)

var (
	errCalNone    = errors.New("no saved calibration")
	errCalDamaged = errors.New("damaged (CRC)")
	errCalVersion = errors.New("another record version")
	errCalRange   = errors.New("made at another range")
)

func encodeCalRecord(b *[calRecordSize]byte, f *calib.Fit, accelRange, gyroRange float32) {
	*b = [calRecordSize]byte{}
	le := binary.LittleEndian
	le.PutUint32(b[0:], calMagic)
	b[4], b[5] = calRecordVersion, byte(f.Model)
	putF32(b[8:], accelRange)
	putF32(b[12:], gyroRange)
	for i := range 3 {
		putF32(b[16+4*i:], f.Offset[i])
		putF32(b[28+4*i:], f.Scale[i])
		putF32(b[40+4*i:], f.GyroBias[i])
	}
	putF32(b[52:], f.RMS)
	putF32(b[56:], f.Cond)
	le.PutUint32(b[60:], crc32(b[:60]))
}

func decodeCalRecord(b *[calRecordSize]byte, accelRange, gyroRange float32) (calib.Fit, error) {
	le := binary.LittleEndian
	var f calib.Fit
	if le.Uint32(b[0:]) != calMagic {
		return f, errCalNone
	}
	if le.Uint32(b[60:]) != crc32(b[:60]) {
		return f, errCalDamaged
	}
	if b[4] != calRecordVersion {
		return f, errCalVersion
	}
	if !near(getF32(b[8:]), accelRange) || !near(getF32(b[12:]), gyroRange) {
		return f, errCalRange
	}
	f.Model = calib.Model(b[5])
	for i := range 3 {
		f.Offset[i], f.Scale[i], f.GyroBias[i] = getF32(b[16+4*i:]), getF32(b[28+4*i:]), getF32(b[40+4*i:])
	}
	f.RMS, f.Cond = getF32(b[52:]), getF32(b[56:])
	return f, nil
}

func near(a, b float32) bool { return a-b < 1e-3 && b-a < 1e-3 }

func putF32(b []byte, v float32) { binary.LittleEndian.PutUint32(b, math.Float32bits(v)) }
func getF32(b []byte) float32    { return math.Float32frombits(binary.LittleEndian.Uint32(b)) }

// crc32 is the IEEE CRC-32, bit by bit (hash/crc32 builds its table on
// first use: an allocation after setup).
func crc32(p []byte) uint32 {
	c := ^uint32(0)
	for _, x := range p {
		c ^= uint32(x)
		for range 8 {
			c = c>>1 ^ 0xedb88320&-(c&1)
		}
	}
	return ^c
}

// calSavedState is what the store holds, as cal reports it.
type calSavedState uint8

const (
	calSavedNone   calSavedState = iota // nothing saved (or erased)
	calSavedInUse                       // the saved calibration is the one in use
	calSavedUnused                      // one is saved but not used (calSavedErr says why)
	calSavedOther                       // one is saved, but a newer fit (not saved) is in use
)

// setCalSaved records what the store holds; the Calibration page says
// "saved" only while the fit in use is the saved one.
func (a *App) setCalSaved(st calSavedState, err error) {
	a.calSaved, a.calSavedErr = st, err
	if a.calib != nil {
		a.calib.g.saved = st == calSavedInUse
	}
}

// calNewFitInUse notes that a fit other than the saved one went into use
// (A on a fit, cal set): until it is saved, the saved one isn't in use.
func (a *App) calNewFitInUse() {
	if a.calSaved == calSavedInUse {
		a.setCalSaved(calSavedOther, nil)
	}
}

// calSavedLine logs what is saved: the boot line comes before a host
// connects, so cal says it again.
func (a *App) calSavedLine() {
	switch {
	case a.calStore == nil:
		a.log("cal saved: nowhere (no store)")
	case a.calSaved == calSavedInUse:
		a.log("cal saved: in use")
	case a.calSaved == calSavedOther:
		a.log("cal saved: not used: a newer fit is in use")
	case a.calSaved == calSavedUnused && a.calSavedErr != nil:
		a.log2("cal saved: not used: ", a.calSavedErr.Error())
	default:
		a.log("cal saved: none")
	}
}

// calLoad puts the saved calibration in use at boot, when it is whole and
// made at the sensor's ranges; the log says what happened.
func (a *App) calLoad() {
	if a.calStore == nil || a.calib == nil {
		return
	}
	if err := a.calStore.Read(a.calRec[:]); err != nil {
		a.setCalSaved(calSavedUnused, err)
		a.log2("cal: can't read the saved calibration: ", err.Error())
		return
	}
	f, err := decodeCalRecord(&a.calRec, AccelFullScale, GyroFullScale)
	if err == errCalNone {
		a.setCalSaved(calSavedNone, nil)
		a.log("cal: no saved calibration")
		return
	}
	if err != nil {
		a.setCalSaved(calSavedUnused, err)
		a.log2("cal: saved one not used: ", err.Error())
		return
	}
	a.calib.putInUse(f)
	a.setCalSaved(calSavedInUse, nil)
	a.log2("cal: the saved calibration is in use: ", f.Model.String())
}

// calSave writes the fit in use as the saved calibration.
func (a *App) calSave() {
	if a.calStore == nil {
		return
	}
	encodeCalRecord(&a.calRec, &a.calib.g.use, AccelFullScale, GyroFullScale)
	if err := a.calStore.Write(a.calRec[:]); err != nil {
		a.log2("cal: not saved: ", err.Error())
		return
	}
	a.setCalSaved(calSavedInUse, nil)
	a.log("cal: saved")
}

// calCommand is the console's cal: no argument prints the calibration
// (calReport); clear erases the saved one; set writes one by hand.
func (a *App) calCommand(arg string) {
	switch {
	case arg == "":
		a.calSavedLine()
		a.calReport()
	case arg == "clear":
		if a.calStore == nil {
			a.log("cal: nowhere to save")
			return
		}
		if err := a.calStore.Erase(); err != nil {
			a.log2("cal clear: ", err.Error())
			return
		}
		a.setCalSaved(calSavedNone, nil)
		a.log("cal: saved calibration erased")
	case len(arg) > 4 && arg[:4] == "set ":
		a.calSetCommand(arg[4:])
	default:
		a.log("cal [clear | set ox oy oz sx sy sz gx gy gz]")
	}
}

// calSetCommand puts a fit given by hand in use and saves it: offsets
// (m/s²), scales and gyro bias (rad/s); implausible numbers are refused.
func (a *App) calSetCommand(s string) {
	if a.calib == nil {
		a.log("cal set: no IMU")
		return
	}
	var v [9]float32
	n := 0
	for len(s) > 0 {
		for len(s) > 0 && s[0] == ' ' {
			s = s[1:]
		}
		end := 0
		for end < len(s) && s[end] != ' ' {
			end++
		}
		if end == 0 {
			break
		}
		if n == len(v) {
			n++ // too many
			break
		}
		x, err := strconv.ParseFloat(s[:end], 32)
		if err != nil {
			a.log("cal set: not a number")
			return
		}
		v[n], n, s = float32(x), n+1, s[end:]
	}
	if n != len(v) {
		a.log("cal set: 9 numbers: ox oy oz sx sy sz gx gy gz")
		return
	}
	f := calib.Fit{Model: calib.OffsetScale}
	for i := range 3 {
		f.Offset[i], f.Scale[i], f.GyroBias[i] = v[i], v[3+i], v[6+i]
		if f.Offset[i] < -2 || f.Offset[i] > 2 || f.Scale[i] < 0.8 || f.Scale[i] > 1.25 || f.GyroBias[i] < -0.1 || f.GyroBias[i] > 0.1 {
			a.log("cal set: offsets within 2 m/s2, scales 0.8-1.25, gyro 0.1 rad/s")
			return
		}
	}
	a.calib.putInUse(f)
	a.calNewFitInUse()
	a.log("cal set: in use")
	a.calSave()
}
