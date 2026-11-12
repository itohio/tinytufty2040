//go:build tufty2040

package board

import "machine"

// IMU is what ProbeIMU found on the Qw/ST bus.
type IMU struct {
	Addr uint8
	ID   uint8  // the WHO_AM_I value read
	Name string // the part it identifies
}

// imuParts are the usual Qw/ST / STEMMA QT motion sensors: address,
// WHO_AM_I register and value.
var imuParts = []struct {
	addr, reg, id uint8
	name          string
}{
	{0x68, 0x00, 0xEA, "ICM20948"}, {0x69, 0x00, 0xEA, "ICM20948"},
	{0x68, 0x75, 0x68, "MPU6050"}, {0x69, 0x75, 0x68, "MPU6050"},
	{0x6A, 0x0F, 0x69, "LSM6DS3"}, {0x6B, 0x0F, 0x69, "LSM6DS3"},
	{0x6A, 0x0F, 0x6A, "LSM6DS3TR-C"}, {0x6B, 0x0F, 0x6A, "LSM6DS3TR-C"},
	{0x6A, 0x0F, 0x6C, "LSM6DSOX"}, {0x6B, 0x0F, 0x6C, "LSM6DSOX"},
	{0x6A, 0x0F, 0x68, "LSM9DS1"}, {0x6B, 0x0F, 0x68, "LSM9DS1"},
	{0x68, 0x00, 0xD1, "BMI160"}, {0x69, 0x00, 0xD1, "BMI160"},
	{0x6A, 0x00, 0x05, "QMI8658"}, {0x6B, 0x00, 0x05, "QMI8658"},
	{0x18, 0x0F, 0x33, "LIS3DH"}, {0x19, 0x0F, 0x33, "LIS3DH"},
	{0x26, 0x01, 0x13, "MSA301"},
}

// ProbeIMU looks for a known motion sensor on the Qw/ST bus (I2C0, GPIO4
// SDA / GPIO5 SCL): each known address is asked for its WHO_AM_I register.
// It returns ok false when none answers (the Tufty 2040 has none on board).
func ProbeIMU() (IMU, bool) {
	bus := machine.I2C0
	if err := bus.Configure(machine.I2CConfig{SDA: machine.GPIO4, SCL: machine.GPIO5, Frequency: 100_000}); err != nil {
		return IMU{}, false
	}
	var v [1]byte
	for _, p := range imuParts {
		if err := bus.Tx(uint16(p.addr), []byte{p.reg}, v[:]); err == nil && v[0] == p.id {
			return IMU{Addr: p.addr, ID: v[0], Name: p.name}, true
		}
	}
	return IMU{}, false
}
