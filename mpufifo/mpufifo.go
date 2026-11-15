// Package mpufifo reads an MPU-6050 FIFO count before a drain (MATH-1 D1).
//
// The FIFO holds Size bytes; the drain stores accel and gyro, Sample bytes
// a sample. When the FIFO fills, the sensor writes over its oldest bytes
// and, Size not being a multiple of Sample, every read after that starts in
// the middle of a sample: the axes decode shifted and a still badge shows a
// fake spin. A full FIFO is that overflow; the drain also tests the status
// bit (FIFO_OFLOW_EN is on). A count that isn't whole samples is not one:
// the count can be read while the sensor writes a sample, and the partial
// sample stays for the next drain.
package mpufifo

const (
	Size   = 1024 // bytes the FIFO holds
	Sample = 12   // bytes a sample: accel then gyro, 3 × int16 each
)

// Samples is how many whole samples a FIFO count holds, or ok false when
// the count says the FIFO overflowed (full). Then the drain restarts the
// FIFO and feeds nothing.
func Samples(count int) (n int, ok bool) {
	if count >= Size {
		return 0, false
	}
	return count / Sample, true
}
