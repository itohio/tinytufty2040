package mpufifo

import "testing"

// A FIFO count is read as whole samples, unless it says the FIFO
// overflowed (MATH-1 D1): a full FIFO (the sensor then writes over the
// oldest bytes, and 1024 isn't a multiple of 12, so the next read starts
// mid-sample). A count read while a sample is being written isn't whole
// samples, and isn't an overflow: the whole ones are read, the rest waits. A misaligned read
// decodes as a fake spin of 0.4–4.4 rad/s at rest (the owner's capture).
func TestSamples(t *testing.T) {
	for _, c := range []struct {
		count, n int
		ok       bool
	}{
		{0, 0, true},
		{12, 1, true},
		{48, 4, true},
		{1020, 85, true}, // the last whole count under full
		{1024, 0, false}, // full: overflowed
		{1030, 0, false},
		{13, 1, true}, // a sample being written: one whole, the rest waits
		{1018, 84, true},
	} {
		n, ok := Samples(c.count)
		if n != c.n || ok != c.ok {
			t.Errorf("count %d: %d samples, ok %v; want %d, %v", c.count, n, ok, c.n, c.ok)
		}
	}
}
