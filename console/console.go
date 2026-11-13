// Package console is the badge's one serial writer (review R-TG-1 T4, T5):
// every byte the firmware sends — log lines, console replies, framebuffer
// and capture dumps — goes through a Ring, and one goroutine drains the ring
// into the USB serial port.
//
// Producers never write to USB themselves, so a host that holds the port
// open without reading stalls only the draining goroutine, not the frame
// loop: a line that doesn't fit is dropped and counted (Dropped). A stream
// (a dump) holds the producer lock from its header to its END, so no line
// lands inside it — a line written meanwhile, by any goroutine (even the
// stream's own), is dropped and counted, never waited for; the stream
// itself waits for room instead of dropping. Nothing allocates.
package console

import (
	"runtime"
	"sync"
	"sync/atomic"
)

// size is the ring's capacity in bytes (a power of two).
const size = 2048

// Ring is a byte ring with any number of producers (serialised by a lock)
// and one consumer (Drain), which needs no lock.
type Ring struct {
	mu      sync.Mutex
	buf     [size]byte
	r, w    atomic.Uint32 // read and write positions, free-running
	dropped atomic.Uint32
}

func (q *Ring) free() uint32 { return size - (q.w.Load() - q.r.Load()) }

func (q *Ring) put(p []byte) {
	w := q.w.Load()
	for i, c := range p {
		q.buf[(w+uint32(i))&(size-1)] = c
	}
	q.w.Store(w + uint32(len(p)))
}

// WriteLine queues p and a newline whole, or drops it (counted) if the ring
// hasn't room or a stream is in progress: it never waits.
func (q *Ring) WriteLine(p []byte) {
	if !q.mu.TryLock() {
		q.dropped.Add(1)
		return
	}
	if uint32(len(p))+1 > q.free() {
		q.dropped.Add(1)
	} else {
		q.put(p)
		q.put(newline[:])
	}
	q.mu.Unlock()
}

var newline = [1]byte{'\n'}

// Stream starts a stream (a dump): until EndStream, only this producer
// writes, and Write waits for room instead of dropping.
func (q *Ring) Stream() { q.mu.Lock() }

// Write queues p within a stream, waiting (yielding) while the ring is full.
func (q *Ring) Write(p []byte) {
	for len(p) > 0 {
		n := min(uint32(len(p)), q.free())
		if n == 0 {
			runtime.Gosched() // the consumer makes room
			continue
		}
		q.put(p[:n])
		p = p[n:]
	}
}

// EndStream lets the other producers write again.
func (q *Ring) EndStream() { q.mu.Unlock() }

// Drain moves queued bytes into dst (the consumer) and returns how many.
func (q *Ring) Drain(dst []byte) int {
	r := q.r.Load()
	n := min(q.w.Load()-r, uint32(len(dst)))
	for i := range n {
		dst[i] = q.buf[(r+i)&(size-1)]
	}
	q.r.Store(r + n)
	return int(n)
}

// Dropped is the number of lines dropped since the last call.
func (q *Ring) Dropped() uint32 { return q.dropped.Swap(0) }

// Line builds one line of text without allocating: values appended with
// S/I/U/B/F2, sent with Send. Make one per call site (it is small and lives
// on the stack); text past its capacity is cut.
type Line struct {
	b [120]byte
	n int
}

// S appends text; a space goes before every part but the first.
func (l *Line) S(s string) *Line {
	l.sep()
	l.n += copy(l.b[l.n:], s)
	return l
}

// I appends a signed integer.
func (l *Line) I(v int) *Line {
	l.sep()
	if v < 0 {
		l.raw('-')
		return l.digits(uint64(-v))
	}
	return l.digits(uint64(v))
}

// U appends an unsigned integer.
func (l *Line) U(v uint64) *Line { l.sep(); return l.digits(v) }

// B appends true or false.
func (l *Line) B(v bool) *Line {
	if v {
		return l.S("true")
	}
	return l.S("false")
}

// F2 appends v with two decimals (rounded).
func (l *Line) F2(v float32) *Line {
	l.sep()
	if v < 0 {
		l.raw('-')
		v = -v
	}
	c := int64(v*100 + 0.5)
	l.digits(uint64(c / 100))
	l.raw('.')
	l.raw(byte('0' + c/10%10))
	l.raw(byte('0' + c%10))
	return l
}

// Send queues the line on q (dropped, counted, if q is full).
func (l *Line) Send(q *Ring) { q.WriteLine(l.b[:l.n]) }

// Reset empties the line.
func (l *Line) Reset() { l.n = 0 }

// Bytes is the text so far.
func (l *Line) Bytes() []byte { return l.b[:l.n] }

func (l *Line) sep() {
	if l.n > 0 {
		l.raw(' ')
	}
}

func (l *Line) raw(c byte) {
	if l.n < len(l.b) {
		l.b[l.n] = c
		l.n++
	}
}

func (l *Line) digits(v uint64) *Line {
	var d [20]byte
	i := len(d)
	for {
		i--
		d[i] = byte('0' + v%10)
		v /= 10
		if v == 0 {
			break
		}
	}
	l.n += copy(l.b[l.n:], d[i:])
	return l
}
