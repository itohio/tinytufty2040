package console

import (
	"bytes"
	"sync"
	"testing"
)

func drainAll(q *Ring) []byte {
	var out, chunk = []byte{}, make([]byte, 100)
	for {
		n := q.Drain(chunk)
		if n == 0 {
			return out
		}
		out = append(out, chunk[:n]...)
	}
}

// Lines come out whole, in order, each with its newline; a line that doesn't
// fit is dropped whole and counted — the producer never waits.
func TestLinesInOrderDroppedWholeWhenFull(t *testing.T) {
	var q Ring
	var l Line
	l.S("pong up_s").I(12).S("frames").U(345).S("ok").B(true).S("a").F2(-0.735)
	l.Send(&q)
	if got := string(drainAll(&q)); got != "pong up_s 12 frames 345 ok true a -0.74\n" {
		t.Fatalf("got %q", got)
	}
	long := bytes.Repeat([]byte("x"), 1000)
	fit := size / 1001  // lines of 1000 bytes + newline that fit the ring
	for range fit + 1 { // the last one doesn't fit
		q.WriteLine(long)
	}
	if d := q.Dropped(); d != 1 {
		t.Fatalf("dropped %d, want 1", d)
	}
	if got := drainAll(&q); len(got) != fit*1001 || bytes.Count(got, []byte("\n")) != fit {
		t.Fatalf("drained %d bytes, %d lines", len(got), bytes.Count(got, []byte("\n")))
	}
	if d := q.Dropped(); d != 0 {
		t.Fatalf("Dropped not reset: %d", d)
	}
}

// A stream (a dump) larger than the ring waits for the consumer and comes
// out complete; a line written meanwhile (here from another goroutine) never
// lands inside it: it is dropped and counted.
func TestStreamIsContiguousAndComplete(t *testing.T) {
	var q Ring
	payload := make([]byte, 3*size+123)
	for i := range payload {
		payload[i] = byte(i % 251)
	}
	var out []byte
	done := make(chan bool)
	go func() { // the consumer
		chunk := make([]byte, 512)
		for len(out) < len("FB\n")+len(payload)+len("\nEND\n") {
			out = append(out, chunk[:q.Drain(chunk)]...)
		}
		done <- true
	}()
	var wg sync.WaitGroup
	q.Stream()
	wg.Add(1)
	go func() { // another producer tries to write in the middle of the dump
		defer wg.Done()
		q.WriteLine([]byte("other"))
	}()
	q.Write([]byte("FB\n"))
	wg.Wait() // the other line was tried while the stream held the ring
	q.Write(payload)
	q.Write([]byte("\nEND\n"))
	q.EndStream()
	<-done
	start := bytes.Index(out, []byte("FB\n"))
	if start < 0 {
		t.Fatal("no header")
	}
	body := out[start+3 : start+3+len(payload)]
	if !bytes.Equal(body, payload) {
		t.Fatal("the dump's bytes were interleaved or lost")
	}
	if bytes.Contains(out, []byte("other")) || q.Dropped() != 1 {
		t.Fatal("the line written during the stream wasn't dropped and counted")
	}
}

// Building and queueing a line allocates nothing (log lines are written on
// the frame loop).
func TestLinesDoNotAllocate(t *testing.T) {
	var q Ring
	chunk := make([]byte, 256)
	if n := testing.AllocsPerRun(100, func() {
		var l Line
		l.S("imu a").F2(9.81).F2(-0.02).I(-12).U(7).Send(&q)
		q.Drain(chunk)
	}); n != 0 {
		t.Fatalf("%.0f allocations per line", n)
	}
}
