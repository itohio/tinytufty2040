package app

import "testing"

// Every help line fits the log line: the log cuts a longer one at
// logLineLen, and its end (motion, dump, version, help) was lost.
func TestHelpLinesFitTheLogLine(t *testing.T) {
	for _, l := range helpLines {
		if len(l) > logLineLen {
			t.Errorf("help line %d characters, the log keeps %d: %q", len(l), logLineLen, l)
		}
	}
}
