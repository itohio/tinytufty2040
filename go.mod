module github.com/itohio/tinytufty2040

go 1.23.4

require (
	github.com/itohio/tinygui v0.0.0
	tinygo.org/x/drivers v0.31.0
	tinygo.org/x/tinyfont v0.6.0
)

require github.com/google/shlex v0.0.0-20191202100458-e7afc7fbc510 // indirect

// tinygui is developed next to this repo; the demo builds against the local
// checkout (no published version yet).
replace github.com/itohio/tinygui => ../tinygui
