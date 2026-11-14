module github.com/itohio/tinytufty2040

go 1.25

require (
	github.com/itohio/tinygui v0.0.0
	tinygo.org/x/drivers v0.31.0
	tinygo.org/x/tinyfont v0.6.0
)

require (
	github.com/chewxy/math32 v1.11.1 // indirect
	github.com/google/shlex v0.0.0-20191202100458-e7afc7fbc510 // indirect
	golang.org/x/sys v0.37.0 // indirect
)

// tinygui is developed next to this repo; the demo builds against the local
// checkout (no published version yet).
replace github.com/itohio/tinygui => ../tinygui

require github.com/itohio/EasyRobot v0.0.0

// EasyRobot (MPU6050 driver, Mahony, strapdown, flip) is developed next to
// this repo too; the demo builds against the local checkout. Built and
// verified against EasyRobot e54c076 (flip's ER-061 commits: divide-free
// division, solve, separation, blend, toGrid; the same liquid).
replace github.com/itohio/EasyRobot => ../EasyRobot
