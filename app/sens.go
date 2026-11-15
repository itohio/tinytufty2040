package app

// The motion sensitivity (owner 2026-11-14, TG-6 #6: he picked the badge up
// and the flock vanished): how far the badge moves for one full-screen pixel
// of the boids' window. In RAM; the Motion page's path keeps its own zoom.
// At 1:1 on the glass (a pixel is 0.1275 mm) any real move swept the window
// past the flock. (Pong's paddle follows pitch since PONG-PITCH-1.)

// sensMM are the choices in mm of badge motion per pixel; sensLabels their
// words on the Motion page, sensArgs the console's (`sens <mm-per-px>`).
var (
	sensMM     = [...]float32{2.5, 5, 10, 20, 40}
	sensLabels = []string{"2.5 mm", "5 mm", "10 mm", "20 mm", "40 mm"}
	sensArgs   = [len(sensMM)]string{"2.5", "5", "10", "20", "40"}
)

// sensDefault: 1 px = 10 mm, a 10 cm move is 10 px (owner).
const sensDefault = 2

// fullCellPx is a cell's size full screen, in px: the boids' box (and
// Pong's court) are 64 cells (boidsW, pongW) across the 320 px screen.
const fullCellPx = ScreenW / pongW

// cellsPerM is the cells a full-screen grid moves per metre of badge motion
// at mm per pixel.
func cellsPerM(mm float32) float32 { return 1000 / (fullCellPx * mm) }

// setSens picks choice i (sensMM) for the window.
func (a *App) setSens(i int) {
	a.sens = i
	a.applySens(sensMM[i])
}

// applySens sets mm of badge motion per pixel; the window doesn't move when
// it changes, only what follows does.
func (a *App) applySens(mm float32) {
	c := cellsPerM(mm)
	var disp [3]float32
	if a.motion != nil {
		disp = a.motion.disp
	}
	a.boids.setScale(c*boidsFix, disp[0], disp[1])
}

// sensArg turns a console argument into a choice (-1 if it is none).
func sensArg(arg string) int {
	for i, s := range sensArgs {
		if s == arg {
			return i
		}
	}
	return -1
}
