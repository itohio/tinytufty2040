package app

import "time"

// frameProfile sums where the frames' time went since the last stats
// report (TG-5 #4: measurement only). The period runs from one frame's start
// to the next, so other = period − imu − update − draw − push is what the
// loop did between frames: buttons, the sleep, the scheduler, the console.
// The IMU drain splits into its I2C reads (bus) and the motion maths.
type frameProfile struct {
	n, samples                           int
	period, imu, bus, update, draw, push int64 // µs, summed
}

// ReportProfile adds one frame: its period (start to start), the IMU drain
// with its time on the I2C bus and the samples it fed, Update (the demos'
// steps), and the bands' draw and push.
func (a *App) ReportProfile(period, imu, bus time.Duration, samples int, update, draw, push time.Duration) {
	p := &a.prof
	p.n++
	p.samples += samples
	p.period += period.Microseconds()
	p.imu += imu.Microseconds()
	p.bus += bus.Microseconds()
	p.update += update.Microseconds()
	p.draw += draw.Microseconds()
	p.push += push.Microseconds()
}

// profileReport logs the averages per frame since the last report and starts
// a new window: read it after a few seconds on one page for that page's split.
func (a *App) profileReport() {
	p := &a.prof
	if p.n == 0 {
		return
	}
	n := int64(p.n)
	other := p.period - p.imu - p.update - p.draw - p.push
	// three lines: the log truncates at logLineLen (56)
	a.log(a.txtStat.reset().str("prof period_us ").int(int(p.period / n)).str(" imu_us ").int(int(p.imu / n)).
		str(" update_us ").int(int(p.update / n)).String())
	a.log(a.txtStat.reset().str("prof draw_us ").int(int(p.draw / n)).str(" push_us ").int(int(p.push / n)).
		str(" other_us ").int(int(other / n)).str(" n ").int(p.n).String())
	// per IMU sample: the I2C bus, then the motion maths (the drain's rest)
	if p.samples > 0 {
		s := int64(p.samples)
		a.log(a.txtStat.reset().str("prof imu samples ").int(p.samples).str(" bus_us ").int(int(p.bus / s)).
			str(" maths_us ").int(int((p.imu - p.bus) / s)).String())
	}
	*p = frameProfile{}
}
