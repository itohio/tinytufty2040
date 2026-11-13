package app

import (
	"bufio"
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"testing"
)

// setupFuncs are the demo's functions reviewed as setup-only: app.New and
// what it calls (constructors, the page builders — every page is built in
// setup by Pages.BuildAll —, the dialogs, the console's handler table), and
// the board's USB setup. Any other function with a heap site fails the test:
// owner hard rule (TG-2 #12), no allocations on hot paths or inside any loop.
// Checked on the firmware build (TinyGo keeps less on the stack there than
// on the host).
var setupFuncs = map[string]bool{
	"New": true, "App.initDemos": true, "App.initLive": true, "App.initMenuDialogs": true, "App.Handlers": true,
	"App.newMenu": true, "newTile": true, "App.pageDemos": true,
	"App.newPageOpt": true, "App.caption": true,
	"App.pageDashboard": true, "App.pageGauges": true, "App.pageControls": true, "App.pagePictures": true,
	"App.pageLayouts": true, "App.pageList": true, "App.pageLog": true,
	"newLife": true, "newLifeView": true, "App.pageLife": true,
	"newFlipFluid": true, "newWater": true, "App.pageWater": true,
	"newFire": true, "newFireView": true, "App.pageFire": true,
	"newPong": true, "newPongView": true, "App.pagePong": true,
	"newBoids": true, "newBoidsView": true, "App.pageBoids": true,
	"newMotion": true, "newOrientView": true, "newMotionDemo": true, "App.pageMotion": true,
	"newCalibration": true, "App.pageCalib": true, // the Calibration page (TG-3), built once
	"newScope": true, "App.pageScope": true,
	// step-4 snapshot only: HEAD's liquid engine (the constructor newWater and
	// newFire call) and package initialisers (run once before main)
	"newLiquid": true, "<package level>": true,
	"newBars": true, "newChip": true, "newDurationLabel": true, "newField": true, "newHeading": true, "newHeadingIn": true,
	"newScrollbar": true, "newSparkline": true, "newSwatch": true, "newTabBar": true, "still": true,
	"SetUSBSerial": true,
}

// TinyGo's heap sites in the whole demo (app, firmware, board) are all in
// setup functions. Needs tinygo (skipped otherwise); EFFECT_ALLOCS_DUMP=1
// lists the sites.
func TestDemoAllocatesOnlyInSetup(t *testing.T) {
	if runtime.Compiler != "gc" {
		t.Skip("runs tinygo itself; run with go test")
	}
	tinygo, err := exec.LookPath("tinygo")
	if err != nil {
		t.Skip("tinygo not on PATH")
	}
	cmd := exec.Command(tinygo, "build", "-target=tufty2040", "-o", filepath.Join(t.TempDir(), "fw.uf2"),
		"-print-allocs=github.com/itohio/tinytufty2040", ".")
	cmd.Dir = ".."
	out, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("tinygo build: %v\n%s", err, out)
	}
	funcs := map[string][]span{}
	var bad []string
	seen := map[string]bool{}
	sc := bufio.NewScanner(strings.NewReader(string(out)))
	for sc.Scan() {
		line := sc.Text()
		i := strings.Index(line, ": object allocated on the heap")
		if i < 0 {
			continue
		}
		parts := strings.Split(line[:i], ":")
		if len(parts) < 3 {
			continue
		}
		path := strings.Join(parts[:len(parts)-2], ":")
		if !strings.Contains(path, "/tinytufty2040/") || strings.HasSuffix(path, "_test.go") {
			continue
		}
		ln, _ := strconv.Atoi(parts[len(parts)-2])
		file := path
		if _, ok := funcs[file]; !ok {
			funcs[file] = funcSpans(t, path)
		}
		fn := enclosingFunc(funcs[file], ln)
		key := filepath.Base(file) + " " + fn
		if seen[key] {
			continue
		}
		seen[key] = true
		if os.Getenv("EFFECT_ALLOCS_DUMP") != "" {
			t.Log(key)
		}
		if !setupFuncs[fn] {
			bad = append(bad, key)
		}
	}
	sort.Strings(bad)
	for _, b := range bad {
		t.Errorf("heap allocation in %s: not a reviewed setup function (owner: no allocations after setup)", b)
	}
}

type span struct {
	name       string
	start, end int
}

func funcSpans(t *testing.T, path string) []span {
	fset := token.NewFileSet()
	f, err := parser.ParseFile(fset, path, nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	var s []span
	for _, d := range f.Decls {
		fd, ok := d.(*ast.FuncDecl)
		if !ok {
			continue
		}
		name := fd.Name.Name
		if fd.Recv != nil && len(fd.Recv.List) > 0 {
			name = recv(fd.Recv.List[0].Type) + "." + name
		}
		s = append(s, span{name, fset.Position(fd.Pos()).Line, fset.Position(fd.End()).Line})
	}
	return s
}

func recv(e ast.Expr) string {
	switch x := e.(type) {
	case *ast.StarExpr:
		return recv(x.X)
	case *ast.IndexExpr:
		return recv(x.X)
	case *ast.Ident:
		return x.Name
	}
	return "?"
}

func enclosingFunc(s []span, line int) string {
	for _, f := range s {
		if line >= f.start && line <= f.end {
			return f.name
		}
	}
	return "<package level>"
}
