package sort

import (
	"fmt"
	"testing"

	"gonum.org/v1/gonum/mat"
)

func TestPrediction1(t *testing.T) {
	w := 20.0
	h := 20.0
	bbox := []float64{0, 0, w, h}
	trk, err := NewKalmanBoxTracker(bbox)
	if err != nil {
		t.Errorf("Error initializing kalman box tracker%s", err)
	}
	// printInternals(&trk)
	trk.Update([]float64{10, 0, 10 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{20, 0, 20 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{30, 0, 30 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{40, 0, 40 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{50, 0, 50 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{60, 0, 60 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{70, 0, 70 + w, 0 + h})
	bboxEquals(trk.CurrentPrediction(), []float64{80, 0, 80 + w, 0 + h}, t)
}

func TestPrediction2(t *testing.T) {
	w := 20.0
	h := 20.0
	bbox := []float64{0, 0, w, h}
	trk, err := NewKalmanBoxTracker(bbox)
	if err != nil {
		t.Errorf("Error initializing kalman box tracker%s", err)
	}
	// printInternals(&trk)
	trk.Update([]float64{70, 0, 70 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{60, 0, 60 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{50, 0, 50 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{40, 0, 40 + w, 0 + h})
	// printInternals(&trk)
	trk.Update([]float64{30, 0, 30 + w, 0 + h})
	printInternals(&trk)
	trk.Update([]float64{20, 0, 20 + w, 0 + h})
	bboxEquals(trk.CurrentPrediction(), []float64{10, 0, 10 + w, 0 + h}, t)
}

func TestPrediction3(t *testing.T) {
	w := 20.0
	h := 20.0
	bbox := []float64{0, 0, w, h}
	trk, err := NewKalmanBoxTracker(bbox)
	if err != nil {
		t.Errorf("Error initializing kalman box tracker%s", err)
	}
	// printInternals(&trk)
	trk.Update([]float64{70, 10, 70 + w, 10 + h})
	// printInternals(&trk)
	trk.Update([]float64{60, 15, 60 + w, 15 + h})
	// printInternals(&trk)
	trk.Update([]float64{50, 20, 50 + w, 20 + h})
	// printInternals(&trk)
	trk.Update([]float64{40, 25, 40 + w, 25 + h})
	// printInternals(&trk)
	trk.Update([]float64{30, 30, 30 + w, 30 + h})
	// printInternals(&trk)
	trk.Update([]float64{20, 35, 20 + w, 35 + h})
	bboxEquals(trk.CurrentPrediction(), []float64{10, 0, 40 + w, 0 + h}, t)
}

func TestPrediction4(t *testing.T) {
	w := 20.0
	h := 20.0
	bbox := []float64{0, 0, w, h}
	trk, err := NewKalmanBoxTracker(bbox)
	if err != nil {
		t.Errorf("Error initializing kalman box tracker%s", err)
	}
	// printInternals(&trk)
	trk.Update([]float64{1000, 10, 1000 + w, 10 + h})
	// printInternals(&trk)
	trk.Update([]float64{800, 20, 800 + w, 20 + h})
	// printInternals(&trk)
	trk.Update([]float64{600, 30, 600 + w, 30 + h})
	// printInternals(&trk)
	trk.Update([]float64{400, 40, 400 + w, 40 + h})
	// printInternals(&trk)
	trk.Update([]float64{500, 50, 500 + w, 50 + h})
	printInternals(&trk)
	trk.Update([]float64{400, 60, 400 + w, 60 + h})
	bboxEquals(trk.CurrentPrediction(), []float64{300, 0, 300 + w, 70 + h}, t)
}

func bboxEquals(bbox1 []float64, bbox2 []float64, t *testing.T) {
	b1 := mat.NewVecDense(4, bbox1)
	b2 := mat.NewVecDense(4, bbox2)
	if !mat.EqualApprox(b1, b2, 1) {
		t.Errorf("bboxes don't match. bbox1=%v bbox2=%v", bbox1, bbox2)
	}
}

func printInternals(trk *KalmanBoxTracker) {
	fmt.Printf("\n\nkalman.ctrl ")
	printVecDense(trk.KalmanCtrl)
	fmt.Printf("\nkalman.ctx X ")
	printVecDense(trk.KalmanCtx.X)
	fmt.Printf("\nkalman.ctx P ")
	printDense(trk.KalmanCtx.P)
	fmt.Printf("\nkalman.currentstate ")
	printVector(trk.KalmanFilter.CurrentState())

	fmt.Printf("\ntrk.currentstate ")
	fmt.Printf("%v", trk.CurrentState())
	fmt.Printf("\ntrk.currentprediction ")
	fmt.Printf("%v", trk.CurrentPrediction())
	fmt.Printf("\ntrk.lastbbox ")
	fmt.Printf("%v", trk.LastBBox)
}

func printVector(md mat.Vector) {
	fmt.Print("Vector: ")
	for i := 0; i < md.Len(); i++ {
		fmt.Printf("%f,", md.AtVec(i))
	}
}

func printVecDense(md *mat.VecDense) {
	fmt.Print("VecDense: ")
	for i := 0; i < md.Len(); i++ {
		fmt.Printf("%f,", md.AtVec(i))
	}
}

func printDense(md *mat.Dense) {
	fmt.Print("Dense: \n")
	di, dj := md.Dims()
	for i := 0; i < di; i++ {
		for j := 0; j < dj; j++ {
			fmt.Printf("%f ", md.At(i, j))
		}
		fmt.Printf("\n")
	}
}
