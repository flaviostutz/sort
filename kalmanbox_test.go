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
	for i := 0.0; i < 10; i++ {
		updateTrk(trk, 10*i, 20, w, h)
	}
	// printInternals(&trk)
	bboxEquals(trk.CurrentPrediction(), 100.0, 20.0, w, h, t)
}

func TestPrediction2(t *testing.T) {
	w := 30.0
	h := 20.0
	bbox := []float64{50, 100, w, h}
	trk, err := NewKalmanBoxTracker(bbox)
	if err != nil {
		t.Errorf("Error initializing kalman box tracker%s", err)
	}
	// printInternals(&trk)
	for i := 0.0; i < 10; i++ {
		updateTrk(trk, 50+(5*i), 20+(2*i), w, h+i)
	}
	// printInternals(&trk)
	bboxEquals(trk.CurrentPrediction(), 100.0, 40.0, w, 30, t)
}

func bboxEquals(bbox1 []float64, bbox2x, bbox2y, bbox2w, bbox2h float64, t *testing.T) {
	bbox2 := []float64{bbox2x, bbox2y, bbox2w + bbox2x, bbox2h + bbox2y}
	b1 := mat.NewVecDense(4, bbox1)
	b2 := mat.NewVecDense(4, bbox2)
	// fmt.Printf("\nbbox1=%v bbox2=%v", bbox1, bbox2)
	if !mat.EqualApprox(b1, b2, 1E-1) {
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

func updateTrk(trk KalmanBoxTracker, x, y, w, h float64) {
	trk.Update([]float64{x, y, x + w, y + h})
}
