package main

import (
	"fmt"
	"math"

	"github.com/flaviostutz/sort"
)

func main() {

	fmt.Printf("Test KalmanBoxTracker\n")
	bt, err := sort.NewKalmanBoxTracker([]float64{1, 1, 3, 3})
	if err != nil {
		panic(err)
	}

	// fmt.Printf("111 %v\n", []float64{2, 2, 4, 4})
	// z := convertBBoxToZ([]float64{2, 2, 4, 4})
	// fmt.Printf("222 %v\n", z)
	// bbox := convertZToBBox(z)
	// fmt.Printf("333 %v\n", bbox)

	bt.Update([]float64{2, 2, 4, 4})
	bt.Update([]float64{3, 3, 5, 5})
	bt.Update([]float64{4, 4, 6, 6})
	bt.Update([]float64{5, 5, 7, 7})
	bt.Update([]float64{6, 6, 8, 8})
	bt.Update([]float64{7, 7, 9, 9})
	fmt.Printf("predicted1=%v\n", bt.PredictNext())
	fmt.Printf("predicted2=%v\n", bt.PredictNext())
	fmt.Printf("predicted3=%v\n", bt.PredictNext())
	fmt.Printf("PredictsSinceUpdate=%d\n", bt.PredictsSinceUpdate)

	fmt.Printf("Test SORT\n")
	s := sort.NewSORT(5, 4, 0.3)

	fmt.Printf("111\n")
	b := [][]float64{
		[]float64{1, 1, 3, 3},
	}
	s.Update(b, 0.3)

	fmt.Printf("222\n")
	b = [][]float64{
		[]float64{2, 2, 4, 4},
		[]float64{10, 10, 30, 30},
	}
	s.Update(b, 0.3)

	fmt.Printf("333\n")
	b = [][]float64{
		[]float64{3, 3, 5, 5},
	}
	s.Update(b, 0.3)

	fmt.Printf("444\n")
	b = [][]float64{
		[]float64{20, 20, 20, 20},
		[]float64{4, 4, 6, 6},
	}
	s.Update(b, 0.3)

	fmt.Printf("555\n")
	b = [][]float64{
		[]float64{23, 23, 23, 23},
		[]float64{30, 30, 30, 30},
		[]float64{5, 5, 7, 7},
	}
	s.Update(b, 0.3)

	fmt.Printf("666\n")
	b = [][]float64{
		[]float64{6, 6, 8, 8},
		[]float64{23, 23, 23, 23},
		[]float64{40, 40, 40, 40},
	}
	s.Update(b, 0.3)

	fmt.Printf("777\n")
	b = [][]float64{
		[]float64{7, 7, 9, 9},
		[]float64{23, 23, 23, 23},
		[]float64{50, 50, 50, 50},
	}
	s.Update(b, 0.3)

	fmt.Printf("888\n")
	b = [][]float64{
		[]float64{8, 8, 10, 10},
		[]float64{24, 24, 24, 24},
		[]float64{60, 60, 60, 60},
	}
	s.Update(b, 0.3)

	fmt.Printf("999\n")
	b = [][]float64{
		[]float64{9, 9, 11, 11},
		[]float64{23, 23, 23, 23},
		[]float64{70, 70, 70, 70},
	}
	s.Update(b, 0.3)

}

// go mod edit -replace github.com/go-chi/chi=./packages/chi

func convertBBoxToZ(bbox []float64) []float64 {
	w := bbox[2] - bbox[0]
	h := bbox[3] - bbox[1]
	x := bbox[0] + w/2.
	y := bbox[1] + h/2.
	s := w * h
	r := w / float64(h)
	return []float64{x, y, s, r}
}

func convertZToBBox(x []float64) []float64 {
	w := math.Sqrt(x[2] * x[3])
	h := x[2] / w
	return []float64{x[0] - w/2., x[1] - h/2., x[0] + w/2., x[1] + h/2.}
}
