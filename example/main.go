package main

import (
	"fmt"
	"math"

	"github.com/flaviostutz/sort"
	"github.com/sirupsen/logrus"
)

func main() {

	fmt.Printf("Test KalmanBoxTracker\n")
	bt, err := sort.NewKalmanBoxTracker([]float64{1, 1, 3, 3})
	if err != nil {
		panic(err)
	}

	logrus.SetLevel(logrus.DebugLevel)

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
	s := sort.NewSORT(2, 4, 0.3)

	fmt.Printf("\n\n11111111111\n")
	b := [][]float64{
		[]float64{1, 1, 4, 4},
		[]float64{100, 100, 120, 120},
	}
	s.Update(b)

	fmt.Printf("\n\n22222222222222\n")
	b = [][]float64{
		[]float64{110, 110, 130, 130},
		[]float64{2, 2, 5, 5},
		[]float64{10, 10, 30, 30},
	}
	s.Update(b)

	fmt.Printf("\n\n333333333333333\n")
	b = [][]float64{
		[]float64{3, 3, 6, 6},
		[]float64{120, 120, 140, 140},
	}
	s.Update(b)

	fmt.Printf("\n\n4444444444444444\n")
	b = [][]float64{
		[]float64{130, 130, 150, 150},
		[]float64{4, 4, 7, 7},
	}
	s.Update(b)

	fmt.Printf("\n\n55555555555555555\n")
	b = [][]float64{
		[]float64{23, 23, 23, 23},
		[]float64{30, 30, 30, 30},
		[]float64{5, 5, 8, 8},
	}
	s.Update(b)

	fmt.Printf("\n\n666666666666666\n")
	b = [][]float64{
		[]float64{6, 6, 9, 9},
		[]float64{23, 23, 23, 23},
		[]float64{40, 40, 60, 60},
	}
	s.Update(b)

	fmt.Printf("\n\n7777777777777777777\n")
	b = [][]float64{
		[]float64{160, 160, 180, 180},
		[]float64{7, 7, 10, 10},
		[]float64{23, 23, 23, 23},
		[]float64{50, 50, 70, 70},
	}
	s.Update(b)

	fmt.Printf("\n\n88888888888888888\n")
	b = [][]float64{
		[]float64{8, 8, 11, 11},
		[]float64{24, 24, 24, 24},
		[]float64{170, 170, 190, 190},
		[]float64{60, 60, 80, 80},
	}
	s.Update(b)

	fmt.Printf("\n\n99999999999999999\n")
	b = [][]float64{
		[]float64{9, 9, 12, 12},
		[]float64{23, 23, 23, 23},
		[]float64{70, 70, 90, 90},
	}
	s.Update(b)

}

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
