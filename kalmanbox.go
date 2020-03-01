package sort

import (
	"github.com/konimarti/kalman"
	"github.com/konimarti/lti"
	"gonum.org/v1/gonum/mat"
)

//KalmanBoxTracker   This class represents the internel state of individual tracked objects observed as bbox.
type KalmanBoxTracker struct {
	count           int
	kf              kalman.Filter
	ctrl            *mat.VecDense
	kctx            kalman.Context
	timeSinceUpdate int
	id              int64
	// history         [][]float64
	bbox      []float64
	hits      int
	hitStreak int
	age       int
}

//NewKalmanBoxTracker     Initialises a tracker using initial bounding box.
func NewKalmanBoxTracker(bbox []float64) KalmanBoxTracker {
	//define constant velocity model
	kf := kalman.NewFilter(
		lti.Discrete{
			Ad: mat.NewDense(7, 7, []float64{
				1, 0, 0, 0, 1, 0, 0,
				0, 1, 0, 0, 0, 1, 0,
				0, 0, 1, 0, 0, 0, 1,
				0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 0, 1}),
			C: mat.NewDense(4, 7, []float64{
				1, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0, 0}),
		},
		kalman.Noise{
			Q: mat.NewDense(7, 7, []float64{
				1, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0.01, 0, 0,
				0, 0, 0, 0, 0, 0.01, 0,
				0, 0, 0, 0, 0, 0, 0.0001}),
			R: mat.NewDense(7, 7, []float64{
				1, 0, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0, 0,
				0, 0, 0, 10, 0, 0, 0,
				0, 0, 0, 0, 10, 0, 0,
				0, 0, 0, 0, 0, 10, 0,
				0, 0, 0, 0, 0, 0, 10}),
		},
	)

	kctx := kalman.Context{
		X: mat.NewVecDense(7, []float64{0, 0, 0, 0, 0, 0, 0}),
		P: mat.NewDense(7, 7, []float64{
			10, 0, 0, 0, 1, 0, 0,
			0, 10, 0, 0, 0, 1, 0,
			0, 0, 10, 0, 0, 0, 1,
			0, 0, 0, 10, 0, 0, 0,
			0, 0, 0, 0, 1000, 0, 0,
			0, 0, 0, 0, 0, 10, 0,
			0, 0, 0, 0, 0, 0, 10}),
	}
	// self.M = np.zeros((dim_z, dim_z)) # process-measurement cross correlation
	// self.K = np.zeros((dim_x, dim_z)) # kalman gain
	// self.S = np.zeros((dim_z, dim_z)) # system uncertainty
	// self.SI = np.zeros((dim_z, dim_z)) # inverse system uncertainty

	ctrl := mat.NewVecDense(4, nil)

	z := mat.NewVecDense(4, convertBBoxToZ(bbox))
	kf.Apply(&kctx, z, ctrl)

	return KalmanBoxTracker{
		id:              0,
		count:           1,
		kf:              kf,
		ctrl:            ctrl,
		kctx:            kctx,
		timeSinceUpdate: 0,
		// history:         [][]float64{},
		hits:      0,
		hitStreak: 0,
		age:       0,
		bbox:      bbox,
	}
}

//     Updates the state vector with observed bbox.
func (k KalmanBoxTracker) update(bbox []float64) {
	k.timeSinceUpdate = 0
	// k.history = [][]float64{}
	k.hits = k.hits + 1
	k.hitStreak = k.hitStreak + 1
	k.bbox = bbox

	z := mat.NewVecDense(4, convertBBoxToZ(bbox))
	k.kf.Apply(&k.kctx, z, k.ctrl)
}

//     Advances the state vector and returns the predicted bounding box estimate.
func (k KalmanBoxTracker) predict() []float64 {
	x := k.kctx.X
	if x.AtVec(6)+x.AtVec(2) <= 0 {
		x.SetVec(6, 0.0)
	}
	k.age = k.age + 1
	if k.timeSinceUpdate > 0 {
		k.hitStreak = 0
	}
	k.timeSinceUpdate = k.timeSinceUpdate + 1

	// predBBox := k.getState()
	// k.history = append(k.history, bbox)

	state := k.kf.State()
	z := []float64{state.AtVec(0), state.AtVec(1), state.AtVec(2), state.AtVec(3)}
	return convertZToBBox(z)
}

//     Returns the current bounding box estimate.
// func (k KalmanBoxTracker) getState() []float64 {
// 	state := k.kf.State()
// 	z := []float64{state.AtVec(0), state.AtVec(1), state.AtVec(2), state.AtVec(3)}
// 	return convertZToBBox(z)
// }

// filter := kalman.NewFilter(
// 	X, // initial state (n x 1)
// 	P, // initial process covariance (n x n)
// 	F, // prediction matrix (n x n)
// 	B, // control matrix (n x k)
// 	Q, // process model covariance matrix (n x n)
// 	H, // measurement matrix (l x n)
// 	R, // measurement errors (l x l)
// )

// Ad - F
// Bd - B
// X - X
// P - P
// Q - Q
// C - H
// R - R

// X, // initial state (n x 1)
// P, // initial process covariance (n x n)
// Ad, // prediction matrix (n x n)
// Bd, // control matrix (n x k)
// Q, // process model covariance matrix (n x n)
// C,  // measurement matrix (l x n)
// R, // measurement errors (l x l)
// D,  // measurement matrix (l x k)

// class KalmanBoxTracker(object):
//   """
//   This class represents the internel state of individual tracked objects observed as bbox.
//   """
//   count = 0
//   def __init__(self,bbox):
//     """
//     Initialises a tracker using initial bounding box.
//     """
//     #define constant velocity model
//     self.kf = KalmanFilter(dim_x=7, dim_z=4)
//     self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
//     self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

//     self.kf.R[2:,2:] *= 10.
//     self.kf.P[4:,4:] *= 1000. #give high uncertainty to the unobservable initial velocities
//     self.kf.P *= 10.
//     self.kf.Q[-1,-1] *= 0.01
//     self.kf.Q[4:,4:] *= 0.01

//     self.kf.x[:4] = convert_bbox_to_z(bbox)
//     self.time_since_update = 0
//     self.id = KalmanBoxTracker.count
//     KalmanBoxTracker.count += 1
//     self.history = []
//     self.hits = 0
//     self.hit_streak = 0
//     self.age = 0

//   def update(self,bbox):
//     """
//     Updates the state vector with observed bbox.
//     """
//     self.time_since_update = 0
//     self.history = []
//     self.hits += 1
//     self.hit_streak += 1
//     self.kf.update(convert_bbox_to_z(bbox))

//   def predict(self):
//     """
//     Advances the state vector and returns the predicted bounding box estimate.
//     """
//     if((self.kf.x[6]+self.kf.x[2])<=0):
//       self.kf.x[6] *= 0.0
//     self.kf.predict()
//     self.age += 1
//     if(self.time_since_update>0):
//       self.hit_streak = 0
//     self.time_since_update += 1
//     self.history.append(convert_x_to_bbox(self.kf.x))
//     return self.history[-1]

//   def get_state(self):
//     """
//     Returns the current bounding box estimate.
//     """
//     return convert_x_to_bbox(self.kf.x)
