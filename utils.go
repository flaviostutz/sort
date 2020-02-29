package sort

import (
	"github.com/pa-m/numgo"
)

//Computes IUO between two bboxes in the form [x1,y1,x2,y2]
func iou(bbtest, bbgt []float64) {
	np := numgo.New()
	xx1 := np.Maximum(bbtest[0], bbgt[0])
	yy1 := np.Maximum(bbtest[1], bbgt[1])
	xx2 := np.Maximum(bbtest[2], bbgt[2])
	yy2 := np.Maximum(bbtest[3], bbgt[3])
	w := np.Maximum(0., np.Sub(xx2, xx1)) //verify
	h := np.Maximum(0., np.Sub(yy2, yy1))
	wh := np.Multiply(w, h)
	o := np.Divide(wh / ((bbtest[2]-bbtest[0])*(bbtest[3]-bbtest[1]) + (bbgt[2]-bbgt[0])*(bbgt[3]-bbgt[1]) - wh)))
	return o
}

func fl(d ...float64) []float64 {
	return d
}

//   Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
//     [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
//     the aspect ratio
func convertBBoxToZ(bbox float[]) {
	w = bbox[2]-bbox[0]
	h = bbox[3]-bbox[1]
	x = bbox[0]+w/2.
	y = bbox[1]+h/2.
	s = w*h    #scale is just area
	r = w/float(h)
	return np.Array([x,y,s,r]).reshape((4,1))
}parei aqui dormindo...
// def convert_bbox_to_z(bbox):
//   """
//   Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
//     [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
//     the aspect ratio
//   """
//   w = bbox[2]-bbox[0]
//   h = bbox[3]-bbox[1]
//   x = bbox[0]+w/2.
//   y = bbox[1]+h/2.
//   s = w*h    #scale is just area
//   r = w/float(h)
//   return np.array([x,y,s,r]).reshape((4,1))


// def iou(bb_test,bb_gt):
//   """
//   Computes IUO between two bboxes in the form [x1,y1,x2,y2]
//   """
//   xx1 = np.maximum(bb_test[0], bb_gt[0])
//   yy1 = np.maximum(bb_test[1], bb_gt[1])
//   xx2 = np.minimum(bb_test[2], bb_gt[2])
//   yy2 = np.minimum(bb_test[3], bb_gt[3])
//   w = np.maximum(0., xx2 - xx1)
//   h = np.maximum(0., yy2 - yy1)
//   wh = w * h
//   o = wh / ((bb_test[2]-bb_test[0])*(bb_test[3]-bb_test[1])
//     + (bb_gt[2]-bb_gt[0])*(bb_gt[3]-bb_gt[1]) - wh)
//   return(o)

// def convert_bbox_to_z(bbox):
//   """
//   Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
//     [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
//     the aspect ratio
//   """
//   w = bbox[2]-bbox[0]
//   h = bbox[3]-bbox[1]
//   x = bbox[0]+w/2.
//   y = bbox[1]+h/2.
//   s = w*h    #scale is just area
//   r = w/float(h)
//   return np.array([x,y,s,r]).reshape((4,1))

// def convert_x_to_bbox(x,score=None):
//   """
//   Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
//     [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
//   """
//   w = np.sqrt(x[2]*x[3])
//   h = x[2]/w
//   if(score==None):
//     return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.]).reshape((1,4))
//   else:
//     return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.,score]).reshape((1,5))
