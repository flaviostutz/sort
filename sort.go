package sort

import (
	"fmt"
	"math"

	"github.com/sirupsen/logrus"

	"github.com/cpmech/gosl/graph"
)

//SORT Detection tracking
type SORT struct {
	maxPredictsWithoutUpdate int
	minUpdatesUsePrediction  int
	iouThreshold             float64
	Trackers                 []*KalmanBoxTracker
	FrameCount               int
}

//NewSORT initializes a new SORT tracking session
func NewSORT(maxPredictsWithoutUpdate int, minUpdatesUsePrediction int, iouThreshold float64) SORT {
	return SORT{
		maxPredictsWithoutUpdate: maxPredictsWithoutUpdate,
		minUpdatesUsePrediction:  minUpdatesUsePrediction,
		iouThreshold:             iouThreshold,
		Trackers:                 make([]*KalmanBoxTracker, 0),
		FrameCount:               0,
	}
}

//Update update trackers from detections
//     Params:
//       dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
//     Requires: this method must be called once for each frame even with empty detections.
//     Returns the a similar array, where the last column is the object ID.
//     NOTE: The number of objects returned may differ from the number of detections provided.
func (s *SORT) Update(dets [][]float64) error {
	logrus.Debugf("SORT Update dets=%v iouThreshold=%f", dets, s.iouThreshold)
	s.FrameCount = s.FrameCount + 1

	//NOT SURE HOW KALMAN ALGO WILL SHOW ERRORS. SEE LATER AND REMOVE INVALID PREDICTORS
	// trks := make([]KalmanBoxTracker, 0)
	// for _, v := range s.Trackers {
	// 	trks = append(trks, v)
	// }
	// get predicted locations from existing trackers.
	//     trks = np.zeros((len(self.trackers),5))
	//     to_del = []
	//     ret = []
	//     for t,trk in enumerate(trks):
	//       pos = self.trackers[t].predict()[0]
	//       trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
	//       if(np.any(np.isnan(pos))):
	//         to_del.append(t)
	//     trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
	//     for t in reversed(to_del):
	//       self.trackers.pop(t)

	matched, unmatchedDets, unmatchedTrks := associateDetectionsToTrackers(dets, s.Trackers, s.iouThreshold, s.minUpdatesUsePrediction)

	logrus.Debugf("Detection X Trackers. matched=%v unmatchedDets=%v unmatchedTrks=%v", matched, unmatchedDets, unmatchedTrks)

	// update matched trackers with assigned detections
	for t := 0; t < len(s.Trackers); t++ {
		tracker := s.Trackers[t]
		//is this tracker still matched?
		if !contains(unmatchedTrks, t) {
			for _, det := range matched {
				if det[1] == t {
					bbox := dets[det[0]]
					err := tracker.Update(bbox)
					if err != nil {
						return err
					}
					logrus.Debugf("Tracker updated. id=%d bbox=%v updates=%d\n", tracker.ID, bbox, tracker.Updates)
					break
				}
			}
			// d = matched[np.where(matched[:,1]==t)[0],0]
			// trk.update(dets[d,:][0])
		}
	}

	// create and initialise new trackers for unmatched detections
	for _, udet := range unmatchedDets {

		aread := area(dets[udet])
		if aread < 1 {
			logrus.Debugf("Ignoring too small detection. bbox=%f area=%f", dets[udet], aread)
			continue
		}

		trk, err := NewKalmanBoxTracker(dets[udet])
		if err != nil {
			return err
		}
		s.Trackers = append(s.Trackers, &trk)
		logrus.Debugf("New tracker added. id=%d bbox=%v\n", trk.ID, trk.LastBBox)
	}

	//remove dead trackers
	ti := len(s.Trackers)
	for t := ti - 1; t >= 0; t-- {
		trk := s.Trackers[t]
		//         if((trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits)):
		//           ret.append(np.concatenate((d,[trk.id+1])).reshape(1,-1)) # +1 as MOT benchmark requires positive
		if trk.PredictsSinceUpdate > s.maxPredictsWithoutUpdate {
			s.Trackers = append(s.Trackers[:t], s.Trackers[t+1:]...)
			logrus.Debugf("Tracker removed. id=%d, bbox=%v updates=%d\n", trk.ID, trk.LastBBox, trk.Updates)
		}
	}

	ct := ""
	for _, v := range s.Trackers {
		ct = ct + fmt.Sprintf("[id=%d bbox=%v updates=%d] ", v.ID, v.LastBBox, v.Updates)
	}
	logrus.Debugf("Current trackers=%s", ct)

	return nil
}

func contains(list []int, value int) bool {
	found := false
	for _, v := range list {
		if v == value {
			found = true
			break
		}
	}
	return found
}

//   Assigns detections to tracked object (both represented as bounding boxes)
//   Returns 3 lists of indexes: matches, unmatched_detections and unmatched_trackers
func associateDetectionsToTrackers(detections [][]float64, trackers []*KalmanBoxTracker, iouThreshold float64, minUpdatesUsePrediction int) ([][]int, []int, []int) {
	if len(trackers) == 0 {
		det := make([]int, 0)
		for i := range detections {
			det = append(det, i)
		}
		return [][]int{}, det, []int{}
	}

	ld := len(detections)
	lt := len(trackers)

	// iouMatrix := make([][]float64, ld)

	mk := graph.Munkres{}
	mk.Init(int(ld), int(lt))

	// mm := munkres.NewMatrix(ld, lt)
	//initialize IOUS cost matrix
	ious := make([][]float64, ld)
	for i := 0; i < len(ious); i++ {
		ious[i] = make([]float64, lt)
	}

	predicted := make([]bool, lt)
	for d := 0; d < ld; d++ {
		// iouMatrix[d] = make([]float64, lt)
		for t := 0; t < lt; t++ {
			trk := trackers[t]

			//use simple last bbox if not enough updates in this tracker
			tbbox := trk.LastBBox

			//use prediction
			if trk.Updates >= minUpdatesUsePrediction {
				//in this frame request, predict just once
				if !predicted[t] {
					tbbox = trk.PredictNext()
					predicted[t] = true
				} else {
					tbbox = trk.CurrentPrediction()
				}
			}

			v := iou(detections[d], tbbox)
			// if v > 0 {
			logrus.Debugf("IOU=%v detbbox=%v trackerrefbbox=%v trackerid=%d lastbbox=%v", v, detections[d], tbbox, trackers[t].ID, trackers[t].LastBBox)
			// }
			//invert cost matrix (we want max cost here)
			ious[d][t] = 1 - v
		}
	}

	//calculate best DETECTION vs TRACKER matches according to COST matrix
	mk.SetCostMatrix(ious)
	mk.Run()
	matchedIndices := [][]int{}
	for i, j := range mk.Links {
		if j != -1 {
			matchedIndices = append(matchedIndices, []int{i, j})
		}
	}

	logrus.Debugf("Detection x Tracker match=%v", matchedIndices)

	unmatchedDetections := make([]int, 0)
	for d := 0; d < ld; d++ {
		found := false
		for _, v := range matchedIndices {
			if d == v[0] {
				found = true
				break
			}
		}
		if !found {
			logrus.Debugf("Unmatched detection found. bbox=%v", detections[d])
			unmatchedDetections = append(unmatchedDetections, d)
		}
	}

	unmatchedTrackers := make([]int, 0)
	for t := 0; t < lt; t++ {
		found := false
		for _, v := range matchedIndices {
			if t == v[1] {
				found = true
				break
			}
		}
		if !found {
			unmatchedTrackers = append(unmatchedTrackers, t)
		}
	}

	matches := make([][]int, 0)
	for _, mi := range matchedIndices {
		//filter out matched with low IOU
		iou := 1 - ious[mi[0]][mi[1]]
		if iou < iouThreshold {
			logrus.Debugf("Skipping detection/tracker because it has low IOU deti=%d trki=%d iou=%f", mi[0], mi[1], iou)
			unmatchedDetections = append(unmatchedDetections, mi[0])
			unmatchedTrackers = append(unmatchedTrackers, mi[1])
		} else {
			matches = append(matches, []int{mi[0], mi[1]})
		}
	}

	return matches, unmatchedDetections, unmatchedTrackers
}

func area(bbox []float64) float64 {
	a := bbox[2] - bbox[0]
	b := bbox[3] - bbox[1]
	return math.Abs(a * b)
}

// def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.3):
//   """
//   Assigns detections to tracked object (both represented as bounding boxes)
//   Returns 3 lists of matches, unmatched_detections and unmatched_trackers
//   """
//   if(len(trackers)==0):
//     return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
//   iou_matrix = np.zeros((len(detections),len(trackers)),dtype=np.float32)

//   for d,det in enumerate(detections):
//     for t,trk in enumerate(trackers):
//       iou_matrix[d,t] = iou(det,trk)
//   matched_indices = linear_assignment(-iou_matrix)

//   unmatched_detections = []
//   for d,det in enumerate(detections):
//     if(d not in matched_indices[:,0]):
//       unmatched_detections.append(d)
//   unmatched_trackers = []
//   for t,trk in enumerate(trackers):
//     if(t not in matched_indices[:,1]):
//       unmatched_trackers.append(t)

//   #filter out matched with low IOU
//   matches = []
//   for m in matched_indices:
//     if(iou_matrix[m[0],m[1]]<iou_threshold):
//       unmatched_detections.append(m[0])
//       unmatched_trackers.append(m[1])
//     else:
//       matches.append(m.reshape(1,2))
//   if(len(matches)==0):
//     matches = np.empty((0,2),dtype=int)
//   else:
//     matches = np.concatenate(matches,axis=0)

//   return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

// class Sort(object):
//   def __init__(self,max_age=1,min_hits=3):
//     """
//     Sets key parameters for SORT
//     """
//     self.max_age = max_age
//     self.min_hits = min_hits
//     self.trackers = []
//     self.frame_count = 0

//   def update(self,dets):
//     """
//     Params:
//       dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
//     Requires: this method must be called once for each frame even with empty detections.
//     Returns the a similar array, where the last column is the object ID.
//     NOTE: The number of objects returned may differ from the number of detections provided.
//     """
//     self.frame_count += 1
//     #get predicted locations from existing trackers.
//     trks = np.zeros((len(self.trackers),5))
//     to_del = []
//     ret = []
//     for t,trk in enumerate(trks):
//       pos = self.trackers[t].predict()[0]
//       trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
//       if(np.any(np.isnan(pos))):
//         to_del.append(t)
//     trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
//     for t in reversed(to_del):
//       self.trackers.pop(t)
//     matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks)

//     #update matched trackers with assigned detections
//     for t,trk in enumerate(self.trackers):
//       if(t not in unmatched_trks):
//         d = matched[np.where(matched[:,1]==t)[0],0]
//         trk.update(dets[d,:][0])

//     #create and initialise new trackers for unmatched detections
//     for i in unmatched_dets:
//         trk = KalmanBoxTracker(dets[i,:])
//         self.trackers.append(trk)
//     i = len(self.trackers)
//     for trk in reversed(self.trackers):
//         d = trk.get_state()[0]
//         if((trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits)):
//           ret.append(np.concatenate((d,[trk.id+1])).reshape(1,-1)) # +1 as MOT benchmark requires positive
//         i -= 1
//         #remove dead tracklet
//         if(trk.time_since_update > self.max_age):
//           self.trackers.pop(i)
//     if(len(ret)>0):
//       return np.concatenate(ret)
//     return np.empty((0,5))

// def parse_args():
//     """Parse input arguments."""
//     parser = argparse.ArgumentParser(description='SORT demo')
//     parser.add_argument('--display', dest='display', help='Display online tracker output (slow) [False]',action='store_true')
//     args = parser.parse_args()
//     return args

// if __name__ == '__main__':
//   # all train
//   sequences = ['PETS09-S2L1','TUD-Campus','TUD-Stadtmitte','ETH-Bahnhof','ETH-Sunnyday','ETH-Pedcross2','KITTI-13','KITTI-17','ADL-Rundle-6','ADL-Rundle-8','Venice-2']
//   args = parse_args()
//   display = args.display
//   phase = 'train'
//   total_time = 0.0
//   total_frames = 0
//   colours = np.random.rand(32,3) #used only for display
//   if(display):
//     if not os.path.exists('mot_benchmark'):
//       print('\n\tERROR: mot_benchmark link not found!\n\n    Create a symbolic link to the MOT benchmark\n    (https://motchallenge.net/data/2D_MOT_2015/#download). E.g.:\n\n    $ ln -s /path/to/MOT2015_challenge/2DMOT2015 mot_benchmark\n\n')
//       exit()
//     plt.ion()
//     fig = plt.figure()

//   if not os.path.exists('output'):
//     os.makedirs('output')

//   for seq in sequences:
//     mot_tracker = Sort() #create instance of the SORT tracker
//     seq_dets = np.loadtxt('data/%s/det.txt'%(seq),delimiter=',') #load detections
//     with open('output/%s.txt'%(seq),'w') as out_file:
//       print("Processing %s."%(seq))
//       for frame in range(int(seq_dets[:,0].max())):
//         frame += 1 #detection and frame numbers begin at 1
//         dets = seq_dets[seq_dets[:,0]==frame,2:7]
//         dets[:,2:4] += dets[:,0:2] #convert to [x1,y1,w,h] to [x1,y1,x2,y2]
//         total_frames += 1

//         if(display):
//           ax1 = fig.add_subplot(111, aspect='equal')
//           fn = 'mot_benchmark/%s/%s/img1/%06d.jpg'%(phase,seq,frame)
//           im =io.imread(fn)
//           ax1.imshow(im)
//           plt.title(seq+' Tracked Targets')

//         start_time = time.time()
//         trackers = mot_tracker.update(dets)
//         cycle_time = time.time() - start_time
//         total_time += cycle_time

//         for d in trackers:
//           print('%d,%d,%.2f,%.2f,%.2f,%.2f,1,-1,-1,-1'%(frame,d[4],d[0],d[1],d[2]-d[0],d[3]-d[1]),file=out_file)
//           if(display):
//             d = d.astype(np.int32)
//             ax1.add_patch(patches.Rectangle((d[0],d[1]),d[2]-d[0],d[3]-d[1],fill=False,lw=3,ec=colours[d[4]%32,:]))
//             ax1.set_adjustable('box-forced')

//         if(display):
//           fig.canvas.flush_events()
//           plt.draw()
//           ax1.cla()

//   print("Total Tracking took: %.3f for %d frames or %.1f FPS"%(total_time,total_frames,total_frames/total_time))
//   if(display):
//     print("Note: to get real runtime results run without the option: --display")
