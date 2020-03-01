package sort

import (
	"fmt"
	"math"

	"github.com/flaviostutz/munkres"
)

//SORT Detection tracking
type SORT struct {
	maxAge     int
	minHits    int
	Trackers   []KalmanBoxTracker
	FrameCount int
}

//NewSort initializes a new SORT tracking session
func NewSORT(maxAge int, minHits int) SORT {
	return SORT{
		maxAge:     maxAge,
		minHits:    minHits,
		Trackers:   make([]KalmanBoxTracker, 0),
		FrameCount: 0,
	}
}

//Update update trackers from detections
//     Params:
//       dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
//     Requires: this method must be called once for each frame even with empty detections.
//     Returns the a similar array, where the last column is the object ID.
//     NOTE: The number of objects returned may differ from the number of detections provided.
func (s SORT) Update(dets [][]float64, iouThreshold float64) error {
	s.FrameCount = s.FrameCount + 1

	//NOT SURE HOW KALMAN ALGO WILL SHOW ERRORS. SEE LATER AND REMOVE INVALID PREDICTORS
	trks := make([]KalmanBoxTracker, 0)
	for _, v := range s.Trackers {
		trks = append(trks, v)
	}
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

	matched, unmatchedDets, unmatchedTrks := associateDetectionsToTrackers(dets, trks, iouThreshold)

	// update matched trackers with assigned detections
	for t, tracker := range s.Trackers {
		//is this tracker still matched?
		if !contains(unmatchedTrks, int64(t)) {
			for _, det := range matched {
				if det[1] == int64(t) {
					err := tracker.Update(dets[det[0]])
					if err != nil {
						return err
					}
					break
				}
			}
			// d = matched[np.where(matched[:,1]==t)[0],0]
			// trk.update(dets[d,:][0])
		}
	}

	// create and initialise new trackers for unmatched detections
	for _, udet := range unmatchedDets {
		trk, err := NewKalmanBoxTracker(dets[udet])
		if err != nil {
			return err
		}
		s.Trackers = append(s.Trackers, trk)
		fmt.Printf("New tracker added. bbox=%v\n", trk.bbox)
	}

	//remove dead trackers
	ti := len(s.Trackers)
	for t := ti - 1; t >= 0; t-- {
		trk := s.Trackers[t]
		//         if((trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits)):
		//           ret.append(np.concatenate((d,[trk.id+1])).reshape(1,-1)) # +1 as MOT benchmark requires positive
		if trk.timeSinceUpdate > s.maxAge {
			s.Trackers = append(s.Trackers[:t], s.Trackers[t+1:]...)
		}
		fmt.Printf("Tracker removed. bbox=%v\n", trk.bbox)
	}

	return nil
}

func contains(list []int64, value int64) bool {
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
func associateDetectionsToTrackers(detections [][]float64, trackers []KalmanBoxTracker, iouThreshold float64) ([][]int64, []int64, []int64) {
	if len(trackers) == 0 {
		det := make([]int64, 0)
		for i := range detections {
			det = append(det, int64(i))
		}
		return [][]int64{}, det, []int64{}
	}

	ld := int64(len(detections))
	lt := int64(len(trackers))

	// iouMatrix := make([][]float64, ld)

	mm := munkres.NewMatrix(int64(math.Max(float64(ld), float64(lt))))

	for d := int64(0); d < ld; d++ {
		// iouMatrix[d] = make([]float64, lt)
		for t := int64(0); t < lt; t++ {
			// iouMatrix[d][t] = iou(detections[d], trackers[t].getState())
			// v := iou(detections[d], trackers[t].getState())
			v := iou(detections[d], trackers[t].Predict())
			mm.SetElement(int64(d), int64(t), v)
		}
	}

	matchedIndices := munkres.ComputeMunkresMax(mm)

	unmatchedDetections := make([]int64, 0)
	for d := int64(0); d < ld; d++ {
		found := false
		for _, v := range matchedIndices {
			if int64(d) == v.Row { //should be Col?
				found = true
				break
			}
		}
		if !found {
			unmatchedDetections = append(unmatchedDetections, d)
		}
	}

	unmatchedTrackers := make([]int64, 0)
	for t := int64(0); t < lt; t++ {
		found := false
		for _, v := range matchedIndices {
			if int64(t) == v.Col { //should be Row?
				found = true
				break
			}
		}
		if !found {
			unmatchedTrackers = append(unmatchedTrackers, t)
		}
	}

	//filter out matched with low IOU
	matches := make([][]int64, 0)
	for _, mi := range matchedIndices {
		v := mm.GetElement(mi.Row, mi.Col)
		if v < iouThreshold {
			unmatchedDetections = append(unmatchedDetections, mi.Row)
			unmatchedTrackers = append(unmatchedTrackers, mi.Col)
		} else {
			matches = append(matches, []int64{mi.Row, mi.Col})
		}
	}

	return matches, unmatchedDetections, unmatchedTrackers
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
