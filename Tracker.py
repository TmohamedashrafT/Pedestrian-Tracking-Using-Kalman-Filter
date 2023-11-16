import numpy as np
from Kalman_filter import Kalman_filter
from utilities import association_metric
class Tracker:
  def __init__(self):
    self.max_age = 90
    self.id      = 1
    self.tracks  = []
  def predict(self):
    self.predictions = []
    for kl in self.tracks:
      kl.predict()
      self.predictions.append(kl.X[:4])
  def update(self,detections):
    if self.tracks:
      matched, un_matched_detections_idx, un_matched_trackers_idx = association_metric(detections, self.predictions, iou_thresh = 0.3)
    else:
      ## If there is no tracking
      matched, un_matched_detections_idx, un_matched_trackers_idx = [],np.arange(len(detections)),[]
    dele = []
    # Update the assigned tracks with the new measurements and restart the missing
    for det, trk in matched:
      Obj_kl = self.tracks[trk]
      Obj_kl.update(detections[det])
      Obj_kl.missing = 1
    # increment the missing counter by one
    for trk in un_matched_trackers_idx:
      Obj_kl = self.tracks[trk]
      Obj_kl.missing +=1
      if Obj_kl.missing > self.max_age:
        dele.append(trk)
    for idx in dele:
      self.tracks.pop(idx)
    # add detections as a new trackers
    for det_idx in un_matched_detections_idx:
      track = Kalman_filter()
      track.X[:4] = detections[det_idx]
      track.track_id = self.id
      self.tracks.append(track)
      self.id += 1
    return un_matched_trackers_idx