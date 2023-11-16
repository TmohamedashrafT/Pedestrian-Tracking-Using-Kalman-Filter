import numpy as np
from scipy.optimize import linear_sum_assignment
def bb_iou(detections, tracks):
    '''
    calc ious between detections and trackers

    '''
    # Add dim to detections and tracks to make output with shape: (len(detections), len(tracks))
    detections = np.expand_dims(detections, 1)
    tracks = np.expand_dims(tracks, 0)

    detections_area = (detections[..., 2] - detections[..., 0]) * (detections[..., 3] - detections[..., 1])
    tracks_area = (tracks[..., 2] - tracks[..., 0]) * (tracks[..., 3] - tracks[..., 1])

    left_corner = np.array([np.maximum(detections[..., 0],tracks[..., 0]), np.maximum(detections[..., 1],tracks[..., 1])])
    right_corner= np.array([np.minimum(detections[..., 2],tracks[..., 2]), np.minimum(detections[..., 3],tracks[..., 3])])

    inter       = np.maximum(right_corner-left_corner,np.zeros_like(right_corner))
    inter       = inter[0] * inter[1]
    union       = detections_area+tracks_area-inter
    iou  = inter/ ( union + 1e-8 )
    return iou


def association_metric(detections, trackers, iou_thresh = 0.5):
  un_matched_detections_idx = []
  un_matched_trackers_idx   = []
  matched = []
  ious      = bb_iou(detections, trackers) ## shape : (detections, trackers)
  ## assign detections to the tracks using Hungarian algorithm
  ## The objective of the Hungarian algorithm is to minimize the cost, but in our task, we aim to maximize it, so we will multiply by -1
  all_matchs = np.array(linear_sum_assignment(- ious))
  ##  Check unmatched detections
  for id in range(len(detections)):
    if id not in all_matchs[0]:
      un_matched_detections_idx.append(id)
  ##  Check unmatched trackers
  for id in range(len(trackers)):
    if id not in all_matchs[1]:
      un_matched_trackers_idx.append(id)
  all_matchs = all_matchs.T
  ## Check if all detections are matched above the threshold
  for mat in all_matchs:
    if ious[mat[0],mat[1]] < iou_thresh:
      un_matched_detections_idx.append(mat[0])
      un_matched_trackers_idx.append(mat[1])
    else:
      matched.append(mat)
  return matched, un_matched_detections_idx, un_matched_trackers_idx
