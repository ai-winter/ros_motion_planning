# Specifies which detected persons have been merged into a composite detection by the spencer_detected_person_association module.

# TODO: Do we need a composite person-specific timestamp (or even a full message header including frame ID)?
# Having a separate timestamp per person could be useful if the timestamps of the merged DetectedPersons messages vary a lot,
# and some persons are only seen by a single sensor (so averaging over all input timestamps would have a detrimental effect). 

uint64      composite_detection_id          # ID of the fused detection

float64     mean_confidence                 # mean of the confidences of the original detections
float64     max_confidence                  # maximum confidence of original detections
float64     min_confidence                  # minimum confidence of original detections


geometry_msgs/PoseWithCovariance    pose    # Merged 3D pose (position + orientation) of the detection center
                                            # check covariance to see which dimensions are actually set!
                                            # unset dimensions shall have a covariance > 9999

DetectedPerson[] original_detections        # The original detections from individual sensor-specific detectors that have been combined into a composite detection
                                            # We are copying the entire DetectedPersons messages, *with poses transformed into the target frame*, such that subscribers
                                            # do not have to subscribe to all the original DetectedPersons topics.