# Message specifying which original detected persons (from all kinds of sensors) have been merged into one fused detection before being processed by the person tracker, in the current time step.
#
# This information is processed by the spencer_detected_person_association module, such that other perception components can associate their results (e.g. person age, gender)
# with a particular spencer_tracking_msgs/TrackedPerson (which contains a reference to a composite person detection ID).

Header                      header          # Header timestamp is set to the *latest* timestamp of all fused DetectedPerson messages.
                                            # Before fusion, all detections are transformed into a common coordinate frame (usually base_footprint).
CompositeDetectedPerson[]   elements        # Fused (merged) detected persons