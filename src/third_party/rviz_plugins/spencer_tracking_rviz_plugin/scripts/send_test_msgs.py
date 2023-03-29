#!/usr/bin/env python

import roslib; roslib.load_manifest( 'spencer_tracking_rviz_plugin' )
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson, DetectedPersons, DetectedPerson
import rospy
from math import cos, sin, tan, pi
import tf
import random
import copy

def setPoseAndTwistFromAngle( pose, twist, angle, radius ) :
    currentx = radius * cos(angle)
    currenty = radius * sin(angle)

    nextx = radius * cos(angle + angleStep)
    nexty = radius * sin(angle + angleStep)
    
    pose.pose.position.x = currentx
    pose.pose.position.y = currenty
    pose.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, angle + pi/2.0)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    pose.covariance[0 + 0 * 6] = 0.4 # x
    pose.covariance[1 + 1 * 6] = 0.2 # y
    pose.covariance[2 + 2 * 6] = 999999 # z
    pose.covariance[3 + 3 * 6] = 0.0 # x rotation
    pose.covariance[4 + 5 * 6] = 0.0 # y rotation
    pose.covariance[4 + 5 * 6] = 0.1 # z rotation

    twist.twist.linear.x = nextx - currentx
    twist.twist.linear.y = nexty - currenty
    twist.twist.linear.z = 0
    
    for i in range(0, 3):
        twist.covariance[i + i * 6] = 1.0 # linear velocity
    for i in range(3, 6):
        twist.covariance[i + i * 6] = float("inf") # rotational velocity

def createTrackAndDetection( tracks, detections, track_id, detection_id, angle, radius ) :
    trackedPerson = TrackedPerson()
    trackedPerson.track_id = track_id

    if detection_id >= 0 :
        trackedPerson.detection_id = detection_id
        trackedPerson.is_occluded = False
    else :
        trackedPerson.is_occluded = True

    trackedPerson.age = rospy.Time.now() - startTime

    setPoseAndTwistFromAngle(trackedPerson.pose, trackedPerson.twist, angle, radius)
    tracks.append(trackedPerson)

    if detection_id >= 0:
        detectedPerson = DetectedPerson()
        detectedPerson.detection_id = detection_id
        detectedPerson.confidence = random.random()

        detectedPerson.pose = copy.deepcopy(trackedPerson.pose)
        detectedPerson.pose.pose.position.x += random.random() * 0.5 - 0.25 # introduce some noise on observation position
        detectedPerson.pose.pose.position.y += random.random() * 0.5 - 0.25

        detections.append(detectedPerson)

    return

# Main code
trackTopic = '/spencer/tracked_persons'
trackPublisher = rospy.Publisher( trackTopic, TrackedPersons )

observationTopic = '/spencer/detected_persons' 
observationPublisher = rospy.Publisher( observationTopic, DetectedPersons )

rospy.init_node( 'publish_test_tracks_and_detections' )
br = tf.TransformBroadcaster()

# State variables
startTime = rospy.Time.now()
currentCycle = 0
currentAngle = 0.0
angleStep = 4.5 * pi / 180.
idShift = 0
updateRateHz = 10

# Test coordinate frame for checking if mapping into the fixed frame works correctly
frameOffset = (0, 0, 0)
frameOrientation = tf.transformations.quaternion_from_euler(0,  0,  0) # 90.0 / 180.0 * pi

print("Sending test messages on " + observationTopic + " and " + trackTopic)
rate = rospy.Rate(updateRateHz)
while not rospy.is_shutdown():
    br.sendTransform(frameOffset, frameOrientation, rospy.Time.now(), "test_tf_frame", "odom")

    trackedPersons = TrackedPersons()
    trackedPersons.header.frame_id = "/test_tf_frame"
    trackedPersons.header.stamp = rospy.Time.now()

    detectedPersons = DetectedPersons()
    detectedPersons.header = trackedPersons.header

    tracks = trackedPersons.tracks;
    detections = detectedPersons.detections;

    createTrackAndDetection(tracks, detections, idShift+0, 3, currentAngle, 2.0)
    createTrackAndDetection(tracks, detections, idShift+1, 7, currentAngle + pi / 2, 2.5)
    createTrackAndDetection(tracks, detections, idShift+2, -1, currentAngle + pi / 1, 3.0)
    createTrackAndDetection(tracks, detections, idShift+3, -1, currentAngle + pi * 1.5, cos(currentAngle) * 3.5  + 7.0)
    createTrackAndDetection(tracks, detections, idShift+4, 88, 0.0, 0.0)

    trackPublisher.publish( trackedPersons )
    observationPublisher.publish( detectedPersons )

    currentAngle += angleStep
    currentCycle += 1

    # Periodically shift the IDs to simulate tracks being removed and new tracks being added
    if(currentCycle % (updateRateHz * 15) == 0) :
        idShift += len(tracks)

    rate.sleep()