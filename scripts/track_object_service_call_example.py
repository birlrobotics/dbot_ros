#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from dbot_ros_msgs.srv import RunObjectTracker1
from dbot_ros_msgs.msg import ObjectState1
from dbot_ros_msgs.msg import ObjectsOri
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def track_object(object_name):
  print "Waiting for service..."
  rospy.wait_for_service("/object_tracker_service")

  try:
    run_object_tracker = rospy.ServiceProxy('/object_tracker_service', RunObjectTracker1)

    # set initial pose
    pose = Pose(position=Point(0, 0, 0.7),
                orientation=Quaternion(0, 0, 0, 0))

    pose2 = Pose(position=Point(1, 0, 0.7),
                orientation=Quaternion(0, 0, 0, 0))

    # set Object resource identifier to find the model
    ori = ObjectsOri(package = "object_meshes",
                    directory="object_models",
                    name=[x + ".obj" for x in object_name[1:]])

    # construct the ObjectState message for the service call
    object_state = ObjectState1(
            name=object_name[0],
            ori=ori,
            pose=PoseStamped(
                   pose=pose,
                   header=Header(seq=0, stamp=rospy.Time(0), frame_id='')))
            # pose2=PoseStamped(
            #        pose=pose2,
            #        header=Header(seq=0, stamp=rospy.Time(0), frame_id='')))

    print "Calling tracker service to track the %s object" % object_name[1:]
    print "names with extension:%s" % ori.name
    run_object_tracker(object_state)
  except rospy.ServiceException, e:
      print "Calling object tracker service failed: %s" % e

if __name__ == "__main__":
  track_object(sys.argv)



