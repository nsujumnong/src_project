#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage

ZERO_VECTOR = [0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
ARM_EXTEND = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
CURL_FINGER = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR)
    msg = appendTrajectoryPoint(msg, 3.0, ELBOW_BENT_UP)
    msg = appendTrajectoryPoint(msg, 4.0, ZERO_VECTOR)
    msg = appendTrajectoryPoint(msg, 5.0, ARM_EXTEND)

    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def sendRightHandTrajectory():
    msg = HandTrajectoryRosMessage()

    msg.robot_side = HandTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPointHand(msg, 3.0, CURL_FINGER)

    msg.unique_ig = -1
  
    rospy.loginf('publish hand trajectory')
    handTrajectoryPublisher




def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

def appendTrajectoryPointHand(hand_trajectory, time, position):
    if not hand_trajector.joint_trajectory_messages:
	hand_trajectory.hand_trajectory_messgaes = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
	point = TrajectoryPoint1DRosMessage()
	point.time = time
	point.position = pose
	point.velocity = 0
	hand_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return hand_trajectory

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
	handTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_trajectory".format(ROBOT_NAME), HandTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0 and handTrajectoryPublisher.get_num_connection() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0 and handTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
	    sendRightHandTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
