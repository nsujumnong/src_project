#!/usr/bin/env python

import copy
import time
import rospy
import numpy

from numpy import append

from sensor_msgs.msg import JointState
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

ZERO_VECTOR = [0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]

ROBOT_NAME = None

pi = numpy.pi

#Inverse kinematics function using Jacobian transpose method
chi=numpy.array([[1],[-1.5],[0]])
#print chi

def inverseKinematics(chi):
    global joints, indices
        
    print "aaa",[joints[indices[0]]]
    q_i = numpy.array([[0],[0],[0],[0]])	#Initialize with starting position
    e = chi-fwdKin(q_i)			#compute the error of desired Chi and given Chi
    alphaInv = 1				#Computational factor
    dchi_i = chi-fwdKin(q_i)		#Same as e
	
    while (numpy.linalg.norm(e) >= numpy.finfo(float).eps*30):
        q_ip1 = q_i + alphaInv*numpy.dot(numpy.transpose(Jacob(q_i)),dchi_i)
        q_i = q_ip1
        #print q_i
        dchi_i = chi-fwdKin(q_i)
        e = chi-fwdKin(q_i)
        q = q_ip1
        return  q
	
    print q
	
    return q



def fwdKin(q):
	q1=q[0]
	q2=q[1]
	q3=q[2]
	q4=q[3]
	a3 = 0.0254
	a4 = -0.2871
	d1 = -0.2499
	d3 = -0.333
	#print q1
	chi_x = (d3*numpy.cos(q1)*numpy.sin(q2))-(a3*(numpy.cos(q3)*numpy.sin(q1)))-(a3*(numpy.cos(q1)*numpy.cos(q2)*numpy.sin(q3))) + (a4*(numpy.cos(q1)*numpy.cos(q4)*numpy.sin(q2))) - (a4*(numpy.cos(q3)*numpy.sin(q1)*numpy.sin(q4))) - (a4*(numpy.cos(q1)*numpy.cos(q2)*numpy.sin(q3)*numpy.sin(q4)))
	chi_y = (a3*numpy.cos(q1)*numpy.cos(q3)) + (d3*numpy.sin(q1)*numpy.sin(q2)) + (a4*numpy.cos(q1)*numpy.cos(q3)*numpy.sin(q4)) - (a3*numpy.cos(q2)*numpy.sin(q1)*numpy.sin(q3)) + (a4*numpy.cos(q4)*numpy.sin(q1)*numpy.sin(q2)) - (a4*numpy.cos(q2)*numpy.sin(q1)*numpy.sin(q3)*numpy.sin(q4))
	chi_z = d1 + (d3*numpy.cos(q2)) + (a4*numpy.cos(q2)*numpy.cos(q4)) + (a3*numpy.sin(q2)*numpy.sin(q3)) + (a4*numpy.sin(q2)*numpy.sin(q3)*numpy.sin(q4))
	chi = numpy.array([chi_x,chi_y,chi_z])
	return chi

def Jacob(q):
	q1,q2,q3,q4 = q	
	q1 = q1[0]
	q2 = q2[0]
	q3 = q3[0]
	q4 = q4[0]
	q = [q1,q2,q3,q4]
	a3 = 0.0254
	a4 = -0.2871
	d1 = -0.2499
	d3 = -0.333
	j1 = numpy.array([(a3*numpy.cos(q2)*numpy.sin(q1)*numpy.sin(q3)) - (d3*numpy.sin(q1)*numpy.sin(q2)) - (a4*numpy.cos(q1)*numpy.cos(q3)*numpy.sin(q4)) - (a3*numpy.cos(q1)*numpy.cos(q3)) - (a4*numpy.cos(q4)*numpy.sin(q1)*numpy.sin(q2)) + (a4*numpy.cos(q2)*numpy.sin(q1)*numpy.sin(q3)*numpy.sin(q4)), (numpy.cos(q1)*(d3*numpy.cos(q2))) +( a4*numpy.cos(q2)*numpy.cos(q4)) + (a3*numpy.sin(q2)*numpy.sin(q3)) + (a4*numpy.sin(q2)*numpy.sin(q3)*numpy.sin(q4)),  ((numpy.sin(q1)*numpy.sin(q3)) - (numpy.cos(q1)*numpy.cos(q2)*numpy.cos(q3)))*(a3 + (a4*numpy.sin(q4))), (-a4*(numpy.cos(q3)*numpy.cos(q4)*numpy.sin(q1)) + (numpy.cos(q1)*numpy.sin(q2)*numpy.sin(q4)) + (numpy.cos(q1)*numpy.cos(q2)*numpy.cos(q4)*numpy.sin(q3)))]) 

	j2 = numpy.array([(d3*numpy.cos(q1)*numpy.sin(q2)) - (a3*numpy.cos(q3)*numpy.sin(q1)) - (a3*numpy.cos(q1)*numpy.cos(q2)*numpy.sin(q3)) + (a4*numpy.cos(q1)*numpy.cos(q4)*numpy.sin(q2)) - (a4*numpy.cos(q3)*numpy.sin(q1)*numpy.sin(q4)) - (a4*numpy.cos(q1)*numpy.cos(q2)*numpy.sin(q3)*numpy.sin(q4)), (numpy.sin(q1)*(d3*numpy.cos(q2)) + (a4*numpy.cos(q2)*numpy.cos(q4)) + (a3*numpy.sin(q2)*numpy.sin(q3)) + (a4*numpy.sin(q2)*numpy.sin(q3)*numpy.sin(q4))), -((numpy.cos(q1)*numpy.sin(q3)) + (numpy.cos(q2)*numpy.cos(q3)*numpy.sin(q1)))*(a3 + (a4*numpy.sin(q4))), (-a4*(numpy.sin(q1)*numpy.sin(q2)*numpy.sin(q4)) - (numpy.cos(q1)*numpy.cos(q3)*numpy.cos(q4)) + (numpy.cos(q2)*numpy.cos(q4)*numpy.sin(q1)*numpy.sin(q3)))])

	j3 = numpy.array([0,(a3*numpy.cos(q2)*numpy.sin(q3)) - (d3*numpy.sin(q2)) - (a4*numpy.cos(q4)*numpy.sin(q2)) + (a4*numpy.cos(q2)*numpy.sin(q3)*numpy.sin(q4)),(numpy.cos(q3)*numpy.sin(q2))*(a3 + (a4*numpy.sin(q4))),(a4*numpy.cos(q4)*numpy.sin(q2)*numpy.sin(q3)) - (a4*numpy.cos(q2)*numpy.sin(q4))])


	j = numpy.array([j1,j2,j3]) 
 
	return j


def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    test = inverseKinematics(chi)
    test1,test2,test3,test4 = test
    test1 = test1[0]
    test2 = test2[0]
    test3 = test3[0]
    test4 = test4[0]
    test = [test1,test2,test3,test4]

    print test

    ZERO_VECTOR = [test[0], test[1], test[2], test[3], 0 ,0 ,0]
  
    msg = appendTrajectoryPoint(msg, 4.0, ZERO_VECTOR)

    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

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


def jointCallback(states):
    global indices, joints
    joints = numpy.array(states.position)
    indices = [22,23,24,25]
    #print list(joints[indices])

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
        jointStateSub = rospy.Subscriber("/hardware_joint_states", JointState,  jointCallback)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
