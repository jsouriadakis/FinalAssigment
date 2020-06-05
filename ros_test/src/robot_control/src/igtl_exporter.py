#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
import math
import tf
import geometry_msgs.msg
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform
from ros_igtl_bridge.msg import igtlpoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Point


def igtl_exporter():



    rospy.init_node('igtl_exporter', anonymous=True)
    pub_igtl_point_out = rospy.Publisher('IGTL_POINT_OUT',igtlpoint,queue_size=10)
    #pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)    

    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "ur5"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                   moveit_msgs.msg.DisplayTrajectory,
    #                                                   queue_size=20)

    # Misc variables
    #self.box_name = ''
   # self.robot = robot
   # self.scene = scene
   # self.move_group = move_group
   # self.display_trajectory_publisher = display_trajectory_publisher
    #self.planning_frame = planning_frame
    #self.eef_link = eef_link
    #self.group_names = group_names

    listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz
    
    
	

    
    while not rospy.is_shutdown():
        wpose = move_group.get_current_pose().pose
    	end = Point()
    	end.x = wpose.position.x*1000
    	end.y = wpose.position.y*1000
    	end.z = wpose.position.z*1000
	end_effector = igtlpoint()
	end_effector.name = "Pose"
	end_effector.pointdata = end
        pub_igtl_point_out.publish(end_effector)
	rospy.loginfo(end_effector)
        #log_str = "igtl_exporter %s" % rospy.get_time()
        #rospy.loginfo(log_str)
        #pub.publish(log_str)
        rate.sleep()
        

if __name__ == '__main__':

    igtl_exporter()


