#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import tf
from math import pi
from moveit_commander.conversions import pose_to_list

class calibration():

    def setupCalibrationPoints(self):
	move_group = self.move_group

	self.calibrationPoints[1] = move_group.get_current_pose().pose
	self.calibrationPoints[1].position.x = 0.24898 
	self.calibrationPoints[1].position.y = -0.14185
	self.calibrationPoints[1].position.z = 0.39021 

	self.calibrationPoints[2] = move_group.get_current_pose().pose
	self.calibrationPoints[2].position.x = 0.24898
	self.calibrationPoints[2].position.y = -0.14185
	self.calibrationPoints[2].position.z = 0.39021 + 0.220

	self.calibrationPoints[3] = move_group.get_current_pose().pose
	self.calibrationPoints[3].position.x = 0.24898
	self.calibrationPoints[3].position.y = -0.14185 + 0.256
	self.calibrationPoints[3].position.z = 0.39021 + 0.220

	self.calibrationPoints[4] = move_group.get_current_pose().pose
	self.calibrationPoints[4].position.x = 0.24898 + 0.256
	self.calibrationPoints[4].position.y = -0.14185 + 0.256
	self.calibrationPoints[4].position.z = 0.39021 + 0.220

	self.calibrationPoints[5] = move_group.get_current_pose().pose
	self.calibrationPoints[5].position.x = 0.24898 + 0.256
	self.calibrationPoints[5].position.y = -0.14185
	self.calibrationPoints[5].position.z = 0.39021

	self.calibrationPoints[6] = move_group.get_current_pose().pose
	self.calibrationPoints[6].position.x = 0.24898 + 0.256
	self.calibrationPoints[6].position.y = -0.14185 + 0.256
	self.calibrationPoints[6].position.z = 0.39021

	self.calibrationPoints[7] = move_group.get_current_pose().pose
	self.calibrationPoints[7].position.x = 0.24898 + 0.256
	self.calibrationPoints[7].position.y = -0.14185 
	self.calibrationPoints[7].position.z = 0.39021 + 0.220

	self.calibrationPoints[8] = move_group.get_current_pose().pose
	self.calibrationPoints[8].position.x = 0.24898
	self.calibrationPoints[8].position.y = -0.14185 + 0.256
	self.calibrationPoints[8].position.z = 0.39021






    def sendCalibrationPoints(self):
	for i in range(1,9):
		self.go_to_pose_goal(self.calibrationPoints[i])
		rospy.loginfo(self.calibrationPoints[i])
		print("Calibration Point "  + str(i) + " saved in slicer?")
		calibration = raw_input("Y/N : ")
		print("\n")
		while calibration != "Y":
			calibration = raw_input("Y/N : ")
			print("\n")
	
	print("Calibration Points Sent")

    def go_to_pose_goal(self,point):
        move_group = self.move_group


        pose_goal = move_group.get_current_pose().pose
        pose_goal = point
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)

   

    def all_close(self,goal, actual, tolerance):

      all_equal = True
      if type(goal) is list:
        for index in range(len(goal)):
          if abs(actual[index] - goal[index]) > tolerance:
            return False

      elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

      elif type(goal) is geometry_msgs.msg.Pose:
        return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

      return True

    def calibration_run(self):
	
        rospy.init_node('calibration', anonymous=True)
        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
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
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Misc variables
        #self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        #self.planning_frame = planning_frame
        #self.eef_link = eef_link
        #self.group_names = group_names
	self.calibrationPoints = range(10)
	self.setupCalibrationPoints()
	self.sendCalibrationPoints()
        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()
		

if __name__ == '__main__':
    jsp = calibration()
    jsp.calibration_run()
