#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import math
import tf
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Transform
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from ros_igtl_bridge.msg import igtlpoint
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform

class importer():
    def cb_transform(self,data):
        pass

    def cb_point(self,data):

        if data.name == 'Entry':
	       self.validEntry = True
               rospy.loginfo(rospy.get_caller_id() + " Entry = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)
               self.entry.position.x = data.pointdata.x/1000.0
               self.entry.position.y = data.pointdata.y/1000.0
               self.entry.position.z = data.pointdata.z/1000.0
               # self.go_to_pose_goal(data.pointdata.x,data.pointdata.y,data.pointdata.z)
        elif data.name == 'Target':
	       self.validTarget = True
               rospy.loginfo(rospy.get_caller_id() + " Target = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)
	       self.target.position.x = data.pointdata.x/1000.0
	       self.target.position.y = data.pointdata.y/1000.0
	       self.target.position.z = data.pointdata.z/1000.0
               # self.plan_cartesian_path(data.pointdata.x,data.pointdata.y,data.pointdata.z)
	if (self.validEntry and self.validTarget):
	       # if (self.validMarkers == False) :
			# self.show_marker() 			  
    
	       if (self.execute()):
			self.show_marker() 
			self.validEntry = False
			self.validTarget = False

    def show_marker(self):
	self.markerArray
	m1 = Marker()
	m2 = Marker()
	m3 = Marker()
	m1.header.frame_id = "base_link"
	m2.header.frame_id = "base_link"
	m3.header.frame_id = "base_link"
	m1.header.stamp = rospy.Time.now()
	m2.header.stamp = rospy.Time.now()
	m3.header.stamp = rospy.Time.now()
	m1.type = 2
	m2.type = 2
	m3.type = m3.MESH_RESOURCE
	m3.mesh_resource = "package://robot_control/src/CortexModel.stl"
	m1.action = m1.ADD
	m2.action = m2.ADD
	m3.action = m3.ADD
	m1.ns = "Entry"
	m2.ns = "Target"
	m2.ns = "Cortex"
	m1.lifetime = rospy.Duration.from_sec(999)
	m2.lifetime = rospy.Duration.from_sec(999)
	m3.lifetime = rospy.Duration.from_sec(999)
	m1.pose.position.x = self.entry.position.x
	m2.pose.position.x = self.target.position.x
	m1.pose.position.y = self.entry.position.y
	m2.pose.position.y = self.target.position.y
	m1.pose.position.z = self.entry.position.z
	m2.pose.position.z = self.target.position.z

	m3.pose.position.x = 0.28778
	m3.pose.position.y = 0.13778
	m3.pose.position.z = 0.20744

	m1.pose.orientation.w = 1.0
	m2.pose.orientation.w = 1.0
	m1.pose.orientation.x = 0.0
	m2.pose.orientation.x = 0.0
	m1.pose.orientation.y = 0.0
	m2.pose.orientation.y = 0.0
	m1.pose.orientation.z = 0.0
	m2.pose.orientation.z = 0.0

	m3.pose.orientation.w = 1.0
	m3.pose.orientation.x = 0.0
	m3.pose.orientation.y = 0.0
	m3.pose.orientation.z = 0.0

	m1.scale.x = 0.02
	m1.scale.y = 0.02
	m1.scale.z = 0.02

	m2.scale.x = 0.02
	m2.scale.y = 0.02
	m2.scale.z = 0.02

	m3.scale.x = 0.001
	m3.scale.y = 0.001
	m3.scale.z = 0.001

	m1.color.a = 1.0
	m1.color.r = 0.0
	m1.color.g = 0.5
	m1.color.b = 0.5

	m2.color.a = 1.0
	m2.color.r = 0.0
	m2.color.g = 0.0
	m2.color.b = 1.0

	m3.color.a = 0.3
	m3.color.r = 0.0
	m3.color.g = 1.0
	m3.color.b = 0.0

	# m2.id = 0
	self.markerArray.markers.append(m1)
	self.markerArray.markers.append(m2)
	# self.pub_Marker.publish(self.markerArray)
	self.validMarkers = True
	self.pub_entryMarker.publish(m1)
	self.pub_targetMarker.publish(m2)
	self.pub_cortexMarker.publish(m3)






    def execute(self):
        vectorET = [self.target.position.x - self.entry.position.x,self.target.position.y - self.entry.position.y,self.target.position.z - self.entry.position.z]
        vectorET = vectorET/self.magnitudeVector(vectorET)
        vectorO = [0.0,1.0,0.0]
        vectorX = np.cross(vectorO,vectorET)
        vectorY = np.cross(vectorET,vectorX)
        orientationMatrix = np.mat([[vectorX[0],vectorY[0],vectorET[0]],[vectorX[1],vectorY[1],vectorET[1]],[vectorX[2],vectorY[2],vectorET[2]]])
        w,x,y,z = self.mat2quat(orientationMatrix)
        self.go_to_pose_entry(w,x,y,z)
	asnwer = raw_input("procced to Target? Press Enter")
        self.plan_cartesian_path(w,x,y,z)
	return True

    def magnitudeVector(self,vector):
        magnitude = np.sqrt(vector[0]**2+vector[1]**2+vector[2]**2)
        return magnitude

    def mat2quat(self,t):
        #   Adapted from "robotic toolbox for python"
        #   Convert homogeneous transform to a unit-quaternion
        #   Return a unit quaternion corresponding to the rotational part of the
        #   homogeneous transform t.

        qs = np.sqrt(np.trace(t)+1)/2.0
        kx = t[2,1] - t[1,2]    # Oz - Ay
        ky = t[0,2] - t[2,0]    # Ax - Nz
        kz = t[1,0] - t[0,1]    # Ny - Ox
        if (t[0,0] >= t[1,1]) and (t[0,0] >= t[2,2]):
            kx1 = t[0,0] - t[1,1] - t[2,2] + 1      # Nx - Oy - Az + 1
            ky1 = t[1,0] + t[0,1]           # Ny + Ox
            kz1 = t[2,0] + t[0,2]           # Nz + Ax
            add = (kx >= 0)
        elif (t[1,1] >= t[2,2]):
            kx1 = t[1,0] + t[0,1]           # Ny + Ox
            ky1 = t[1,1] - t[0,0] - t[2,2] + 1  # Oy - Nx - Az + 1
            kz1 = t[2,1] + t[1,2]           # Oz + Ay
            add = (ky >= 0)
        else:
            kx1 = t[2,0] + t[0,2]           # Nz + Ax
            ky1 = t[2,1] + t[1,2]           # Oz + Ay
            kz1 = t[2,2] - t[0,0] - t[1,1] + 1  # Az - Nx - Oy + 1
            add = (kz >= 0)
        if add:
            kx = kx + kx1
            ky = ky + ky1
            kz = kz + kz1
        else:
            kx = kx - kx1
            ky = ky - ky1
            kz = kz - kz1
        kv = np.matrix([kx, ky, kz])
        nm = np.linalg.norm( kv )
        if nm == 0:
            e0 = 1.0
            q = np.matrix([0.0, 0.0, 0.0])
        else:
            e0 = qs
            q = (np.sqrt(1 - qs**2) / nm) * kv
        return e0, q[0,0], q[0,1], q[0,2]


    def cb_string(self,data):
        pass

    def go_to_pose_entry(self,w,x,y,z):
        move_group = self.move_group


        pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.x = x
	pose_goal.orientation.y = y
	pose_goal.orientation.z = z
        pose_goal.orientation.w = w
        pose_goal.position.x = self.entry.position.x
        pose_goal.position.y = self.entry.position.y
        pose_goal.position.z = self.entry.position.z
        print(pose_goal.position.x)
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

    def plan_cartesian_path(self,w,x,y,z):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scale = 10
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []
	wpose = move_group.get_current_pose().pose
	xTarget = wpose.position.x
	yTarget = wpose.position.y
	zTarget = wpose.position.z
	xEntry = self.target.position.x
	yEntry = self.target.position.y
	zEntry = self.target.position.z
	for step in range(scale):
            xEnd = xEntry + (step/scale)*(xTarget-xEntry)
            yEnd = yEntry + (step/scale)*(yTarget-yEntry)
	    zEnd = zEntry + (step/scale)*(zTarget-zEntry)
	    wpose.orientation.x = x
	    wpose.orientation.y = y
	    wpose.orientation.z = z
	    wpose.orientation.w = w
            wpose.position.x = xEnd
            wpose.position.y = yEnd
            wpose.position.z = zEnd
	    rospy.loginfo(wpose)
            waypoints.append(copy.deepcopy(wpose))


        # We want the Cartsesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        self.display_trajectory(plan)
        self.execute_plan(plan)
        return plan, fraction

        ## END_SUB_TUTORIAL


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL


    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


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

    def igtl_importer(self):

        global pub_igtl_transform_out

        pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)
	# self.pub_Marker = rospy.Publisher('marker_entry', MarkerArray, queue_size=10)
	self.pub_entryMarker = rospy.Publisher('marker_entry', Marker, queue_size=1)
	self.pub_targetMarker = rospy.Publisher('marker_target', Marker, queue_size=1)
	self.pub_cortexMarker = rospy.Publisher('marker_cortex', Marker, queue_size=1)
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('igtl_importer', anonymous=True)

        rospy.Subscriber("IGTL_TRANSFORM_IN", igtltransform, self.cb_transform)
        rospy.Subscriber("IGTL_POINT_IN", igtlpoint, self.cb_point)
        rospy.Subscriber("IGTL_STRING_IN", igtlstring, self.cb_string)
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
        group_name = "ur5"
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
	self.entry = geometry_msgs.msg.Pose()
	self.target = geometry_msgs.msg.Pose()
	self.validEntry = False
	self.validTarget = False
	self.validMarkers = False
	self.markerArray = MarkerArray()
        #self.planning_frame = planning_frame
        #self.eef_link = eef_link
        #self.group_names = group_names

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    jsp = importer()
    jsp.igtl_importer()
