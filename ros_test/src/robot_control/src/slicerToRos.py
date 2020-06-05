#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from ros_igtl_bridge.msg import igtlpoint
from ros_igtl_bridge.msg import igtltransform
from NodeTypeEnum import NodeTypeEnum


class SlicerToRos:

    def cbPoint(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Position = (%f, %f, %f).", data.pointdata.x, data.pointdata.y,
                      data.pointdata.z)
        self.moveToGoal(data.pointdata.x, data.pointdata.y, data.pointdata.z)

    def moveToGoal(self, x, y, z):
        moveGroup = self.moveGroup

        # Set pose
        pose = geometry_msgs.msg.Pose()
        x, y, z = self.getConvertedCoordinates(x, y, z)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        moveGroup.set_pose_target(pose)

        # Execute plan
        plan = moveGroup.go(wait=True)

        # call stop() and clear for stability
        moveGroup.stop()
        moveGroup.clear_pose_targets()

        currentPose = moveGroup.get_current_pose().pose
        return self.allClose(pose, currentPose, 0.01)

    def getConvertedCoordinates(self, x, y, z):
        return self.convertCoordinate(x), self.convertCoordinate(y), self.convertCoordinate(z)

    @staticmethod
    def convertCoordinate(coordinate):
        return coordinate / 1000.0

    def allClose(self, goal, actual, tolerance):
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.allClose(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.allClose(pose_to_list(goal), pose_to_list(actual), tolerance)
        return True

    def start(self):

        global pubIgtlTransformOut

        pubIgtlTransformOut = rospy.Publisher(NodeTypeEnum.TRANSFORM_OUT.value, igtltransform, queue_size=10)
        rospy.init_node(NodeTypeEnum.IMPORTER.value, anonymous=True)
        rospy.Subscriber(NodeTypeEnum.POINT_IN.value, igtlpoint, self.cbPoint)

        ## First initialize and instantiate moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface_object to help us navigate the robot
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander_ object. This helps us plan and execute movements
        groupName = "panda_arm"
        moveGroup = moveit_commander.MoveGroupCommander(groupName)

        ## To display the trajectory of the robot in rviz
        trajectoryPublisher = rospy.Publisher(NodeTypeEnum.MOVE_GROUP_PLANNED_PATH.value,
                                              moveit_msgs.msg.DisplayTrajectory,
                                              queue_size=20)

        self.robot = robot
        self.scene = scene
        self.moveGroup = moveGroup
        self.trajectoryPublisher = trajectoryPublisher

        rospy.spin()


if __name__ == '__main__':
    slicerToRos = SlicerToRos()
    slicerToRos.start()
