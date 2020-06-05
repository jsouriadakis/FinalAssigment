# How to use:

## Initialise the connection between ROS and Slicer 4.8.1:
### Setting up the connection
### Slicer Part
1.	Launch Slicer 4.8.1
2.	Go OpenIGTLinkIF extension
3.	Create a connection with Slicer as the server on port 18944
4.  Check that the current status is set to "WAIT"
### ROS Part
1.	Start the VM (if using one. I run this on Ubuntu, so it was not required) with the ap-propriate configuration for two-way communication
2.	Launch the bridge file of the ros_igtl_bridge 
3.	Choose to run as client
4.	Set IP to the appropriate IP found by writing “ifconfig” in your terminal. The IP is expected to be set in the bridge.launch file but you can comment this part out if you want to change it everytime
5.	Set port to 18944. The IP is expected to be set in the bridge.launch file but you can comment this part out if you want to change it everytime
6.  If you check Slicer, the status should have changed to "ON"

A connection should be now established.

### Sending data from Slicer to ROS
### Step 1:
1.	In Slicer 4.8.1
2.	Go to Markups
3.	Create two new MarkupFiducials called “Entry” and “Target”
4.	Place a point in the workspace (I did this the other way around. I manually moved the robot in RVIZ and those coordinates instead)
### Step 2:
1.	Launch a new terminal
2.	Go to your workspace and source it
3.	Enter “roslaunch ismr19_moveit demo.launch”
### Step 3:
4.	Launch a new terminal
5.	Go to your workspace and source it
6.	Enter “rosrun robot_control igtl_importer.py”
### Step 4:
1.	Launch a new terminal
2.	Go to your workspace and source it
3.	Enter “rosrun robot_control igtl_exporter.py”
### Step 5:
1.	Go back to slicer
2.	Go to IGT -> IGTLinkIF
3.	Scroll down to I/O Configuration
4.	Click send for both “Entry” and “Target”
5.  The robot should now move to the entry point and you should be prompted to click "enter" on the terminal to proceed to the target point. 

Note: You need to send both points for the robot to begin the operation

## Running the whole pipeline:
The whole pipeline is a multistep process. Most parts require some manual observations, and this serves to provide extra safety/protection, assuming this would be translated to a real-life scenario. The steps required to run it are as follows:
1.	We launch Slicer and load all our volumes (critical structures, cortex and entry-target points)
2.	We convert the critical structures from “.vtk” format to labelMaps/markupFiducials using Slicer’s “Model to Label Map” module using as reference volumes the labelMaps given from the previous coursework.
3.	We run our PathPlanner algorithm in order to calculate and save the best entry-target pair. Our scene should now resemble figures 2-4 (without the yellow background on the entry-target points)
4.	We then initialise the connection and launch our robot in RVIZ as described previously.
5.	We then start the calibration process as mentioned in the Calibration (to translate points between ROS and Slicer) section of the report.
6.	Then using the resulting transformation matrix, we transform the critical structures, cortex and entry-target points of Slicer to match the workspace of ROS. 
7.	We use the ‘harden’ functionality on Slicer to have a correct representation of the structures/points and send the correct entry-target points to ROS.
8.	We load the cortex (converted to .stl format using Slicer) as a marker node to RVIZ through RVIZ’s interface, using a scale factor of 0.001 to match Slicer’s and ROS’ measurements. 
9.	The cortex’s origin point in RVIZ is set according to the bounding box created during the calibration step. Once we ensure that the robot is placed correctly around the structure, we can start sending our entry and target point.
10.	We go again to IGT in Slicer and click “Send” on the “Entry” and "Target" points. We should now be able to see the robot’s end effector moving to the entry point in RVIZ. The “Entry” point is loaded as a cyan marker. The robot will not move unless both points are sent.
11.	At the same time, we can check the Slicer scene to confirm that the end effector is at the correct entry point. This is shown by the yellow background around the optimal entry position.
12.	There should now be a prompt in the terminal to press "enter" to proceed to the target point. After doing this, we should be able to see the robot’s end effector moving to the target point from the entry point in a straight line . The “Target” point is loaded as a blue marker.
13.	At the same time, we can check the Slicer scene to confirm that the end effector is at the correct target point. This is shown by the yellow background around the optimal target position.

These should be all the steps required to perform the whole pipeline. Scenes of these steps and saved models can be found within the repository under the “models and scenes” folders

You can check the report for a more detailed explanation on how the different parts of the pipeline work.
