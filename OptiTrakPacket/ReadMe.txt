roslaunch contents:

<launch> 
  <node pkg="mocap_optitrack" 
	type="mocap_node" 
	name="mocap_node" 
	respawn="false"
	launch-prefix=""
	required="true">
    <rosparam file="$(find mocap_optitrack)/config/mocap.yaml" command="load" />
  </node>
</launch>



yaml file:
# 
# Definition of all trackable objects
# Identifier corresponds to Trackable ID set in Tracking Tools
#
rigid_bodies:
    '1':
        pose: Robot_1/pose
        pose2d: Robot_1/ground_pose
        frame_id: Robot_1/base_link
    '2':
        pose: Robot_2/pose
        pose2d: Robot_2/ground_pose
        frame_id: Robot_2/base_link



