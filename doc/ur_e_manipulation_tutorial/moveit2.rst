rosrun husky_ur_moveit_config customize_moveit.sh husky_ure_tutorial_moveit_config
catkin_make
roslaunch husky_ure_tutorial_moveit_config setup_assistant.launch
laser_enabled:=false kinect_enabled:=false urdf_extras:="$(catkin_find husky_ure_tutorial_description urdf/husky_ur5_e_tutorial_2.urdf.xacro --first-only)"
