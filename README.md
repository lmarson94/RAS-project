# ras_project_group1
Repository for Group 1 DD2425 2018 

catkin_make

1. roslaunch ras_launch motors.launch 

For exploration:
  2. roslaunch ras_launch milestone4.launch map_file:="contest_maze_2018.txt" readmap:="0"
For rescue:
  2. roslaunch ras_launch milestone4.launch map_file:="contest_maze_2018.txt" readmap:="1" 
3. rviz

For exploration:
  4. rosrun ras_state_machine statemachine_MS4.py -explore 
For rescue:
  4. rosrun ras_state_machine statemachine_MS4.py -rescue
  
