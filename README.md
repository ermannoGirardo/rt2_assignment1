# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

This is the action branch you may find:
	Instead of using a position server as in the main branch here it is replaced by an action server /action/Position.action
	In this way the user can stop the robot in a current position simply press 0 after has pressed 1 in order to generate a new random target and setting the velocity of the robot
	To run the simulation in Gazebo simply copy the following command in the ubuntu shall:	
```
	rosrun rt2_assignment1 sim.launch
```
	A new simulation in Coppelia is implemented.
	To run the simulation follow this simply instructions:
```
	roscore &      #in order to launch the ROS master
```
	Only after having installed CoppeliaSim move into the executable folder and launch it
```
	./coppeliaSim.sh
```
	Please check that ROS is correctly setted before proceed
	
	Then add the Coppelia scene that you can find in this branch:
	coppeliaScene.ttt
	
	To launch the simulation digit the following command
	
```
	roslaunch rt2_assignment1 simVrep.launch
```
	
