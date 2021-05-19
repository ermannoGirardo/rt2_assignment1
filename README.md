# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige) Ermanno Girardo S4506472

# Requirements of action branch
Localisation and path planning of a non-holonomic robot in an environment with Gazebo simulator.
Taking the code in the main branch i want to replace the go_to_point service with an action service by custom action message Position.action.
In this way the user can stop the robot in whatever place wants, instead of stopping the robot once it reaches the target goal.
Infact implementing the action service is possible to cancel the goal once the user press 0.

# What you can find in this branch 
In this branch you can find two nodes in python and two nodes in C++:

* go_to_point.py is an action server used to reach the position of a random target.
  In this script is implemented the behaviour of an Finite State Machine in order to:
  1) align the robot with the goal
  2) move the robot in the direction of the goal
  3) once the goal is reached the robot align its orientation with the goal's orientation
  This node uses a publisher in order to publish the velocity of the robot  
  on /cmd_vel topic and it uses a subscriber in order to check the robot's position on /odom topic.

* user_interface.py implements an user interface in order to start or stop the robot.
  In particular it is a client that reads the user command and via custom service message Command.srv 
  send it to the state_machine node.
  When the user press 1 the node sends a "start" messagge to the user_interface server.
  Instead if the user press 0 the node the function of go_to_point is stopped and in state_machine node 
  the goal is cancelled, setting also the velocity to zero.
  
* position_service.cpp is a simple server that has the scope of generate a random target.
  This node infact generate three random components (x and y components for the position,
  theta component for the orientation) in a specified interval [min,max] via 
  custom service message RandomPosition.srv.
  
* state_machine.cpp is a node wich communicats with all other nodes, in particular:
  1) a client is generated in order to set the fields of the random target and send the request
     to position_service
  2) The node implements a function callback of the user_interface , in order to check the request done
     by the user, setting properly the value of the variable start.
  3) Implements an action client in order to cancel the goal if the user decides to stop the robot.
  4) Publish on topic /cmd_vel the velocity equal to zero if the user decides to stop the robot.

* A coppelia scene in order to launch the simulation also in Coppelia (Vrep).
  
# Launch files
In the folder launch you can find two launch file:

* sim.launch starts the simulation launching Gazebo simulator, with an empty map and loading the Universal Robot desciption on it.
  It run all the four nodes above described.
* simVrep.launch starts only the four nodes described above without launching Gazebo simulator and load the URDF on the ROS server.
 
# How to launch the simulation in Gazebo
In order to launch the simulation on Gazebo copy and paste the following command on Ubuntu shall

```
roslaunch rt2_assignment1 sim.launch
```

![GazeboSimGraph](https://user-images.githubusercontent.com/48509825/118884149-628ff800-b8f6-11eb-8165-a19e41745097.png)
# How to launch the simulation on Vrep
In order to launch the simulation on Vrep:
```
https://www.coppeliarobotics.com/downloads
```
Download the education version zip and unzip it, should be something as CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.
Then move into this folder and change the permission of the coppeliaSim.sh by digit 
```
chmod +x coppeliaSim.sh
```
Then in order to launch the simulation open another terminal and run the ros master then launch the executable of Coppelia
```
roscore &
./coppeliaSim.sh
```
Check if ROS is correctly sourced before proceeds
Then you must open the scene coppeliaScene.ttt, you should see a pioner mobile robot in the centre of the map
Now to start the simulation open another terminal, move to the ros workspace and digit the follow command and start the simulation on Coppelia.
```
roslaunch rt2_assignment1 simVrep.launch
```
If you press 1 in order to start moving the robot you must see the pioner moves to reach the goal position.
if you press 0 you must see the pioner stops in the current position.
If the simulation is too slow click on the hare on the top.

![VrepSimGraph](https://user-images.githubusercontent.com/48509825/118884208-72a7d780-b8f6-11eb-8146-c53e642f9187.png)


