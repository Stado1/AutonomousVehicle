# AutonomousVehicle

## Project description
The goal of this project was to create a track made up of short walls, and have a TurtleBot3 complete laps around the track using only a single camera based navigation system.

The software used for this project:
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- OpenCV 4.6.0
- TurtleBot3 2.3.3


### How to run
First install TurtleBot3 and OpenCV with APT. 
Open a new terminal, and go the Autonomous vehicle folder. There use command: "colcon build". Then use the command "source install/setup.bash". Then use then command: "ros2 launch my_world_pkg world_with_trutlebot-CHEKC IF THIS IS CORRECT". This opens a Gazebo world with the parkour in it. See image. ![Logo](images/ParkourScreenshot.png).

Now open a new terminal, and go the Autonomous vehicle folder. Then use the command "source install/setup.bash". Then use the command "ros2 run velocity_aSOMETHINF CHECK IF THIS IS CORRECT". This will make de robot drive and follow the track.

If you also want to see what the camera sees then do this: Now open a new terminal, and go the Autonomous vehicle folder. Then use the command "source install/setup.bash". Then use the command "ros2 run vision_pkg SOMETHING CHECK IF THIS IS CORRECT". Now you should see the camera footage.


### How does the navigation system work

