# Robot Path Following in ROS 
The task is to write a ROS Kinetic package containing node that:
(1) Subscribes to /waypoint_cmd to find out the next waypoint in the route that the robot needs to travel. This route is provided by a separate referee package (trajectory_referee_456).
(2) Uses the tf library to find the location of the robot according to its odometry. (3) Sends commands to /cmd_vel_mux/input/navi to go to each point in the path.
You do not need to make use of the robot's range sensors (if you don't want to). Odometry will be enough for this task.
