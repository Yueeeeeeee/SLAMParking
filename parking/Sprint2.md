Updated on 06.06.2019 based on current progress and new concepts.

*Goal for the second sprint*

We should be able to drive the car around in manual mode and the LIDAR + SLAM maps the area around.
Then we can manually specify a parking position with Rviz or later by detection a large enough corner in the map.
On a button press a path out of several pre-defined paths depending on the relative position of the goal is selected and the car follows this multi-curve path to this goal. To increase accuracy, we correct the motion with feedback from odometry and SLAM. Obstacles are ignored by the path planning.

*Tasks*

- *Daniel* Get the goal position from Rviz. Extend the Driver node to accept Twist messages.
- *Julia* Implement a custom path planner. Input: Location and Goal, Output a list of bezier splines.
- *Jannik* Follow several bezier splines with position feedback (Custom local planner).
- *Yue & Xhang* Adapt the ROS navigation stack.
- *Marcel* Collect and order a set of possible location and parking goal scenarios. We are going to use this list to develop, test and challenge our planning algorithms.
