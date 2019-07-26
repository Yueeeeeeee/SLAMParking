- Before starting the server, make sure that a ROS Master is running (or start with "roscore").

- Start server with "rosrun server server"

- To manually establish a connection use "nc -zv <YOUR_IP> 4444"

- To manually publish a steering angle run: "rostopic pub /angle std_msgs/Float32 3.21"
 


