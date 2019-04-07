# Low Level Control
This package is the bridge between the ROS computer and the Arduino controller

The node "llc.py" subscribes to the topic /cmd_vel and sends the info to the arduino via UART

The velocity angular z should be [-1,1]
The velocity of x is [-0.2,0.2]
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
