# rostalker
Testing for communication across devices/robots, using ROS kinetic on an ubuntu 16.04 LTS

### Please note
The two devices begin used are a HP notebook running ubuntu 16.04 LTS and a ubuntu 16.04 64 bit virtual machine in virtual box


# TODO
1. services for the turtlesim 
2. commuicate data 
3. synchronize protocols between 2 different computers
4. create a readable README 



# Turtletest commands
Master:
rosrun turtlesim turtlesim_node 

Slave: 
rosrun turtletest movement_talker 1    0 for straight circle

# Install 

ROS kinetic on ubuntu 16.04 LTS is required 

1. git clone [url of github] 
2. cd rostalker 
3. catkin_make 
4. source devel/setup.bash 

networking between the 2 different computers must also be complete 
