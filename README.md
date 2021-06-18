# rostalker
Testing for communication across devices/robots, using ROS kinetic on an ubuntu 16.04 LTS

### Please note
The two devices begin used are a HP notebook running ubuntu 16.04 LTS and a ubuntu 16.04 64 bit virtual machine in virtual box

# TODO 
1. Communicate custom messages 
2. test out the rostopic feature, does it go both ways? can the master be told information from the slave and vice versa? 

# Turtletest commands
Master:
rosrun turtlesim turtlesim_node 

Slave: 
rosrun turtletest movement_talker 

The turlte should start moving in a circle on the gui 


# Install 

ROS kinetic on ubuntu 16.04 LTS is required 

1. git clone [url of github] 
2. cd rostalker 
3. catkin_make 
4. source devel/setup.bash 

networking between the 2 different computers must also be complete 
