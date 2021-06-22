# rostalker
Testing for communication across devices/robots, using ROS kinetic on an ubuntu 16.04 LTS

### Please note
The two devices begin used are a HP notebook running ubuntu 16.04 LTS and a ubuntu 16.04 64 bit virtual machine in virtual box


# TODO
1. Clean up robo[1 and 2] servers 
2. Expand to multiple CPUs 


# Robo test commands 

On machine 1 (robo1)  
**In new terminal:**  
source ~/rostalker/devel/setup.bash  
rosrun sync_robot_test robo1_server.py  
**In new terminal:**  
source ~/rostalker/devel/setup.bash  
rostopic echo /robo1/data  

On machine 2 (robo2)  
**In new terminal:**  
source ~/rostalker/devel/setup.bash  
rosrun sync_robot_test robo2_server.py  
**In new terminal:**  
source ~/rostalker/devel/setup.bash  
rostopic echo /robo2/data  

When ready press enter on the robo1_server.py and robo2_server terminal tabs to start the communication and syncronization.
You should see that in the topics /robo1/data  /robo2/data are alternating, 10 blocks are updated in robo1 data and then 10 updated in robo2 this repeats forever.    
Ctrl-c to terminate the process 


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

**networking between the 2 different computers must also be complete**
