# rostalker
Testing for communication across devices/robots, using ROS kinetic on an ubuntu 16.04 LTS

### Please note
The two devices begin used are a HP notebook running ubuntu 16.04 LTS and a ubuntu 16.04 64 bit virtual machine in virtual box

# Information
Current model of robo1_server and robo2_server is really weird, technically there is only one resource the "data stream", only one can publish to it at once. This would mean that deadlocks aren't possible as the circular weight condition is violated. However, in the model used in robo1_server and robo2_server, its almost as if there are 2 resources, that can also access themselves making deadlocks possible. This is easily prevented as you can just use a ticket lock (weird ticket lock used) to alternate between the 2 satisfying mutual exclusion and sychronization. 

**R1-R2-R3 Model**
In future models better programmings is needed to circular waits aren't just added out of nowhere, in a model with 3 robots, 1 robot called R3 will be a resource used by R1 and R2. In this model, again deadlocks should be impossible as no circular weights exist, but mutual exlcusion is necessary requiring the use of locks, since a R1 could preempt R2s critical section. There are some issues as. Do we need an atomic write and read for the locks in R3, does each service call run inside R3 or run inside R1 and R2 causing multiple computers acessing memory to see the lock which will prevent mutual exclusion. If it runs in R3, is the service preempted before completion? Does it spawn 2 threads/processes to handle separated requests? It may require using atomic actions. There is also the issue of space, R2 can't move things to R1 if R1 is out of space, so semaphores may be needed. 

# TODO
1. Expand to multiple CPUs 
2. There are mutual exclusion and deadlock concerns now with multiple resources 

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


# Service Tests 

On machine 1
**In new terminal:**
source ~/rostalker/devel/setup.bash
rosrun sync_robot_test service_test.py
**In new terminal:**
source ~/rostalker/devel/setup.bash
rosrun sync_robot_test service_test_client.py

On machine 2
**In new terminal:**
source ~/rostalker/devel/setup.bash
rosrun sync_robot_test service_test_client.py

If everything works the server should print out 2 million and nothing different from that, this means that the mutual exclusion primitive (mutex) worked out. Removing the lock and retest will yield non-determinstic results signaling a race condition

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
