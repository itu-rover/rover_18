--Do in Rover (Master)
roscore 
roslaunch rover_base initroverall.launch  // opens initrover.launch , imu.launch and rover_sim.launch 
roslaunch rover_base start_ui.launch // open javascript node
roslaunch rover_navigation move_base.launch 
--for autonoumus action  if you want to choose  creating waypoint odom with navsat run these in terminals 
rosrun rover_control waypointhandler4.py
rosrun rover_Control rover_autonom4.py
 

----


--Do in Computer(Slave)
write in a web browser MASTER_IP:8002 

for seeing Rviz you have to open VNC client server
----
