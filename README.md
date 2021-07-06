# IIT-B_Eyantra-Vitarana_Drone-Competition-2020-
ROS code (Python) for Eyantra Robotics competition held by IIT-B for Vitarana_drone Category(2020)  

Vitarana Drone. Vitarana (IPA: vitaraṇa) means distribution in Sanskrit and many extant languages of South Asia.

In this theme, I learned concepts of control systems, path planning, image processing and algorithm development. 
I was exposed to tools such as the Robot Operating System, robotics simulator Gazebo, the Python programming language and many of its libraries.

Task 1:
   The objective of Task-1 was to achieve position control for eDrone in ROS-Gazebo environment.
   Broken into two parts where:

A) The objective of the task is to design an attitude controller for the eDrone. 
   In other words, attitude of a drone means the orientation of the drone in terms of Euler angles ie. roll, pitch and yaw.
   To achieve control over the attitude of eDrone, we designed a PID controller.
   The controller is a closed loop controller with the present orientation of eDrone being fed back by the IMU sensor.

B) The objective of the task is to design a position controller for the eDrone.
   The position was described in terms of GPS co-ordinates ie. latitude, longitude & altitude.
   To achieve control over the position of eDrone, we designed another PID controller which was in cascade with the attitude controller designed in Task 1 A).
   
   
Task 2:
   The aim of this task is to pick a parcel and deliver it to its destination.
   Broken into three parts where:

A) The aim of this task was to detect a QR code placed on the parcel box via the camera of eDrone and extract the information encoded in the QR code.

B) The aim of this task was to be able to pick and drop the parcel box using the eDrone in the Gazebo world.
   For picking the box, eDrone has to land exactly on top of the box on the centre and then activate the gripper present on the eDrone.

C) Navigation:
   This task merged tasks 2A and 2B with obstacle avoidance and path planning.
   We used the onboard rangefinder sensors on the eDrone to detect obstacles and navigate around them.
   
Task 3:
   The aim of this task is to detect the landing markers present in the general vicinity of a given GPS coordinate.
   Our drone had to hover at certain height from the given coordinates and scan for the landing marker by applying image processing techniques 
   and design an algorithm to take the eDrone above the landing marker and eventually land on it.
   
   Broken in three parts

  A) Navigation: As used in Task 2.
  
  B) Marker detection using Haar Cascade Classifiers.
  
  C) Hovering above Marker.
  
Task 4:
   The Task 4 is a cumulative task of all the previous tasks.
   The main aim of this task is to complete a set of deliveries form warehouse location to their destinations.
   
   The eDrone spawned on the roof of a warehouse.
   
   The warehouse had a grid on which the parcel boxes are placed.The grid consists of square cells.
   
   The grid consisted of square cells. The parcel boxes were placed exactly at the centre of the cells.
   The manifest of the deliveries was provided to us in a Comma Separated Values file format (.csv) present in the scripts folder.
   We needed to read the .csv file in the python program to get information about the location of cells and their destination locations.
   The destination coordinate was given in the .csv file pointed to the terrace or rooftop of the destination building and we had to search for a landing marker
   as done in the previous Tasks
   After dropping a box we needed to go to the warehouse location to perform the next delivery. 
   After completing all the deliveries listed in the .csv file, our eDrone must land back to the start coordinate. The landing of eDrone will indicate the end of 
   this task
   
   [![Task_4](https://img.youtube.com/vi/nBAJfkexXyU/0.jpg)](https://www.youtube.com/watch?v=nBAJfkexXyU)
   
