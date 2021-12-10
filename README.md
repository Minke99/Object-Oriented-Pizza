# Object-Oriented-Pizza

## Introduction
The aim of this project is to make a working prototype for a pizza delivery system. It is implemented this with the virtual Turtlebot 3 in Gazebo. In this system, there is a restaurant which bakes the pizzas, 8 houses to which pizza must be delivered, and a charging station for the robot.

Pizza shops are the target market for this system. However, since the system is designed to be extensible and is focused on autonomous delivery, it is also relevant to other food and drink stores that wish to upgrade their delivery system.

Autonomous delivery eliminates the need for human drivers, which reduces concerns of availability and cost. This is especially important in current society due to the increasing demand from an increasing population, which is more reliant than ever on online orders and delivery due to the impact of the pandemic.

Numerous health and safety issues also come from human delivery, which have also been under increasing media attention. Delivery in hostile weather conditions could be very dangerous for workers, especially those on bikes and scooters. Large volumes of orders cause exhaustion in workers and tired drivers may lead to accidents or mistakes. This could all be avoided with the automation of the pizza delivery system.

## Platform
ROS Melodic
 
Gazebo

Turtlebot3

## System Design
High level overview:
<div align=center><img width = '500' src = "https://github.com/KNN-6948/Pizza-Delivery/blob/main/chart/High%20level%20overview.jpg"></div>

|Node|Roll 
|---     |---  
|User_Interface|Client 
|Delivery_Platform|Order Delivery System 
|Move_Robot|Navigation system
|Apriltag_ROS|Recognizing AprilTags

#### Demo
Turtlebot & AprilTag:
<div align=center><img width = '500' src = "https://github.com/KNN-6948/Pizza-Delivery/blob/main/demo/demo1.png"></div>

RViz mapping:
<div align=center><img width = '500' src = "https://github.com/KNN-6948/Pizza-Delivery/blob/main/demo/rviz.png"></div>

