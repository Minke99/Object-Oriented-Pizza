// Camera.h

#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include <string>
#include "apriltag_ros/id.h"
#include "delivery_platform/location.h"

// This is the Lidar which inherits from Sensor:
// It defines the methods and variables that a Lidar has
// It is mainly used to follow the wall


class CCamera
{
  public:
	// Constructor of Lidar that initialises the subscriber and nodehandle
	CCamera(ros::NodeHandle nh);

	// Deconstructor
	~CCamera();


	std::string bot_location_;

  private:
	// ROS NodeHandle
	// Required to operate ROS nodes
	ros::NodeHandle nh_;

	ros::Subscriber sensor_sub_;
	ros::Publisher location_pub_;

	const int restaurant_ = 8;
	const int charge_station_ = 9;



	// Callback function that 
	void IDMsgCallBack(const apriltag_ros::id::ConstPtr &id);
};


#endif // CAMERA_H_
