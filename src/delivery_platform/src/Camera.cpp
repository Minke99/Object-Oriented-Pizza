
#include "delivery_platform/Camera.h"

// Camera constructor
// Define NodeHandles and subscribe a topic
CCamera::CCamera(ros::NodeHandle nh)
	// Private variable to store node handle 
	:nh_(nh)
{
	// Print statement to let the user know that the Camera is set up
	ROS_INFO("[Robot]: Camera set up");

	// Subscribe a topic with a buffer size of 10. Perform callback function 
	// We use the callback function to get the message of scaned April Tag ID.
	sensor_sub_ = nh_.subscribe("idTrans", 10, &CCamera::IDMsgCallBack, this);

	// Pub the location of current robot when it reach any building on the map.
	location_pub_ = nh_.advertise<delivery_platform::location>("Location",10);
}

// Camera destructor 
CCamera::~CCamera()
{
	// Print statement to let the user know the Camera is destroyed
	ROS_INFO("[Robot]: Camera deleted\n");
}


// Id callback tells what the camera sees.
void CCamera::IDMsgCallBack(const apriltag_ros::id::ConstPtr &id)
{	
	// Whether reaches restaurant
	if(id->detected_id == restaurant_)
	{
		// pub a position message that robot is at restaurant.
		delivery_platform::location current_location;

        current_location.where = "restaurant";

        location_pub_.publish(current_location);

	}
	// Whether reaaches charge station
	else if (id->detected_id == charge_station_)
	{
		// pub a position message that robot is at charge station.
		delivery_platform::location current_location;

        current_location.where = "charge_station";

        location_pub_.publish(current_location);
	}
	else
	{
		// pub a position message that robot is at which house.
		delivery_platform::location current_location;

        current_location.where = "house";
		current_location.num = id->detected_id;

        location_pub_.publish(current_location);
	}
	
	
}


