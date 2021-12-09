
#include "delivery_platform/TurtleBot.h"
#include "delivery_platform/House.h"
#include "delivery_platform/Restaurant.h"
#include "delivery_platform/Preloading_system.h"


int main(int argc, char* argv[])
{
	// Initialise the function 
	ros::init(argc, argv, "turtlebot3_drive");

	// Create a nodehandle
	ros::NodeHandle nh;

	// Define Turtlebot, restaurant, house and loading system
	// Each of them needs to know the nodehandle. 
	CTurtleBot robot(nh);
	CRestaurant restaurant(nh);
	CHouse house(nh);
	CPreload system(nh);

	// Set the loop rate = 1 which is one second
	// FOr this task, it is enough to using this rate of refreshing
	// Do not need a very high performance reaction.
	ros::Rate loop_rate(1);
	
	// Run the while loop indefinitely until program is terminated by the user
	while (ros::ok())
	{

		// Correct the loop frequency
		// Ensure the loop rate is corrected to 1
		ros::spinOnce();
		loop_rate.sleep();

		// The robot is always consuming power when it is turned on
		// SInce our looprate = 1. It means that our robot cost 0.3% of
		// power per second.
		robot.ConsumePower();
	}

	return 0;
}

 
