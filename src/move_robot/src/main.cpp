#include "move_robot/Navigation.h"



int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");

//////////////////////testing/////////////
	ROS_INFO("\nHi\n");
///////////////////////////////////
   	// Create a nodehandle
	ros::NodeHandle nh;
	CNavigation Nav(nh);
   	ros::Rate loop_rate(50);

   	while (ros::ok())
	{
		// Correct the loop frequency
		// Ensure the loop rate is corrected to 50
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
