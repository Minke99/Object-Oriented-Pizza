#ifndef HOUSE_H_
#define HOUSE_H_
#include <ros/ros.h>
#include "a_user_portal/order.h"

class CUser
{
    public:

		// Constructor for house node
		CUser(ros::NodeHandle nh);

		// Deconstructor
		~CUser();

		void UserInput();

    private:

		// ROS NodeHandle
		ros::NodeHandle nh_;

		// ROS publisher for orders from the house
		ros::Publisher order_pub_;

		static const int num_house_ = 8;
    
};


#endif
