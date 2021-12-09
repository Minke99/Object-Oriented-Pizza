// House.h

#ifndef HOUSE_H_
#define HOUSE_H_

#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <fstream>    // file input
#include "delivery_platform/delivered.h"
#include "delivery_platform/preloadpos.h"
#include "delivery_platform/location.h"

class CHouse
{
    public:

		// Constructor for house node
		CHouse(ros::NodeHandle nh);

		// Deconstructor
		~CHouse();

    private:

		// ROS NodeHandle
		ros::NodeHandle nh_;

		// ROS subscriber to preloaded positions
		ros::Subscriber preload_sub_;

		// ROS subscriber to delivered, from the bot
		ros::Subscriber delivered_sub_; 

		// ROS subscriber to idTrans
		ros::Subscriber location_sub_;


		static const int num_house_ = 8;
		int house_x_pos_[num_house_];
		int house_y_pos_[num_house_];

		void PreLoadPos();
		void PositionMsgCallBack(const delivery_platform::preloadpos::ConstPtr &pos);
		void LocCallback(const delivery_platform::location::ConstPtr &location);
		void DeliveredMsgCallBack(const delivery_platform::delivered::ConstPtr &order_delivered);
    
};


#endif
