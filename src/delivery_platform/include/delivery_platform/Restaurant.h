// Restaurant.h

#ifndef RESTAURANT_H_
#define RESTAURANT_H_

#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include "delivery_platform/delivering.h"
#include "delivery_platform/location.h"
#include "a_user_portal/order.h"
#include "apriltag_ros/id.h"
#include "delivery_platform/preloadpos.h"


class CRestaurant
{
    public:
        // Constructor initialises restaurant and node handle
        CRestaurant(ros::NodeHandle nh);

        // Deconstructor
        ~CRestaurant();
    private:
        // ROS NodeHandle
        ros::NodeHandle nh_;

        // ROS subscriber to preloaded positions
        ros::Subscriber preload_sub_;

        // ROS subscriber to house orders
        ros::Subscriber order_sub_; 

        // ROS subscriber to location
        ros::Subscriber location_sub_;

        // ROS publisher for delivery?
        ros::Publisher delivering_pub_;

		// Max order can exist in the restaurant is 100.
		const static int max_order_ = 100;
		const static int num_house_ = 8;

		// Store the position of the restaurant.
		float restaurant_x_pos_;
		float restaurant_y_pos_;

		float house_x_pos_[num_house_];
		float house_y_pos_[num_house_];	
		
		struct order_array
		{
			// The pizza loaded on the bot.
			// Each cell of the array represents the number of pizza
			// in the corresponding order. 
			// eg. if qizza = 3, 3, 1, it means 3 pizzas for first 2 orders
			// and 1 pizza for the third order.
			int pizza_array_[max_order_];

			// Load pointer points to the top of the pizza array.
			// Also the pointer is the same of for the pos queue below.
			int load_ptr_;

			// The adress array where the pizza needs to send.
			// These two arrays should be same in length.
			// Also, pointed by load_ptr_ simutaneously means that
			// these coordinate is the next destination to go.
			float x_pos_array_[max_order_];
			float y_pos_array_[max_order_];
		}order_array;

        // Where the robot is right now
        int detected_id;
        
        // make this enum?
        // Restaurant is id 0
        const int at_restaurant_ = 8;
        const int order_sent_ = 1;
        const int PizzaMade = 1;
        
        int load_once_;
        int send_status_;

        // Callback when new msg arrives in order topic 
		void LocCallBack(const delivery_platform::location::ConstPtr &location);

        void OrderMsgCallBack(const a_user_portal::order::ConstPtr &order);

        void idCallBack(const apriltag_ros::id::ConstPtr &idTrans);

		void PositionMsgCallBack(const delivery_platform::preloadpos::ConstPtr &pos);


        int SendOrder();

};


#endif
