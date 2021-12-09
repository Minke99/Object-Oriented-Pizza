// Turtlebot.h

#ifndef TURTLEBOT_H_
#define TURTLEBOT_H

#include "Camera.h"
#include "delivery_platform/delivered.h"
#include "delivery_platform/destination.h"
#include "delivery_platform/delivering.h"
#include "delivery_platform/preloadpos.h"

// This is the class of the turtlebot itself
// It implements the logic of load, delivery and send pizza
// It has the feature of a battery. When it is low,
// it will automately find a charge station to charge.
class CTurtleBot
{
  public:
	// Constructor of Turtlebot that initialises the publisher and nodehandle
	CTurtleBot(ros::NodeHandle nh);

	// Deconstructor of Turtlebot that frees the memory
	~CTurtleBot();

	// Load pizza from resturant
	bool LoadPizza(int num_pizza, float x_pos, float y_pos);

	// Send pizza to customer when reaches the ideal house
	void SendPizza(int house_num);

	// Update the state of the battery
	void ConsumePower();

	// Robot has a Camera sensor
	CCamera* bot_camera_;

  private:
	// ROS NodeHandle
	ros::NodeHandle nh_;

	ros::Subscriber delivering_sub_;

    // ROS subscriber to preloaded positions
    ros::Subscriber preload_sub_;

	// ROS subscriber to location
    ros::Subscriber location_sub_;

	// publisher of the delivered topic
	ros::Publisher delivered_pub_;

	// publisher of the destination topic
	ros::Publisher destination_pub_;

	// Max pizza that can be carried on the bot
	const static int max_pizza_ = 8;

	// Defines the threshold of the battery state
	const int low_battery_threshold_ = 90;
	const int full_battery_value_ = 99;
	const double change_battery_ = 0.3;

	struct loaded_pizza_{
		// The pizza loaded on the bot.
		// Each cell of the array represents the number of pizza
		// in the corresponding order. 
		// eg. if qizza = 3, 3, 1, it means 3 pizzas for first 2 orders
		// and 1 pizza for the third order.
		int pizza_array_[max_pizza_];

		// Load pointer points to the top of the pizza array.
		// Also the pointer is the same of for the pos queue below.
		int load_ptr_;

		// The adress array where the pizza needs to send.
		// These two arrays should be same in length.
		// Also, pointed by load_ptr_ simutaneously means that
		// these coordinate is the next destination to go.
		float x_pos_array_[max_pizza_];
		float y_pos_array_[max_pizza_];
	}loaded_pizza_;

	// The battery of robot. It cantains the remaining power.
	double battery_;

	// State of battery. Fully charged when 100%
	// LowBattery when <20%, others are normal.
	enum Battery_state{
		Normal,
		LowBattery,
		Full
	}battery_state_;

	// State of load (max 8 pizzas)
	// Eight means it is fully loaded and underload means the pizza
	// loaded is below 8.
	enum Load{
		Eight,
		Underload,
		Empty
	};
	int is_moving_;

	float restaurant_x_pos_;
	float restaurant_y_pos_;
	float charge_station_x_pos_;
	float charge_station_y_pos_;

	float hanging_x_pos;
	float hanging_y_pos;

	// Check the pizza currently on the bot, it will be max 8.
	//Load CheckLoad();
	// Charge the battery in the charge station
	void Charge();

	// Check the remaining power in the robot battery
	Battery_state CheckPower();


	void LocCallBack(const delivery_platform::location::ConstPtr &location);
	void PositionMsgCallBack(const delivery_platform::preloadpos::ConstPtr &pos);
	// This is the call back function when loading pizza at restaturant.
	// It will be actived when the msg is sending to delivering topic
	// Then it will implement the logic of loading the pizza and
	// recording the coordinates of destination
	void LoadCallBack(const delivery_platform::delivering::ConstPtr &delivering);

};
#endif // TURTLEBOT_H
