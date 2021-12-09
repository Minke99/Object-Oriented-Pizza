#include "delivery_platform/TurtleBot.h"


// Turtlebot Constructor
// Define nodeHandles & initialise variables 
CTurtleBot::CTurtleBot(ros::NodeHandle nh)
	: nh_(nh)
{
	// Print a statement to let the user know that the turtlebot simualation
	// node is initalised
	ROS_INFO("[Robot]: TurtleBot Initialized");

	// Initialize the load pointer to zero.
	loaded_pizza_.load_ptr_ = 0;

	// Initialize not moving.
	is_moving_ = 0;

	// Initialize the battery to be full.
	battery_ = full_battery_value_;

	// Turtlebot has 3 sensors, Lidar, Odom and camera, initialise
	bot_camera_ = new CCamera(nh);

	delivering_sub_= nh_.subscribe("Delivering", 10, &CTurtleBot::LoadCallBack, this);

    // Subscribe to preloadpos topic
    preload_sub_ = nh_.subscribe("Preloadpos",10, &CTurtleBot::PositionMsgCallBack, this);

	// Subscribe to location
    location_sub_ = nh_.subscribe("Location",10, &CTurtleBot::LocCallBack, this);

	// TurtleBot node will have 2 published topic, Delivered and Destination.
	// Delivered is used to communication with the House that the robot reaches
	// the house, and want to send the corresponding number of pizza to the house.
	delivered_pub_ = nh_.advertise<delivery_platform::delivered>("Delivered", 10);

	// Destination is used to tell the nav system where to go next. The message
	// will be received by move_robot node to implement the navigation.
	destination_pub_ = nh_.advertise<delivery_platform::destination>("Destination", 10);

}


// Turtlebot destructor 
CTurtleBot::~CTurtleBot()
{
	// Delete the memory
	delete bot_camera_;

	// Shut down the ros
	ros::shutdown();
}

// Callback from the camera. So that robot will know is it at a house or a charge station.
// At house or station, the robot should actively send pizza or charge, 
// so it needs to know these two kind of places to do an action.
void CTurtleBot::LocCallBack(const delivery_platform::location::ConstPtr &location)
{
	
	// If it is at a house, then send the ordered pizza to the house.
	if(location->where == "house")
	{
		// Send number of pizzas
		SendPizza(location->num);
	}

	// If it is at the charge station, then charge.
	if(location->where == "charge_station")
	{
		Charge();
	}
	
}

// The preload system subscribe to the topic and get the information of coordinates of buildings in the map.
// For the bot, it only needs to know the position of restaurant and the charge station
void CTurtleBot::PositionMsgCallBack(const delivery_platform::preloadpos::ConstPtr &pos)
{
	// Store the position of restaurant
    restaurant_x_pos_ = pos->x_pos[8];
    restaurant_y_pos_ = pos->y_pos[8];

	// Store the position of charge station
	charge_station_x_pos_ = pos->x_pos[9];
    charge_station_y_pos_ = pos->y_pos[9];

	// Show info to user that the system is loaded succesfully.
    ROS_INFO("[Robot]: Knows restaurant is at x: %f, y: %f", restaurant_x_pos_, restaurant_y_pos_);
	ROS_INFO("[Robot]: Knows charge station is at x: %f, y: %f", charge_station_x_pos_, charge_station_y_pos_);
	
	// Publish the position of the restauart it goes to initially
	// So that where ever the turtlebot is generated, it will initially 
	// go to restaurant for the next command.
	delivery_platform::destination goto_restaurant;

	// The position is stored in the corresponding pos_array_.
	goto_restaurant.x_pos = restaurant_x_pos_;
	goto_restaurant.y_pos = restaurant_y_pos_;

	// Now the hanging position is the restaurant.
	hanging_x_pos = restaurant_x_pos_;
	hanging_y_pos = restaurant_y_pos_;

	// Publish the destination out, it will be subscribed by move_robot node.
	destination_pub_.publish(goto_restaurant);
}

bool CTurtleBot::LoadPizza(int num_pizza, float x_pos, float y_pos)
{
	ROS_INFO("[Robot]: Start loading");
	// Put the number of pizza in one order into one cell of pizza_array_
	loaded_pizza_.pizza_array_[loaded_pizza_.load_ptr_] = num_pizza;

	loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_] = x_pos;
	loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_] = y_pos;

	// Increment the pointer ready for the next load.
	loaded_pizza_.load_ptr_++;

	// Print the info to tell the user that how many pizzas is loaded.
	ROS_INFO("[Robot]: Load %d pizza(s)\n", num_pizza);

	return true;
}

void CTurtleBot::SendPizza(int house_num)
{
	// Since it is always navigating to the destination, thus the house must be
	// the house needs the top order.
	// Pub to the house and corresponding sending pizza. To update the info
	// on delivery platform.
	delivery_platform::delivered delivered_pizza;
	delivered_pizza.house_num = house_num;
	delivered_pizza.num_pizza = loaded_pizza_.pizza_array_[loaded_pizza_.load_ptr_];
	delivered_pub_.publish(delivered_pizza);

	// After send a pizza, the order is decreased by 1.
	// So the load_ptr_ should also be decreased.
	// This will points to a new pizza_array_ and pos_array_ for next order
	loaded_pizza_.load_ptr_--;

	// Publish the position of the house it wants to go.
	delivery_platform::destination pos;

	// If there is no remaining pizzas on the bot.
	// The bot needs to go to restaurant for the following tasks.
	if(loaded_pizza_.load_ptr_ < 0 )
	{
		loaded_pizza_.load_ptr_++;
		// Back to the restaurant
		pos.x_pos = restaurant_x_pos_;
		pos.y_pos = restaurant_y_pos_;

		// Now the hanging position is the restaurant.
		hanging_x_pos = restaurant_x_pos_;
		hanging_y_pos = restaurant_y_pos_;
	}
	// If there is remaining pizzas on the bot.
	// It needs to go to the next house to send pizza.
	else
	{
		ROS_INFO("[Robot]: Going to (%f, %f)\n", loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_], loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_]);
		// The position is stored in the corresponding pos_array_.
		pos.x_pos = loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_];
		pos.y_pos = loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_];

		// Now the hanging position is the next house.
		hanging_x_pos = loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_];
		hanging_y_pos = loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_];
	}

	// Publish the destination out, it will be subscribed by move_robot node.
	destination_pub_.publish(pos);
}


// Charge the battery, untill it is full
void CTurtleBot::Charge()
{
	// Declear a state variable to record the battery state.
	Battery_state state;
	// Check the power in the battery, store the result into the state.
	state = CheckPower();
	ROS_INFO("[Robot]: Start charging ------");

	// Use a loop to fast chage the battery
	// Assume it is very fast
	while(state != Full)
	{
		// Charging.
		battery_ += change_battery_;

		// Update the current battery power.
		state = CheckPower();
	}

	// Print info to terminal that the bot is fully charged.
	ROS_INFO("[Robot]: Fully charged!");

	// Going back to the hanging destination
	delivery_platform::destination pos;

	// After fully charged, continuous next work.
	// Currently, the destination of next work is stored in the hanging position.
	pos.x_pos = hanging_x_pos;
	pos.y_pos = hanging_y_pos;

	// If it is not restaurant, we need to increament the pointer.
	// Because it has been decreamented previously.
	if(hanging_x_pos != restaurant_x_pos_ && hanging_y_pos != restaurant_y_pos_)
	{
		loaded_pizza_.load_ptr_++;
	}
	ROS_INFO("[Robot]: Going to back to hanging coordinate (%f, %f)\n", hanging_x_pos, hanging_y_pos);
	
	// Publish the destination out, it will be subscribed by move_robot node.
	destination_pub_.publish(pos);

}

// Consume the power while the robot is working 
void CTurtleBot::ConsumePower()
{
	if(battery_state_ == LowBattery)
	{
		// Going to charge station
		delivery_platform::destination pos;

		// The position is stored in the corresponding pos_array_.
		pos.x_pos = charge_station_x_pos_;
		pos.y_pos = charge_station_y_pos_;
		ROS_INFO("[Robot]: Need to go to charge station (%f, %f)\n", charge_station_x_pos_, charge_station_x_pos_);
		
		// Publish the destination out, it will be subscribed by move_robot node.
		destination_pub_.publish(pos);
		
	}
	// Declear a state variable to record the battery state.
	CTurtleBot::Battery_state state;

	// Power consumed while working.
	battery_ -= change_battery_;
	ROS_INFO("[Robot]: current power %f", battery_);

	// Check the remaining power, update it.
	battery_state_ = CheckPower();

}

// Check the remaining power in the battery
CTurtleBot::Battery_state CTurtleBot::CheckPower()
{
	// Declear a state variable to record the battery state.
	CTurtleBot::Battery_state state;

	// Judge the remaining power in the battery.
	// There will be 3 states describing the battery.
	// LowBattery, Full and Normal.
	// They are defined by const threshold defined previously.
	if(battery_ < low_battery_threshold_)
	{
		state = LowBattery;
	}
	// Full power
	else if(battery_ >= full_battery_value_)
	{
		state = Full;
	}
	// Not too low and not full.
	else
	{
		state = Normal;
	}

	// Return the state out.
	return state;
}

// Callback function when receiving pizza from restaurant 
void CTurtleBot::LoadCallBack(const delivery_platform::delivering::ConstPtr &delivering)
{
	// ROS_INFO("[Robot]: Get info from restaurant.");
	// If it is not 0, means here are still pizzas to be load.
	if(delivering->num_pizza_loaded != 0)
	{
		is_moving_ = 0;

		LoadPizza(delivering->num_pizza_loaded, delivering->x_pos, delivering->y_pos);

		// If it is not fully loaded, tell the user how many pizzas are there.
		// ROS_INFO("[Robot]: Loading -- %d orders\n", loaded_pizza_.load_ptr_);
	}
	// If number of pizza loaded is 0 and also robot is not moving, 
	// then robot will leave the restaurant and start going to the first destinaiton.
	// The first destination will be the top order in the load array.
	else if(is_moving_ == 0)
	{
		// Tell the user that it can start sending now.
		ROS_INFO("[Robot]: No more order, starting delivering now!\n");
		
		// Decrease the pointer by one.
		// Now points to the newest order that needs to be sent.
		loaded_pizza_.load_ptr_--;

		// Publish the position of the house it wants to go.
		delivery_platform::destination pos;

		// The position is stored in the corresponding pos_array_.
		pos.x_pos = loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_];
		pos.y_pos = loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_];
		ROS_INFO("[Robot]: Going to (%f, %f)\n", loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_], loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_]);

		// Currently, the destination of first house is stored in the hanging position.
		hanging_x_pos = loaded_pizza_.x_pos_array_[loaded_pizza_.load_ptr_];
		hanging_y_pos = loaded_pizza_.y_pos_array_[loaded_pizza_.load_ptr_];
		
		// Publish the destination out, it will be subscribed by move_robot node.
		destination_pub_.publish(pos);

		// Now it is moving.
		is_moving_ = 1;
	
	}
}
