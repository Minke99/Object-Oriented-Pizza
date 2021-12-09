
#include "delivery_platform/Restaurant.h"

// Constructor for restaurant
CRestaurant::CRestaurant(ros::NodeHandle nh)
    //Initialise private variable and store node handle
    :nh_(nh)
{
    ROS_INFO("[Restaurant]: Restaurant Init");
    load_once_ = 0;
    // Subscribe to preloadpos topic
    //preload_sub_ = nh_.subscribe("Preloadpos",10, &CRestaurant::PreloadCoordinateCallBack, this);

    // Subscribe to location topic
    location_sub_ = nh_.subscribe("Location",10, &CRestaurant::LocCallBack, this);

    // Subscribe to order topic
    order_sub_ = nh_.subscribe("Order",10, &CRestaurant::OrderMsgCallBack, this);

    // Initialise publishers for delivering
    // Tells robot number of pizzas loaded and destination of house 
    delivering_pub_ = nh_.advertise<delivery_platform::delivering>("Delivering",10);
}

// Destructor for restaurant
CRestaurant::~CRestaurant()
{
    // Tells user the restaurant is deleted.
    ROS_INFO("[Restaurant]: Restaurant destroyed\n");
}

void CRestaurant::PositionMsgCallBack(const delivery_platform::preloadpos::ConstPtr &pos)
{
    // &pos contains the coordinated of buildings on the map.
    // Firstly the restaurant needs to store coordinates of houses.
    // So that when house send order to restaurant, restaurant knows the 
    // exact coordinates of the house. Since client usually knows which house
    // which number and which street is he at, not the x y coordinates.
    for(int i = 0; i < num_house_; i++)
    {
        // Store x y coordniates as a array.
        house_x_pos_[i] = pos->x_pos[i];
        house_y_pos_[i] = pos->y_pos[i];
        ROS_INFO("[Restaurant]: House %d position loaded", i);
    }
    // Also restaurant needs to know the coordnate itself.
    restaurant_x_pos_ = pos->x_pos[at_restaurant_];
    restaurant_y_pos_ = pos->y_pos[at_restaurant_];
    ROS_INFO("[Restaurant]: Position loaded x: %f, y: %f", restaurant_x_pos_, restaurant_y_pos_);
}

// Receive order
void CRestaurant::OrderMsgCallBack(const a_user_portal::order::ConstPtr &order)
{
    // Store information from order callback in private variables
	order_array.pizza_array_[order_array.load_ptr_] = order->num_pizza_wanted;
	
    // Get the house number
	int which_house = order->which_house;
	
    // Find the coordinates of the house and store then in the order array.
    order_array.x_pos_array_[order_array.load_ptr_] = house_x_pos_[which_house];
    order_array.y_pos_array_[order_array.load_ptr_] = house_y_pos_[which_house];

	// Increment the pointer ready for the next load.
	order_array.load_ptr_++;

    // Output info to let user know.
    ROS_INFO("[Restaurant]: Reveived an order");

    // If the number of order saved at restaurant exceeds 100, restaurant will
    // not accept any new orders anymore. It is too much !!!!!!
	if(order_array.load_ptr_ > max_order_)
	{
		ROS_INFO("[Restaurant]: Cannot receive orders anymore!!! \n");
	}
    else
    {
        // Bake pizza (minimise time taken in simulation)
        ROS_INFO("[Restaurant]: Baking pizzas");
        ROS_INFO("[Restaurant]: Finished Baking\n");
    }

}


// Callback from the camera. So that restaurant will know whether the robot reaches the restaurant.
void CRestaurant::LocCallBack(const delivery_platform::location::ConstPtr &location)
{

    // If the bot is at the restaurant
    if (location->where == "restaurant")
    {
        // If the pizza has been baked, load the bot
        if (order_array.load_ptr_ != 0)
        {
            ROS_INFO("[Restaurant]: Loading Order to Bot");

            // Send order to bot
            send_status_ = SendOrder();
        }
        else
        {
            // Bot is at restaurant but no pizza is made
            ROS_INFO("[Restaurant]: Order Not Ready to be Loaded\n");
        }

        // If the order has been sent, rest the send status and the bake status
        if (send_status_ == order_sent_)
        {
            ROS_INFO("[Restaurant]: Order Sent to Bot\n");
            // Robot is still not start sending.
            send_status_ = 0;
        }
    }
    else
    {
        // If the bot is not at the restaurant, wait for the bot to arrive
        // Clear the load_once_ variable so that when robot comes to restaurant
        // next time, it can be counted from 0. 
        load_once_ = 0;
    }

}


// Send Pizza order to bot
int CRestaurant::SendOrder()
{
    // A varable for return msg.
    int result = 0;

    // Count the total pizza loaded on to the bot from bot reaches restaurant
    load_once_ += order_array.pizza_array_[order_array.load_ptr_ - 1];

    // If the loaded pizza is less than 8, it is possible to load more
    // depends the number of pizza next order.
    if(load_once_ <= 8)
    {
        // Indicates that it still possible to load
        ROS_INFO("Okay to load");

        // Then decrease the pointer to implement the real loading process
        order_array.load_ptr_--;

        // Publish order details
        delivery_platform::delivering load_pizza;

        // Number of pizza to order
        load_pizza.num_pizza_loaded = order_array.pizza_array_[order_array.load_ptr_];

        // Position of order
        load_pizza.x_pos = order_array.x_pos_array_[order_array.load_ptr_];
        load_pizza.y_pos = order_array.y_pos_array_[order_array.load_ptr_];

        // Indicates that the pizza is loaded and pub this message to bot
        ROS_INFO("Loaded %d pizzas onto robot.\n", order_array.pizza_array_[order_array.load_ptr_]);
        delivering_pub_.publish(load_pizza);

        // Returns the success
        result = order_sent_;

        // If carried pizzas is already 8, stop load to bot.
        if(load_once_ == 8)
        {
            // Pub a number 0 to bot to tell bot that it can start sending now!
            delivery_platform::delivering goto_house;
            goto_house.num_pizza_loaded = 0;
            delivering_pub_.publish(goto_house);
            ROS_INFO("8 pizzas now, stop loading\n");
        }
    }
    // If it will exceed 8 after this time of loading, then avoid it.
    else
    {
        // Pub a number 0 to bot to tell bot that it can start sending now!
        delivery_platform::delivering goto_house;
        goto_house.num_pizza_loaded = 0;
        delivering_pub_.publish(goto_house);
        ROS_INFO("Can be max 8 pizzas, stop loading\n");
    }
    return result;
}

