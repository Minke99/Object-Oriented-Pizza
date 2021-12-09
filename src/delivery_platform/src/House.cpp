
#include "delivery_platform/House.h"

// Constructor for house node
CHouse::CHouse(ros::NodeHandle nh)
    //Initialise private variable and store node handle
    :nh_(nh)
{
    ROS_INFO("[House]: Create House Node");

    // Subscribe to delivered  to ensure the client knows when the pizza is recieved
    delivered_sub_ = nh_.subscribe("Delivered",10, &CHouse::DeliveredMsgCallBack, this);

}

// Destructor for house node
CHouse::~CHouse()
{
    // Output info that the house is deleted
    ROS_INFO("[House]: House Node Destroyed\n");
}


// Receive the order on by using simulated house.
// So that this infomation will show on platform terminal
// Clients can know the order is delivered when seeing this msg
void CHouse::DeliveredMsgCallBack(const delivery_platform::delivered::ConstPtr &order_delivered)
{
    // Store information from order callback in private variables
	int num_pizza;
    num_pizza = order_delivered->num_pizza;

    // Print the order that it recieved
    ROS_INFO("[House]: Recieved %d Pizzas\n",num_pizza);
}

