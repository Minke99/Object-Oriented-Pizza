#include "ros/ros.h"
#include "a_user_portal/User.h"

// Need to create inital orders from main. Create a main to read user input std, and 
// publish from main.cpp to house node with pizza amount and house location
// also need to create a callback to tell the robot order has been picked up by client

// Constructor for house node
CUser::CUser(ros::NodeHandle nh)
    //Initialise private variable and store node handle
    :nh_(nh)
{
    ROS_INFO("[User]: Create User Node");

    // Initialise publishers for orders to restaurant
    // Tell the restaurant the amount of pizzas and xy co-ords of house
    order_pub_ = nh_.advertise<a_user_portal::order>("Order",10);
}

// Destructor for house node
CUser::~CUser()
{
    ROS_INFO("[User]: User Node Destroyed\n");
}

//Send order function called in the callback from messege from cpp
void CUser::UserInput()
{
	int house_num;
	int num_pizza;
	std::cout<<"[Order]: Enter which house are you in: ";
	std::cin>>house_num;
	std::cout<<"[Order]: Enter how many pizza(s) do you want: ";
	std::cin>>num_pizza;

    // Publish order details
    a_user_portal::order order;

    // Number of pizza to order
    order.num_pizza_wanted = num_pizza;
	order.which_house = house_num;

    // Position of order
    order_pub_.publish(order);
}




