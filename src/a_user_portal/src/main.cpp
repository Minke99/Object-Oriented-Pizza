
#include "a_user_portal/User.h"


// Main loop to let the turtle bot always follow the left wall and solve the maze
// It creates and initialises the variables for the turtle bot
// This main file will loop indefinitely until the program is stopped by the user 
int main(int argc, char* argv[])
{
	// Initialise the function 
	ros::init(argc, argv, "User");

	// Create a nodehandle
	ros::NodeHandle nh;

	CUser user(nh);

	// Loop rate = 50
	// We set the loop rate to 50 as we found that at this rate of 50 loops
	// a second is a proper rate to achieve the task 
	ros::Rate loop_rate(1);
	
	//Run the while loop indefinitely until program is terminated by the user
	while (ros::ok())
	{

		// Correct the loop frequency
		// Ensure the loop rate is corrected to 1
		ros::spinOnce();
		loop_rate.sleep();
		user.UserInput();
	}

	return 0;
}

 
