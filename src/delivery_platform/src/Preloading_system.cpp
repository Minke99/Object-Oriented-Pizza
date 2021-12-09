
#include "delivery_platform/Preloading_system.h"
#include "delivery_platform/preloadpos.h"

// Constructor for house node
CPreload::CPreload(ros::NodeHandle nh)
    //Initialise private variable and store node handle
    :nh_(nh)
{
    ROS_INFO("[System]: Create Preload System");

    // Initialise publishers for orders to restaurant
    // Tell the restaurant the amount of pizzas and xy co-ords of house
    preload_pub_ = nh_.advertise<delivery_platform::preloadpos>("Preloadpos",10);

    PreLoadPos();
}

// Destructor for house node
CPreload::~CPreload()
{
    // Preload system is deleted.
    ROS_INFO("[System]: System is shuting down\n");
}

void CPreload::PreLoadPos()
{
    // Ask for user input the file path that contains the x y coordinates.
	std::string path;
	std::cout<<"[System]: Enter which path of the coordinate file: ";
	std::cin>>path;
    delivery_platform::preloadpos positions;

    // Variable used to read information from the file
    float num;
	int index = 0;

    // Read the xCoordinatePair.txt file and get the number in each line.
    std::cout << "[System]: Loading the x file... \n" << std::endl;
    std::ifstream InFilex(path + "xCoordinatePair.txt");
    while(InFilex.good())
    {   
        // Store the number in num variable
        InFilex>>num;

        // Store the x coordinate in the x_pos array with the index
		positions.x_pos[index] = num;

        // Increase the index.
		index++;
    }
    // Close the file
    InFilex.close();

    // Tells the x coordinates are loaded
    std::cout << "[System]: x coordinates loaded... \n" << std::endl;
	
    // Put the index back to 0.
	index = 0;

    // Read the yCoordinatePair.txt file and get the number in each line.
    std::cout << "[System]: Loading the y file... \n" << std::endl;
    std::ifstream InFiley(path + "yCoordinatePair.txt");
    while(InFiley.good())
    {   
        // Store the number in num variable
        InFiley>>num;

        // Store the x coordinate in the y_pos array with the index
		positions.y_pos[index] = num;

        // Increase the index.
		index++;
    }
    // Close the file
    InFiley.close();
    // Tells the y coordinates are loaded.
    std::cout << "[System]: y coordinates loaded... \n" << std::endl;

    // Pub the read coordinates out.
    preload_pub_.publish(positions);

}
