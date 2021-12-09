// Preloading_system.h

#ifndef PRELOAD_H_
#define PRELOAD_H_

#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fstream>    // file input
#include "delivery_platform/preloadpos.h"

class CPreload
{
    public:

    // Constructor for house node
    CPreload(ros::NodeHandle nh);

    // Destructor
    ~CPreload();


    private:

    // ROS NodeHandle
    ros::NodeHandle nh_;

    // ROS publisher for position
    ros::Publisher preload_pub_;

	void PreLoadPos();
};


#endif
