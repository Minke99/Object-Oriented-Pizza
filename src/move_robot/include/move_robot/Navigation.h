#ifndef Navigation_H_
#define Navigation_H_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "delivery_platform/destination.h"

class CNavigation 
{
    public:
		CNavigation(ros::NodeHandle nh);
		~CNavigation();
		bool moveToGoal(double xGoal, double yGoal);

    private:
        ros::NodeHandle nh_;
		ros::Subscriber position_sub_;
        void DestinationMsgCallback(const delivery_platform::destination::ConstPtr &msg);

};

#endif // Navigation_H_
