#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

    /**
    * This tutorial demonstrates simple receipt of messages over the ROS system.
    */
    using namespace std;
    
    void callback(const std_msgs::Float64::ConstPtr& msg){
        ROS_INFO("I heard: [%.2f]", msg->data);
    }
    
    int main(int argc, char **argv)
        {
        
        ros::init(argc, argv, "listener");
        
        ros::NodeHandle n;
        
        ros::Subscriber sub = n.subscribe("control_error", 1000, callback);

        ros::spin();

        return 0;
    }
