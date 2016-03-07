//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>

    /**
    * This tutorial demonstrates simple sending of messages over the ROS system.
    */
    using namespace std;
    
    int main(int argc, char **argv){
        
        ros::init(argc, argv, "gv_fake");
        
        ros::NodeHandle n;
        
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("car_coordinates", 1000);
        
        ros::Rate loop_rate(10);
        
        int count = 0;
    
        while (ros::ok()){
            
            std_msgs::String msg;
            
            std::stringstream ss;
            
            ss << "hello world " << count;
            
            msg.data = ss.str();
            
            ROS_INFO("%s", msg.data.c_str());
            
            chatter_pub.publish(msg);
            
            ros::spinOnce();
            
            loop_rate.sleep();
            
            ++count;
            
            }
        
        return 0;
    }
