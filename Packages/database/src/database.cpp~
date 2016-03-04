//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "srd_msgs/Float64.msg"
#include "ackermann_msgs/AckermannDrive.h"

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>

    /**
    * -------------------NOTE-------------------
    * Run this rosnode from workspace. Otherwise
    * the file data.txt won't be found.
    *
    */

    
    using namespace std;
    
    // [row - message][column - number]
    int lines[20][3];
    
    /**
    * Function for loading the checkpoints from a textfile.
    * Rows in Data.txt has the format:
    *   int:x int:y heading:int
    *   
    *   Note: heading in degrees. 
    */
    void loadTrack(string file){
        string line;
        ifstream datafile;
        datafile.open("src/database/src/data.txt",ifstream::in);
        int coordinate_x, coordinate_y, heading;
        int i = 0;
        
        if(datafile.is_open()){
            while(getline(datafile,line)){
                stringstream lineStream (line);
                
                //Get values from row
                lineStream >> coordinate_x >> coordinate_y >> heading;
                
                //For test
                cout << "Coordinate_x: " << x << '\n';
                cout << "Coordinate_y: " << y << '\n';
                cout << "Heading: " << heading << '\n';
                
                lines[i][0] = coordinate_x;
                lines[i][1] = coordinate_y;
                lines[i][2] = heading;
                
                cout << "Heading from array: " << lines[i][2] << '\n';
                
                i++;
            }
            datafile.close();
        }
        else cout << "Unable to open file";
        
        return;
    }

    int main(int argc, char **argv){

        /**
        * The ros::init() function needs to see argc and argv so that it can perform
        * any ROS arguments and name remapping that were provided at the command line.
        * For programmatic remappings you can use a different version of init() which takes
        * remappings directly, but for most command-line programs, passing argc and argv is
        * the easiest way to do it.  The third argument to init() is the name of the node.
        *
        * You must call one of the versions of ros::init() before using any other
        * part of the ROS system.
        */
        
        ros::init(argc, argv, "database_node");
        
        /**
        * NodeHandle is the main access point to communications with the ROS system.
        * The first NodeHandle constructed will fully initialize this node, and the last
        * NodeHandle destructed will close down the node.
        */
        
        ros::NodeHandle n;
        
        /**
        * The advertise() function is how you tell ROS that you want to
        * publish on a given topic name. This invokes a call to the ROS
        * master node, which keeps a registry of who is publishing and who
        * is subscribing. After this advertise() call is made, the master
        * node will notify anyone who is trying to subscribe to this topic name,
        * and they will in turn negotiate a peer-to-peer connection with this
        * node.  advertise() returns a Publisher object which allows you to
        * publish messages on that topic through a call to publish().  Once
        * all copies of the returned Publisher object are destroyed, the topic
        * will be automatically unadvertised.
        *
        * The second parameter to advertise() is the size of the message queue
        * used for publishing messages.  If messages are published more quickly
        * than we can send them, the number here specifies how many messages to
        * buffer up before throwing some away.
        */
        
        /**
        * Change topic when using controller
        */
        
        ros::Publisher database_pub = n.advertise<std_msgs::Float64>("control_error", 1000);
        
        ros::Rate loop_rate(10);
        
        /**
        * A count of how many messages we have sent. This is used to create
        * a unique string for each message.
        */
        
        loadTrack("data.txt");
        
        int count = 0;
    
        while (ros::ok()){
            /**
            * This is a message object. You stuff it with data, and then publish it.
            */
            
            if(count < 6) {
                std_msgs::Float64 msg;
                
                float fl;
                
                //Steering Angle
                fl = (float)lines[count][2];
                msg.data = fl;
                
                ROS_INFO("%.2f", msg.data);
                
                /**
                * The publish() function is how you send messages. The parameter
                * is the message object. The type of this object must agree with the type
                * given as a template parameter to the advertise<>() call, as was done
                in the constructor above.
                */
                database_pub.publish(msg);
                
                usleep(3000000);
            }
            
            ros::spinOnce();
            
            loop_rate.sleep();
            
            ++count;
            
            }
        
        return 0;
    }
