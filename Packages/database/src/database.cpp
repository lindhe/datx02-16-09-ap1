//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDrive.h"

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <cmath>

    /**
    *
    * This node is a subscriber to the topic "camera_output", which takes
    * messages of type somethang:smth.
    *
    * This node is a publisher to the topic control_error, which takes
    * messages of type std_msgs:Float64.
    *
    *
    * -------------------NOTE-------------------
    * Run this rosnode from workspace. Otherwise
    * the file data.txt won't be found.
    *
    */
    
    using namespace std;
    
    /**Array for storing the track. [row - point][column - x, y]
    * ------------NOTE ------------
    * Initialized to 0. Change in case we need coordinates with zeros
    *
    */
    
    //------------------------Global-Variables---------------------
    
    int track[20][2] = {0};
    
    /**
    *  Index in array "track" to the 2 closest points to the car.
    */
    int point1, point2, point1_distance, point2_distance;
    
    //Coordinates for position and heading of car
    int car_x, car_y, car_heading;
    
    //-------------------------------End---------------------------
    
    
    
    /**
    * Function for initializing track pointers
    * Argument: Position of the car
    * Returns: Array of indicies to coordinates in track. 
    * [index_point1, index_point2]
    */
    int* initializeIndicies(int x1, int y1){
        int x2, y2, xdiff, ydiff, xdiff_pow, ydiff_pow, distance;
        point1 = 0;
        point2 = 0;
        point1_distance = numeric_limits<int>::max();
        point2_distance = numeric_limits<int>::max();
        
        int i = 0;
        while(track[i][0] != 0 && i < sizeof(track)){
            x2 = track[i][0];
            y2 = track[i][1];
            
            xdiff = x2 - x1;
            ydiff = y2 - y1;
            
            xdiff_pow = xdiff * xdiff;
            ydiff_pow = ydiff * ydiff;
            
            distance = (int)sqrt((double)(xdiff_pow + ydiff_pow));
            
            if(distance < point1_distance){
                point2_distance = point1_distance;
                point2 = point1;
                
                point1_distance = distance;
                point1 = i;
            }
            else if(distance < point2_distance){
                point2_distance = distance;
                point2 = i;
            }
            i++;
        }
    }
    
    /**
    * Function for loading the checkpoints from a textfile.
    * Rows in Data.txt has the format:
    *   int:x int:y
    */
    void loadTrack(){
        string line;
        ifstream datafile;
        datafile.open("src/database/src/data.txt",ifstream::in);
        int x, y;
        int i = 0;
        
        if(datafile.is_open()){
            while(getline(datafile,line)){
                stringstream trackstream (line);
                
                //Get values from row
                trackstream >> x >> y;
                
                //For test
                cout << "X: " << x << '\n';
                cout << "Y: " << y << '\n';
                
                track[i][0] = x;
                track[i][1] = y;
                
                i++;
            }
            datafile.close();
        }
        else cout << "Unable to open file";
        
        return;
    }

    void callback(const std_msgs::Float64::ConstPtr& msg){
        ROS_INFO("I heard: [%.2f]", msg->data);
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
        
        //Update the name of topic
        ros::Subscriber database_sub = n.subscribe("name_of_topic",1000,callback);
        
        ros::Rate loop_rate(10);
        
        /**
        * A count of how many messages we have sent. This is used to create
        * a unique string for each message.
        */
        
        loadTrack();
        
        int count = 0;
    
        while (ros::ok()){
            /**
            * This is a message object. You stuff it with data, and then publish it.
            */
            
            if(count < 6) {
                std_msgs::Float64 msg;
                
                float fl;
                
                //Steering Angle
                fl = (float)track[count][2];
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
