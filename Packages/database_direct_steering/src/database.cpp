//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
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
    float lines[20][5];
    
    /**
    * Function for loading the checkpoints from a textfile.
    */
    void loadTrack(string file){
        string line;
        ifstream datafile;
        datafile.open("src/database/src/data.txt",ifstream::in);
        float steering_angle, steering_angle_velocity;
        float speed, acceleration, jerk;
        int i = 0;
        
        if(datafile.is_open()){
            while(getline(datafile,line)){
                stringstream lineStream (line);
                
                //Get values from row
                lineStream >> steering_angle >> steering_angle_velocity;
                lineStream >> speed >> acceleration >> jerk;
                
                //For test
                cout << "Steering Angle: " << steering_angle << '\n';
                cout << "Steering Angle Velocity: " << steering_angle_velocity << '\n';
                cout << "Speed: " << speed << '\n';
                cout << "Acceleration: " << acceleration << '\n';
                cout << "Jerk: " << jerk << '\n';
                
                lines[i][0] = steering_angle;
                lines[i][1] = steering_angle_velocity;
                lines[i][2] = speed;
                lines[i][3] = acceleration;
                lines[i][4] = jerk;
                
                cout << "Speed from array: " << lines[i][2] << '\n';
                
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
        
        ros::Publisher database_pub = n.advertise<ackermann_msgs::AckermannDrive>("mc_cmds", 1000);
        
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
                ackermann_msgs::AckermannDrive msg;
                
                float fl;
                
                //Steering Angle
                fl = lines[count][0];
                msg.steering_angle = fl;
                
                //Steerng Angle Velocity
                fl = lines[count][1];
                msg.steering_angle_velocity = fl;
                
                //Speed
                fl = lines[count][2];
                msg.speed = fl;
                
                //Acceleration
                fl = lines[count][3];
                msg.acceleration = fl;
                
                //Jerk
                fl = lines[count][4];
                msg.jerk = fl;
                
                ROS_INFO("%.2f", msg.steering_angle);
                ROS_INFO("%.2f", msg.steering_angle_velocity);
                ROS_INFO("%.2f", msg.speed);
                ROS_INFO("%.2f", msg.acceleration);
                ROS_INFO("%.2f", msg.jerk);
                
                /**
                * The publish() function is how you send messages. The parameter
                * is the message object. The type of this object must agree with the type
                * given as a template parameter to the advertise<>() call, as was done
                in the constructor above.
                */
                database_pub.publish(msg);
                
                usleep(100000);
            }
            
            ros::spinOnce();
            
            loop_rate.sleep();
            
            ++count;
            
            }
        
        return 0;
    }
