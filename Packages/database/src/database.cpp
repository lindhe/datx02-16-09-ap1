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
    * No blank rows in database file,
    * no newline at end of file.
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
    * TODO: Write function for calculating the distance from
    * the car to the 2 closest checkpoints, and switching pointers
    * to next checkpoint when one is passed.
    */
    
    /**
    * Function for initializing track pointers
    * Argument: Position of the car
    * Returns: Array of indicies to coordinates in track. 
    * [index_point1, index_point2]
    */
    
    //Tested and working.
    void initializeIndicies(int x1, int y1){
        double x2, y2, xdiff, ydiff, xdiff_pow, ydiff_pow, distance;
        point1 = 0;
        point2 = 0;
        point1_distance = numeric_limits<int>::max();
        point2_distance = numeric_limits<int>::max();
        
        int i = 0;
        while(((track[i][0] != 0) || (track [i][1] != 0)) && (i < sizeof(track))){
            x2 = (double)track[i][0];
            y2 = (double)track[i][1];
            
            xdiff = x2 - (double)x1;
            ydiff = y2 - (double)y1;
            
            xdiff_pow = xdiff * xdiff;
            ydiff_pow = ydiff * ydiff;
            
            distance = sqrt(xdiff_pow + ydiff_pow);
            cout << "Distance between points: " << distance << '\n';
            
            if(distance < point1_distance){
                point2_distance = point1_distance;
                point2 = point1;
                
                point1_distance = (int)distance;
                point1 = i;
            }
            else if(distance < point2_distance || point1 == point2){
                point2_distance = (int)distance;
                point2 = i;
            }
            i++;
        }
        return;
    }
    
    /**
    * Arguments (vectors) on the form int[x_begin, x_end, y_begin, y_end].
    * Function projects vec2_first onto vec1_first.
    * Returns pointer to the resulting vector, int[x_res, y_res]
    */
    
    //Tested and working. Need to fix so the distance can be negative
    //depending on the direction of the car. 
    double calculateDistance(int* vec2_first, int* vec1_first){
        //Maybe add exceptions for accessing arrays
        int vec1[2], vec2[2];
        double res_vec[2];
        int x1_begin, x1_end, y1_begin, y1_end;
        int x2_begin, x2_end, y2_begin, y2_end;
        
        //Move vectors to origo and represent them as [x,y]
        x1_begin = vec1_first[0];
        x1_end = vec1_first[1];
        y1_begin = vec1_first[2];
        y1_end = vec1_first[3];
        
        x2_begin = vec2_first[0];
        x2_end = vec2_first[1];
        y2_begin = vec2_first[2];
        y2_end = vec2_first[3];
        
        //Vector 1
        vec1[0] = x1_end - x1_begin;
        vec1[1] = y1_end - y1_begin;
        
        cout << "Vector 1: [" << vec1[0] << "," << vec1[1] << "]" << '\n'; 
        
        //Vector 2
        vec2[0] = x2_end - x2_begin;
        vec2[1] = y2_end - y2_begin;
        
        cout << "Vector 2: [" << vec2[0] << "," << vec2[1] << "]" << '\n'; 
        
        if((vec1[0] == 0 && vec1[1] == 0) || (vec2[0] == 0 && vec2[1] == 0)){
            throw 666;
            return 0.0;
        }
        
        //Project Vector 2 onto Vector 1
        int numerator, denominator, product1, product2;
        double numerator2, denominator2, result, product3, product4;
        
        product1 = vec2[0] * vec1[0];
        product2 = vec2[1] * vec1[1];
        numerator = product1 + product2;
        
        cout << "Numerator: " << numerator << '\n';
        
        product1 = vec1[0] * vec1[0];
        product2 = vec1[1] * vec1[1];
        denominator = product1 + product2;
        
        cout << "Denominator: " << denominator << '\n';
        
        numerator2 = (double)numerator;
        denominator2 = (double)denominator;
        
        result = numerator2 / denominator2;
        
        cout << "Vector to scale: [" << vec1[0] << "," << vec1[1] << "]" << '\n';
        cout << "Result: " << result << '\n';
        
        res_vec[0] = (double)vec1[0] * result;
        res_vec[1] = (double)vec1[1] * result;
        
        cout << "Projected vector: [" << res_vec[0] << "," << res_vec[1] << "]" << '\n'; 
        
        //Calculate distance
        double sum1, sum2, result2, distance;
        
        sum1 = vec2[0] - res_vec[0];
        sum2 = vec2[1] - res_vec[1];
        
        product3 = sum1 * sum1;
        product4 = sum2 * sum2;
        
        result2 = sqrt(product3 + product4);
        
        return result2;
    }
    
    /**
    * Function for loading the checkpoints from a textfile.
    * Rows in Data.txt has the format:
    *   int:x int:y
    */
    //Tested and working
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
            if(count < 6) {
                std_msgs::Float64 msg;
                
                float fl;
                
                //Steering Angle
                fl = (float)track[count][1];
                msg.data = fl;
                
                ROS_INFO("%.2f", msg.data);
                
                database_pub.publish(msg);
                
                usleep(3000000);
            }
            
            ros::spinOnce();
            
            loop_rate.sleep();
            
            ++count;
            
            }
        
        return 0;
    }
