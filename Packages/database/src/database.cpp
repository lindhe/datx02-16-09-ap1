//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDrive.h"
//Comment out when gulliview_server is compiled
//#include "gulliview_server/Pos.h"

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
    * Function for updating pointers to the closest points on the track.
    * Argument is on the form int[x,y,heading].
    * Updates the current points to aim for on the track
    * - global variables point1 and point2 - if the current points are reached
    * or "unreachable".
    */
    void updateIndicies(int* car_information){
        //Maybe add error handling if array is too small or too big.
        int car_coordinate_x, car_coordinate_y, car_heading;
        int track_vector[2], car_vector[2];
        double car_projection[2];
        
        car_coordinate_x = car_information[0];
        car_coordinate_y = car_information[1];
        car_heading = car_information[2];
        
        //Create vector from point 1 to point 2.
        track_vector[0] = track[point2][0] - track[point1][0];
        track_vector[1] = track[point2][1] - track[point1][1];
        
        //Create vector from point 1 to position of car.
        car_vector[0] = car_coordinate_x - track[point1][0];
        car_vector[1] = car_coordinate_y - track[point1][1];
        
        cout << "Track Vector: " << "[" << track_vector[0] << ","
                << track_vector[1] << "]" << '\n';
        cout << "Car Vector: " << "[" << car_vector[0] << ","
                << car_vector[1] << "]" << '\n';
        
        
        
        //Calculate orthogonal projection. If projected vector is
        //almost as long (marginal needs to be decided), the current points
        //are updated with the next points on the track.
        
    }
    
    /**
    * Function that projects one vector onto another and returns the
    * resulting vector.
    * Arguments: pointers to 2 vectors on the form; int[x,y].
    * vec2 is projected on to vec1. res_vec is the vector variable
    * that should store the result, also on form int[x,y].
    */
    
    //Tested and working!
    void orthogonalProjection(int* vec2, int* vec1, double* res_vec){
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
        
        cout << "Vector to scale: [" << vec1[0] << ","
                << vec1[1] << "]" << '\n';
        cout << "Result: " << result << '\n';
        
        res_vec[0] = (double)vec1[0] * result;
        res_vec[1] = (double)vec1[1] * result;
        
        cout << "Projected vector: [" << res_vec[0] << ","
                << res_vec[1] << "]" << '\n'; 
        
        return;
    }
    
    /**
    * Function for initializing track pointers
    * Argument: Position of the car
    * Initializes global variables point1 and point2 to the closest point to
    * the car and its predecessor.
    */
    
    //Tested and working.
    void initializeIndicies(int x1, int y1){
        double x2, y2, xdiff, ydiff, xdiff_pow, ydiff_pow, distance;
        point1 = 0;
        point2 = 0;
        point1_distance = numeric_limits<int>::max();
        point2_distance = numeric_limits<int>::max();
        
        int i = 0;
        int place = 0;
        while(((track[i][0] != 0) || (track [i][1] != 0)) &&
                (i < (sizeof(track)/8))){
            
            x2 = (double)track[i][0];
            y2 = (double)track[i][1];
            
            xdiff = x2 - (double)x1;
            ydiff = y2 - (double)y1;
            
            xdiff_pow = xdiff * xdiff;
            ydiff_pow = ydiff * ydiff;
            
            distance = sqrt(xdiff_pow + ydiff_pow);
            cout << "Distance between points: " << distance << '\n';
            
            if(distance < point1_distance){
                point1_distance = (int)distance;
                point1 = i;
                place = i;
            }
            i++;
        }
        int ape = 1;
        cout << "Size of an int in memory: " << sizeof(ape) << '\n';
        cout << "Size of array track: " << sizeof(track) << '\n';
        cout << "Number of columns in array track: " << sizeof(track)/8 << '\n';
        
        //Wrap around
        if((track[point1 + 1][0] == 0 && track[point1 + 1][1] == 0) ||
                point1 >= ((sizeof(track)/8) - 1)){
            
            point2 = 0;
            cout << "Hello!" << '\n';
        }
        else{
            point2 = place + 1;
        }
        cout << "Point1: " << "[" << track[point1][0] << ","
                << track[point1][1] << "]" << '\n';
        cout << "Point2: " << "[" << track[point2][0] << ","
                << track[point2][1] << "]" << '\n';
        
        return;
    }
    
    /**
    * Arguments on the form int[coordinate_x, coordinate_y].
    * Function calculates the shortest distance from the car to
    * a line drawn between the 2 points on the track.
    *
    * Returns the distance as a double;
    */
    
    //Tested and working. Need to fix so the distance can be negative
    //depending on the direction of the car. 
    double calculateDistance(int* first_point, int* next_point, int* car_point){
        //Maybe add exceptions for accessing arrays
        int vec1[2], vec2[2];
        double res_vec[2];
        int x1_begin, x1_end, y1_begin, y1_end;
        int x2_begin, x2_end, y2_begin, y2_end;
        
        //Create vectors, move them to origo and represent them as [x,y]
        x1_begin = first_point[0];
        x1_end = next_point[0];
        y1_begin = first_point[1];
        y1_end = next_point[1];
        
        x2_begin = first_point[0];
        x2_end = car_point[0];
        y2_begin = first_point[1];
        y2_end = car_point[1];
        
        //Vector 1
        vec1[0] = x1_end - x1_begin;
        vec1[1] = y1_end - y1_begin;
        
        cout << "Vector 1: [" << vec1[0] << "," << vec1[1] << "]" << '\n'; 
        
        //Vector 2
        vec2[0] = x2_end - x2_begin;
        vec2[1] = y2_end - y2_begin;
        
        cout << "Vector 2: [" << vec2[0] << "," << vec2[1] << "]" << '\n'; 
        
        //Throw exception if one of the vectors are 0 in lenght.
        if(vec1[0] == 0 && vec1[1] == 0){
            throw 666;
            return 0.0;
        }
        
        if(vec2[0] == 0 && vec2[0] == 0){
            return 0.0;
        }
        
        //Project Vector 2 onto Vector 1
        orthogonalProjection(&vec2[0], &vec1[0], &res_vec[0]);
        cout << "Resulting vector: " << "[" << res_vec[0] << ","
                << res_vec[1] << "]" << '\n';
        
        //Calculate distance
        double sum1, sum2, result2, distance, product3, product4;
        
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

/*    //Template to use the position from gulliview 
    void callback(const gulliview_server::Pos& msg){
        ROS_INFO("I heard: x = %lld", msg.x);
        ROS_INFO("I heard: y = %lld", msg.y);
        ROS_INFO("I heard: heading = %lld", msg.heading);
    }
*/
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
        
        ros::Publisher database_pub =
                n.advertise<std_msgs::Float64>("control_error", 1000);
        
        //Update the name of topic
        ros::Subscriber database_sub = n.subscribe("position",1000,callback);
        
        ros::Rate loop_rate(10);
        
        /**
        * A count of how many messages we have sent. This is used to create
        * a unique string for each message.
        */
        
        loadTrack();
        
        //Test new calculateDistance();
        int car_testvector[3] = {3,-5,0};
        
        initializeIndicies(car_testvector[0], car_testvector[1]);
        
        updateIndicies(&car_testvector[0]);
        
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
