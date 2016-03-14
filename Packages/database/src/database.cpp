//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDrive.h"
//Comment out when gulliview_server is compiled
#include "gulliview_server/Pos.h"

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
    int point1, point2;
    
    //Loop counter for callback
    int callback_loop = 0;
    
    //Error saved for publishing.
    float car_error = 0.0;
    
    //-------------------------------End---------------------------
    
    /**
    * Function that projects one vector onto another and returns the
    * resulting vector.
    * Arguments: pointers to 2 vectors on the form &int[x,y].
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
        
        product1 = vec1[0] * vec1[0];
        product2 = vec1[1] * vec1[1];
        denominator = product1 + product2;
        
        numerator2 = (double)numerator;
        denominator2 = (double)denominator;
        
        result = numerator2 / denominator2;
        
        res_vec[0] = (double)vec1[0] * result;
        res_vec[1] = (double)vec1[1] * result;
        
        return;
    }
    
    /**
    * Function for calculating the distance between two points.
    * Arguments on form &double[x,y].
    */
    //Tested and Working!
    double distanceBetweenPoints(double* first_point, double* second_point){
        double first_point_x, first_point_y, second_point_x, second_point_y;
        double sum1, sum2, product1, product2, result;
        
        first_point_x = first_point[0];
        first_point_y = first_point[1];
        second_point_x = second_point[0];
        second_point_y = second_point[1];
        
        sum1 = first_point_x - second_point_x;
        sum2 = first_point_y - second_point_y;
        
        product1 = sum1 * sum1;
        product2 = sum2 * sum2;
        
        result = sqrt(product1 + product2);
        
        return result;
    }  
    
    /**
    * Arguments on the form &int[x,y]. vec1 is the track vector,
    * and vec2 is the car vector. 
    * Function calculates the shortest distance from the car to
    * the track vector.
    *
    * Returns the distance as a double;
    */
    
    //Tested and Working!
    double calculateDistance(int* vec2, int* vec1){
        //Maybe add exceptions for accessing arrays
        double res_vec[2];
        
        //Throw exception if track vector is 0 in lenght.
        if(vec1[0] == 0 && vec1[1] == 0){
            throw 666;
            return 0.0;
        }
        
        //Project Vector 2 onto Vector 1
        orthogonalProjection(&vec2[0], &vec1[0], &res_vec[0]);
        
        //Calculate distance
        double result;
        double car_vec[2];
        car_vec[0] = (double)vec2[0];
        car_vec[1] = (double)vec2[1];
        //Here is this error, rethink.
        result = distanceBetweenPoints(&car_vec[0], &res_vec[0]);
        
        return result;
    }
    
    /**
    * Function for updating pointers to the closest points on the track.
    * Argument is on the form int[x,y].
    * Updates the current points to aim for on the track
    * - global variables point1 and point2 - if the current points are reached
    * or "unreachable".
    */
    
    //Tested and working!
    void updateIndicies(int* car_information){
        //Maybe add error handling if array is too small or too big.
        int car_coordinate_x, car_coordinate_y;
        int track_vector[2], car_vector[2];
        double car_projection[2], track_double[2];
        
        car_coordinate_x = car_information[0];
        car_coordinate_y = car_information[1];
        
        //Create vector from point 1 to point 2.
        track_vector[0] = track[point2][0] - track[point1][0];
        track_vector[1] = track[point2][1] - track[point1][1];
        
        track_double[0] = (double)track_vector[0];
        track_double[1] = (double)track_vector[1];
                
        //Create vector from point 1 to position of car.
        car_vector[0] = car_coordinate_x - track[point1][0];
        car_vector[1] = car_coordinate_y - track[point1][1];
        
        //Calculate orthogonal projection. If projected vector is
        //almost as long (marginal needs to be decided), the current points
        //are updated with the next points on the track.
        orthogonalProjection(&car_vector[0], &track_vector[0],
                                &car_projection[0]);                     
        
        double lenght_of_track_vector;
        double length_of_projection_vector;
        double origo[2] = {0,0};
        
        lenght_of_track_vector = distanceBetweenPoints(&track_double[0],
                                    &origo[0]);
        
        length_of_projection_vector = distanceBetweenPoints(&car_projection[0],
                                        &origo[0]);
        
        //Change the value 1 to the margin we want to have to the next point.
        //Checks if the projection of the car is almost at the right point,
        //and if the projection vector is pointing in the same direction as
        //the track vector.
        
        if((lenght_of_track_vector - length_of_projection_vector) <= 1 &&
                ((track_double[0] * car_projection[0] > 0) ||
                track_double[1] * car_projection[1] > 0)){
            //Wrap around
            if((track[point2 + 1][0] == 0 && track[point2 + 1][1] == 0) ||
                point2 >= ((sizeof(track)/8) - 1)){
                point1 = point2;
                point2 = 0;
            }
            else if((track[point1 + 1][0] == 0 && track[point1 + 1][1] == 0) ||
                point1 >= ((sizeof(track)/8) - 1)){
                point1 = 0;
                point2 = 1;
            }
            else {
                point1 = point2;
                point2 = point2 + 1;
            }
        return;
        }
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
        int point1_distance, point2_distance;
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
            
            if(distance < point1_distance){
                point1_distance = (int)distance;
                point1 = i;
                place = i;
            }
            i++;
        }
        
        //Wrap around
        if((track[point1 + 1][0] == 0 && track[point1 + 1][1] == 0) ||
                point1 >= ((sizeof(track)/8) - 1)){
            
            point2 = 0;
        }
        else{
            point2 = place + 1;
        }
        return;
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
                
                track[i][0] = x;
                track[i][1] = y;
                
                i++;
            }
            datafile.close();
        }
        else cout << "Unable to open file";
        
        return;
    }

   //Template to use the position from gulliview 
   //Not yet tested.
    void callback(const gulliview_server::Pos& msg){
        ROS_INFO("I heard: x = %lld", (long long)msg.x);
        ROS_INFO("I heard: y = %lld", (long long)msg.y);
        ROS_INFO("I heard: heading = %lld", (long long)msg.heading);
        int x, y, heading;
        int car_coordinates[2], track_vector[2], car_vector[2];
        double car_point[2], track_point[2], origo_point[2];
        double distance_to_car, origo_to_car, origo_to_track;
        float error;
        x = (int)msg.x;
        y = (int)msg.y;
        heading = (int)msg.heading;
        
        //If it is the first message received, initialize pointers to the track.
        if(callback_loop == 0){
            initializeIndicies(x,y);
        }
        
        //Update pointers to the track;
        car_coordinates[0] = x;
        car_coordinates[1] = y;
        updateIndicies(&car_coordinates[0]);
        
        //Create a vector from closest point on the track to the car,
        //and one from closest point on the track to the next point on
        //the track.
        car_vector[0] = x - track[point1][0];
        car_vector[1] = y - track[point1][1];
        
        track_vector[0] = track[point2][0] - track[point1][0];
        track_vector[1] = track[point2][1] - track[point1][0];
        
        //Calculate distance between the car and the track
        distance_to_car = calculateDistance(&car_vector[0], &track_vector[0]);
        
        //Check which side of the circular track the car is on
        car_point[0] = (double)x;
        car_point[1] = (double)y;
        
        track_point[0] = (double)track[point1][0];
        track_point[1] = (double)track[point1][1];
        
        origo_point[0] = 0;
        origo_point[1] = 0;
        
        origo_to_car = distanceBetweenPoints(&car_point[0], &origo_point[0]);
        origo_to_track = distanceBetweenPoints(&track_point[0],
                            &origo_point[0]);
        //Calculate if negative or positive distance
        //Negative if right side of car, positive if left.
        if(origo_to_car > origo_to_track){
            distance_to_car = distance_to_car * -1;
        }
        
        car_error = (float)distance_to_car;
        
        callback_loop ++;
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

        int count = 0;
    
        while (ros::ok()){
            
            //Create message to send to PID-controller
            std_msgs::Float64 msg;
            
            msg.data = car_error;
            
            ROS_INFO("%.2f", msg.data);
            
            database_pub.publish(msg);
            
            ros::spinOnce();
            
            loop_rate.sleep();
            
            ++count;
            
            }
        
        return 0;
    }
