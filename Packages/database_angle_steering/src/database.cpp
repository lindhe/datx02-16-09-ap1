//Save this in: catkin_workspace/src
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "gulliview_server/Pos.h"

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <cmath>
#include <numeric>

using namespace std;

class DatabaseHandler{
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
        
    private: 
        /**Array for storing the track. [row - point][column - x, y]
        * ------------NOTE ------------
        * Initialized to 0. Change in case we need coordinates with zeros
        *
        */
        int track[320][2];
        
        /**
        *  Index in array "track" to the 2 closest points to the car.
        */
        int point1, point2;
        
        //Loop counter for callback
        int callback_loop = 0;
        
        //Error saved for publishing.
        //float car_error = 0.0;
        
        ros::Publisher database_pub;
        ros::Subscriber database_sub; 
        ros::NodeHandle n;
    public:        
        /*
         * Constructor to initialize the publisher/subscriber and load the track
         */
        DatabaseHandler(){
                database_pub = n.advertise<std_msgs::Float64>("path_error", 1000);
        
                database_sub = n.subscribe("position",1,&DatabaseHandler::callback, this);
                
                for(int i = 0; i < 319; i++){
                    track[i][0] = 0;
                    track[i][1] = 0;
                }
                
                loadTrack();
                
                
        }

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
        * Function returns TRUE if track_point is on the right side of the car,
        * false if left.
        * Arguments: int car_point[x,y] is the coordinates of the car.
        */
        
        //Not yet tested.
        bool isRight(int* car_point, int heading){
            int car_x, car_y, track_x, track_y, new_x, new_y;
            
            car_x = car_point[0];
            car_y = car_point[1];
            
            track_x = track[point2][0];
            track_y = track[point2][1];
            
            new_x = (track_x - car_x)*cos(((double)heading*3.1415)/180) +
                    (track_y - car_y)*sin(((double)heading*3.1415)/180);
                    
            new_y = -(track_x - car_x)*sin(((double)heading*3.1415)/180) +
                    (track_y - car_y)*cos(((double)heading*3.1415)/180);
                    
            if(new_y-1 > 0){
                return true;
            }else{
                return false;
            }
            
            
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
        double updateIndicies(int* car_information, int car_heading){
            //Maybe add error handling if array is too small or too big.
            int car_coordinate_x, car_coordinate_y;
            int track_vector[2], car_vector[2];
            double car_projection[2], track_double[2], car_point[2];
            double heading, wanted_heading;
            
            car_coordinate_x = car_information[0];
            car_coordinate_y = car_information[1];
            
            car_point[0] = (double)car_coordinate_x; 
            car_point[1] = (double)car_coordinate_y;
            heading = (double)car_heading;
            
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
            cout << "Orthogonal Projection Vector: [" << car_projection[0] <<
                "," << car_projection[1] << "]" << '\n';
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
            double length_track;
            int skip = 0;
            do{
                //Loop until a segment of the track in front of the car is chosen.
                //Do this only once.
                while((lenght_of_track_vector - length_of_projection_vector) <= 20 &&
                    ((track_double[0] * car_projection[0] > 0) ||
                    (track_double[1] * car_projection[1] > 0)) &&
                    !skip){
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
                    
                    //Create vector from point 1 to point 2.
                    track_vector[0] = track[point2][0] - track[point1][0];
                    track_vector[1] = track[point2][1] - track[point1][1];
                    
                    track_double[0] = (double)track_vector[0];
                    track_double[1] = (double)track_vector[1];
                            
                    //Create vector from point 1 to position of car.
                    car_vector[0] = car_coordinate_x - track[point1][0];
                    car_vector[1] = car_coordinate_y - track[point1][1];
                    
                    orthogonalProjection(&car_vector[0], &track_vector[0],
                                             &car_projection[0]);                     
                    
                    
                    lenght_of_track_vector = distanceBetweenPoints(&track_double[0],
                                                &origo[0]);
                    
                    length_of_projection_vector = distanceBetweenPoints(&car_projection[0],
                                                    &origo[0]);
                }
                if(skip == 1){
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
                }
                skip = 1;
                vector<double> ref_point(2);       
                vector<double> next_point(2);       
                vector<double> result_point(2);       
                ref_point[0] = 1;
                ref_point[1] = 0;
                next_point[0] = (double)track[point2][0] - car_point[0];
                next_point[1] = (double)track[point2][1] - car_point[1];
                length_track = sqrt(pow(next_point[0],2) + pow(next_point[1],2));
                result_point[0] = next_point[0]/length_track; 
                result_point[1] = next_point[1]/length_track;
                double dotproduct = inner_product(result_point.begin(),
                    result_point.end(), ref_point.begin(), 0.0);
                double angle = acos(dotproduct) * 180.0 / 3.14159265; 
                if(next_point[1] < 0){
                    angle = 360.0 - angle;
                }
                        
                cout << "Angle car_point: " << angle << endl;
                
                wanted_heading = angle - heading;
                
                if(abs((int)(angle-heading)) > 180){
                    if(angle > heading){
                        wanted_heading = wanted_heading - 360;
                    }
                    else{
                        wanted_heading = 360 + wanted_heading;
                    }
                }
            
                cout << "WANTED HEADING: " << wanted_heading << endl;
                cout << "LENGTH_TRACK: " << length_track << endl;
                
            }while((wanted_heading < -45 || wanted_heading > 45 ||
                    length_track < 550) &&
                    length_track < 2000);
            
            return wanted_heading;
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
            double track_point[2], origo_point[2], orth_proj[2];
            double track_point1[2], track_point2[2], track_double[2];
            double distance_to_car, origo_to_car, origo_to_track;
            double division, wanted_heading, projection_vector, track_length;
            float error;
            x = (int)msg.x;
            y = (int)msg.y;
            heading = (int)msg.heading;
            
            //If it is the first message received, initialize pointers to the track.
            if(callback_loop == 0){
                initializeIndicies(x,y);
            }
            
            //Create a vector from closest point on the track to the car,
            //and one from closest point on the track to the next point on
            //the track.
            car_vector[0] = x - track[point1][0];
            car_vector[1] = y - track[point1][1];
            
            track_vector[0] = track[point2][0] - track[point1][0];
            track_vector[1] = track[point2][1] - track[point1][1];
            
            track_double[0] = (double)track_vector[0];
            track_double[1] = (double)track_vector[1];
            
            track_point1[0] = (double)track[point1][0];
            track_point1[1] = (double)track[point1][1];
            
            track_point2[0] = (double)track[point2][0];
            track_point2[1] = (double)track[point2][2];
            //Update pointers to the track. If the new pointers are
            //still behind the car, update again.
            car_coordinates[0] = x;
            car_coordinates[1] = y;
            
            wanted_heading = updateIndicies(&car_coordinates[0], heading);
            
            cout << "Point 1: [" << track[point1][0] << "," << track[point1][1] << "]" << '\n';
            cout << "Point 2: [" << track[point2][0] << "," << track[point2][1] << "]" << '\n';
            
            //Calculate distance between the car and the track
            distance_to_car = calculateDistance(&car_vector[0], &track_vector[0]); 
            
            //Old code, uncomment if new does not work.
            
            //Check which side of the circular track the car is on
            /*
            track_point[0] = (double)track[point1][0];
            track_point[1] = (double)track[point1][1];
            
            orthogonalProjection(&car_vector[0], &track_vector[0], &orth_proj[0]);
            
            track_point[0] = track_point[0] + orth_proj[0];
            track_point[1] = track_point[1] + orth_proj[1];
            
            origo_point[0] = 0;
            origo_point[1] = 0;
            
            origo_to_car = distanceBetweenPoints(&car_point[0], &origo_point[0]);
            origo_to_track = distanceBetweenPoints(&track_point[0],
                                &origo_point[0]);
            */
            /* 
            //Calculate angle of vector pointing from car to next point
            track_point[0] = (double)track[point2][0] - car_point[0];
            track_point[1] = (double)track[point2][1] - car_point[1];
            
            division = track_point[1]/track_point[0];
            
            wanted_heading = atan(division)*180/3.141593;
            cout << "Angle car_point: " << wanted_heading << endl;

            if(car_point[1] > track_point[1]){
                wanted_heading = 360 - wanted_heading;
            }
            
            //Calculate if negative or positive angle
            //Negative if turning right, positive if left.
            if(wanted_heading > 180){
                wanted_heading = wanted_heading - 360;
            }
            
            */
            
            //car_error = (float) distance_to_car;
            
            std_msgs::Float64 path_error;

            path_error.data = (float)wanted_heading;

            ROS_INFO("Angle error: %.2f", path_error.data);

            database_pub.publish(path_error);

            callback_loop ++;
        }

};

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

    DatabaseHandler obj; 
    //        int count = 0;
    
    ros::spin();

    //        ++count;
    return 0;
}
