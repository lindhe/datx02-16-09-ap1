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

DatabaseHandler::DatabaseHandler(){
    database_pub = n.advertise<std_msgs::Float64>("path_error", 1000);


    database_sub = n.subscribe("position",1,&DatabaseHandler::callback, this);
    
    for(int i = 0; i < 319; i++){
        track[i][0] = 0;
        track[i][1] = 0;
    }
    
    loadTrack();  
}

void DatabaseHandler::orthogonalProjection(int* vec2, int* vec1,
        double* res_vec){
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

void DatabaseHandler::convertCoordinates(int* car_point, int heading,
        int* new_point){
    int car_x, car_y, track_x, track_y, new_x, new_y;
    
    car_x = car_point[0];
    car_y = car_point[1];
    
    track_x = track[point2][0];
    track_y = track[point2][1];
    
    new_x = (int)((track_x - car_x)*cos(((double)heading*3.1415)/180) +
            (track_y - car_y)*sin(((double)heading*3.1415)/180));
            
    new_y = (int)(-(track_x - car_x)*sin(((double)heading*3.1415)/180) +
            (track_y - car_y)*cos(((double)heading*3.1415)/180));                    
    
    cout << "New x: " << new_x << endl;
    cout << "New y: " << new_y << endl;
                
    new_point[0] = new_x;
    new_point[1] = new_y;
}

double DatabaseHandler::distanceBetweenPoints(double* first_point,
        double* second_point){
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


double DatabaseHandler::calculateDistance(int* vec2, int* vec1){
    double res_vec[2];
    
    //Throw exception if track vector is 0 in lenght.
    if(vec1[0] == 0 && vec1[1] == 0){
        throw 666;
        return 0.0;
    }
    
    orthogonalProjection(&vec2[0], &vec1[0], &res_vec[0]);
    
    double result;
    double car_vec[2];
    car_vec[0] = (double)vec2[0];
    car_vec[1] = (double)vec2[1];
    
    result = distanceBetweenPoints(&car_vec[0], &res_vec[0]);
    
    return result;
}

double DatabaseHandler::updateIndicies(int* car_information, int car_heading){
    int car_coordinate_x, car_coordinate_y;
    int track_vector[2], car_vector[2];
    double car_projection[2], track_double[2], car_point[2];
    double heading, wanted_heading, steering_angle;
    double length_of_track_vector, length_of_projection_vector;
    double origo[2] = {0,0};
    
    car_coordinate_x = car_information[0];
    car_coordinate_y = car_information[1];
    
    car_point[0] = (double)car_coordinate_x; 
    car_point[1] = (double)car_coordinate_y;
    heading = (double)car_heading;
    
    track_vector[0] = track[point2][0] - track[point1][0];
    track_vector[1] = track[point2][1] - track[point1][1];
    
    track_double[0] = (double)track_vector[0];
    track_double[1] = (double)track_vector[1];
    
    car_vector[0] = car_coordinate_x - track[point1][0];
    car_vector[1] = car_coordinate_y - track[point1][1];
    
    orthogonalProjection(&car_vector[0], &track_vector[0],
                            &car_projection[0]);                     
    cout << "Orthogonal Projection Vector: [" << car_projection[0] <<
        "," << car_projection[1] << "]" << '\n';
    
    length_of_track_vector = distanceBetweenPoints(&track_double[0],
                                &origo[0]);
    
    length_of_projection_vector = distanceBetweenPoints(&car_projection[0],
                                    &origo[0]);
    
    double length_track, distance_to_next;
        
    //Loop until a segment of the track in front of the car is chosen.
    while((length_of_track_vector - length_of_projection_vector)<= 20 &&
        ((track_double[0] * car_projection[0] > 0) ||
        (track_double[1] * car_projection[1] > 0))){
            
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
        
        track_vector[0] = track[point2][0] - track[point1][0];
        track_vector[1] = track[point2][1] - track[point1][1];
        
        track_double[0] = (double)track_vector[0];
        track_double[1] = (double)track_vector[1];
        
        car_vector[0] = car_coordinate_x - track[point1][0];
        car_vector[1] = car_coordinate_y - track[point1][1];
    }
        
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
    
    return wanted_heading;
}

void DatabaseHandler::initializeIndicies(int x1, int y1){
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
    
    if((track[point1 + 1][0] == 0 && track[point1 + 1][1] == 0) ||
            point1 >= ((sizeof(track)/8) - 1)){
        
        point2 = 0;
    }
    else{
        point2 = place + 1;
    }
    return;
}

void DatabaseHandler::loadTrack(){
    string line;
    ifstream datafile;
    datafile.open("src/database_angle_steering/src/data.txt",ifstream::in);
    int x, y;
    int i = 0;
    
    if(datafile.is_open()){
        while(getline(datafile,line)){
            stringstream trackstream (line);
            
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

void DatabaseHandler::callback(const gulliview_server::Pos& msg){
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
    
    //Instead use a global variable here that is initialized and
    //updated just one time in function main.
    if(callback_loop == 0){
        initializeIndicies(x,y);
    }
    
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
    
    car_coordinates[0] = x;
    car_coordinates[1] = y;
    
    wanted_heading = updateIndicies(&car_coordinates[0], heading);
    
    cout << "Point 1: [" << track[point1][0] << "," << track[point1][1] << "]" << '\n';
    cout << "Point 2: [" << track[point2][0] << "," << track[point2][1] << "]" << '\n'; 
    
    std_msgs::Float64 path_error;

    path_error.data = (float)wanted_heading;

    ROS_INFO("Angle error: %.2f", path_error.data);

    database_pub.publish(path_error);

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
    
    DatabaseHandler obj; 
    
    ros::spin();

    return 0;
}
