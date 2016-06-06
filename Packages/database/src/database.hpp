#ifndef DATABASE_HPP
#define DATABASE_HPP

//Maximum number of points to construct the track.
//Have one more point in the maximum value than needed.
#define maximum_number_of_points 320

//The length between the back axle and front axle of the car
#define length_between_axles_of_car 325

//The maximum steering angle of the car
#define maximum_steering_angle 26

//Lookahead distance of the car. A point within this distance of the car
//won't be chosen
#define lookahead 200
//Old value - 1400

//The maximum lookahead of the car. A point outside this distance wont be
//chosen.

//---------------------------IMPORTANT-----------------------------------

//If using a circular track, this value needs to be smaller than the
//radius of the circle
#define maximum_lookahead_range 1150
//Old value 2500

//The maximum range of the track in all directions. If the car leaves this
//area, it will stop running.

//Maximum x-value
#define maximum_x 2300

//Minimum x-value
#define minimum_x -1700

//Maximum y-value
#define maximum_y 2000

//Minimum y-value
#define minimum_y -1300

using namespace std;

class DatabaseHandler{

    /**
    *
    * This node is a subscriber to the topic position, which takes
    * messages of type pos, with contents long x, long y, long heading.
    *
    * This node is a publisher to the topic path_error, which takes
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
    int track[maximum_number_of_points][2];
    
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
    DatabaseHandler();
    
    /**
    * Projects one vector onto another and returns the
    * resulting vector. vec2 is projected onto vec1.
    * 
    * Arguments: pointers to 2 vectors on the form int[x,y].
    * Vectors start in origo, making x and y their end points.
    * 
    * res_vec is the vector variable that should store the result,
    * also on form double[x,y].
    */
    void orthogonalProjection(int* vec2, int* vec1, double* res_vec);
    
    /**
    * Calculates the new steering angle of the car. 
    *
    * Arguments: car_point[x,y] is the coordinates of the car and heading
    * is the angle of the car in degrees.
    *
    * Returns: The new steering angle in degrees.
    */
    double calculateSteeringAngle(int* car_point, int heading);
    
    /**
    * Takes the global coordinates of a point and converts them to coordinates
    * in the cars local coordinate system.
    * 
    * Arguments: Pointer to car_point, which is an array on form int[x,y] where
    * x and y are the global coordinates of the car.
    * Heading is the angle of the car in degrees.
    * converted coordinates.
    *
    * new_point is an array on form int[x,y] where the function stores the
    * resulting coordinates in the cars local coordinate system.
    */
    void convertCoordinates(int* car_point, int heading, int* new_point);
    
    /**
    * Calculates the distance between two points.
    * 
    * Arguments: pointers to 2 points on form double[x,y].
    *
    * Returns: the distance between the two points.
    */
    double distanceBetweenPoints(double* first_point, double* second_point);
    
    /** 
    * Calculates the shortest distance from the car to the track vector.
    *
    * Arguments: Pointers to vectors on form int[x,y]. Vectors start in origo,
    * making x and y their end points.
    *
    * Returns: the shortest distance between the car and the track;
    */
    double calculateDistance(int* vec2, int* vec1);
    
    /**
    * Updates the pointers to the track, so that the chosen segment of the track
    * is located in front of the car.
    * 
    * Arguments: pointer to car_information, which is on the form int[x,y]
    * where x and y represents the coordinates of the car. car_heading is the
    * heading of the car in degrees.
    *
    * Returns the angle in degrees the car needs to have to reach the next
    * point.
    */
    double updateIndicies(int* car_information, int car_heading);
    
    /**
    * Initializes track pointers. Chooses the closest point to the car and the
    * following point on the track.
    *
    * Arguments: x and y are the global coordinates of the car.
    * 
    */
    void initializeIndicies(int x1, int y1);
    
    /**
    * Loads the track from a textfile and stores the checkpoints in the array
    * int track[][]; Fyll på här med sökväg och namn för filen.
    */
    void loadTrack();
    
    /**
    * Runs every time the program receives a message from the master node.
    * Runs updateIndicies() and publishes the error that was returned.
    */
    void callback(const gulliview_server::Pos& msg);
    
};

#endif
