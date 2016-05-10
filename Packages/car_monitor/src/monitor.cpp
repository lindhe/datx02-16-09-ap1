#include <ros/ros.h>
#include "gulliview_server/Pos.h"
#include <cmath>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <cmath>
#include <numeric>
#include <stdlib.h>
#include <ros/package.h>

using namespace std;
class EventHandler{
    private:
        ros::Publisher marker_pub;
        ros::NodeHandle n;
        ros::Subscriber sub; 
    public:
        EventHandler(){
            marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
            sub = n.subscribe("position", 1, &EventHandler::chatterCallback, this);
        }
        void chatterCallback(const gulliview_server::Pos& msg){
            uint32_t shape = visualization_msgs::Marker::ARROW;
            visualization_msgs::Marker points;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/my_frame";
            points.header.frame_id = "/my_frame";
            marker.header.stamp = ros::Time::now();
            points.header.stamp = ros::Time::now();

            marker.ns = "monitor";
            marker.id = 0;
            points.ns = "monitor";
            points.id = 1;

            marker.type = shape;
            points.type = visualization_msgs::Marker::POINTS;

            marker.action = visualization_msgs::Marker::ADD;
            points.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = (double)msg.x/1000;
            marker.pose.position.y = (double)msg.y/1000;
            marker.pose.position.z = 0.22;

            float angle = (msg.heading) * 3.14 / 180; 
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;// - prev_y)/length;
            marker.pose.orientation.z = 1.0*sin(angle/2);
            marker.pose.orientation.w = cos(angle/2);

            marker.scale.x = 0.55;
            marker.scale.y = 0.24;
            marker.scale.z = 0.22;
            points.scale.x = 0.03;
            points.scale.y = 0.03;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            points.color.g = 1.0f;
            points.color.a = 1.0;
            string line;
            ifstream datafile;
            string path = ros::package::getPath("database");
            string file_path = "/src/data.txt";
            path = path+file_path;
            datafile.open(path.c_str(),ifstream::in);
            int x, y;

            if(datafile.is_open()){
                while(getline(datafile,line)){
                    stringstream trackstream (line);

                    //Get values from row
                    trackstream >> x >> y;

                    geometry_msgs::Point p;
                    p.x = (double)x/1000;
                    p.y = (double)y/1000;
                    p.z = 0;
                    points.points.push_back(p);
                }
                datafile.close();
            }
            else cout << "Unable to open file";

            marker.lifetime = ros::Duration();
            points.lifetime = ros::Duration();

            marker_pub.publish(points);
            marker_pub.publish(marker);
        }

};


int main( int argc, char** argv ){
    ros::init(argc, argv, "monitor");
    EventHandler obj;
    ros::spin();
    return 0;
}
