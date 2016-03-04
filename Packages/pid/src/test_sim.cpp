#include <ros/ros.h>
#include <std_msgs/Float64.h>
static bool i_am_stupid = false;

// Callback when something is published on 'control_effort'
void ControlEffortCallback(const std_msgs::Float64& control_effort_input)
{
  // the stabilizing control effort
  if (i_am_stupid)
  {
    ROS_INFO("Hejsan Coco!");
  }
  else
  {
    ROS_INFO("You are a genius!");
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "dummy");
  ros::NodeHandle test_node;

  ros::NodeHandle node_priv("~");
  node_priv.param<bool>("i_am_stupid", i_am_stupid, false);

  // Advertise a plant state msg
  std_msgs::Float64 plant_state;
  ros::Publisher servo_state_pub = test_node.advertise<std_msgs::Float64>("state", 1);
  plant_state.data = 4.7;


  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = test_node.subscribe("control_effort", 1, ControlEffortCallback );

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Testing...");
    plant_state.data = plant_state.data + 1;
    servo_state_pub.publish(plant_state);
  }

  return 0;
}
