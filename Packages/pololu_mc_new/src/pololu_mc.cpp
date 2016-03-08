/*
  Copyright 2014-2015 Viktor Nilsson and Herman Fransson
  Copyright 2012-2014 Benjamin Vedder benjamin@vedder.se
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pololu_mc.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace boost;

// Callback when something is published on 'control_effort'
void ControlEffortCallback(const ackermann_msgs::AckermannDrive::ConstPtr& control_effort_input)
{
  // the stabilizing control effort
  ROS_INFO("Got a message!: \nSteering angle: %f\nSteering angle velocity:%f\nSpeed: %f\nAcceleration: %f\nJerk: %f\n", control_effort_input->steering_angle, control_effort_input->steering_angle_velocity, control_effort_input->speed,control_effort_input->acceleration, control_effort_input->jerk);

    control_effort = control_effort_input->steering_angle;
}

/*
  When an ackermann_msg is received, we log the msg and set steering angle and speed.
*/
/*void callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
  ROS_INFO("Got a message!: \nSteering angle: %f\nSteering angle velocity:%f\nSpeed: %f\nAcceleration: %f\nJerk: %f\n", msg->steering_angle, msg->steering_angle_velocity, msg->speed,msg->acceleration, msg->jerk);

  set_steering_target(msg->steering_angle);
  set_speed(msg->speed);
}*/

/*
  Initializes the motor controller.
  Opens the serial port.
*/
int init_mc() {
  std::stringstream ss;

  try {
    mc = new Serial("/dev/ttyACM0", 115200);
  } catch(boost::system::system_error e) {
    ss << "Error, cant open Serial Port!\n"
       << "Error msg: " << e.what();
    ROS_INFO("%s", ss.str().c_str());
    return -1;
  }
    ROS_INFO("Inside init-mc ok"); //Debugging
  return 0;
}

/*
  Sends an set speed command to the motor controller.
*/
int set_speed(float speed) {
  const int len = 4; // Set speed commands are 4 bytes long
  uint8_t cmd[len] = {0x84, 0x00, 0x00, 0x00}; // 0x87 == set target
  uint8_t mc_speed = uint8_t(speed*100+6040);

  cmd[1] = 4; // Channel hardcoded to nr 4
  cmd[2] = mc_speed & 0x7F; // Will be max 10, so never uses top bits
  cmd[3] = mc_speed >> 7 & 0x7F;

  cout << "Speed to be set: " << std::dec << mc_speed << endl;
  printf("Set target (mc-speed) command to be sent: %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3]);

  current_speed = speed;
  return send_packet(cmd, len);
}

/*
  Sends a steering command to the Polulu servo controller.
*/
int set_steering_target(float angle) {
  const int len = 4; // set target command is 4 bytes long
  uint8_t cmd[len] = {0x84, 0x00, 0x00, 0x00};

/*We get an angle value between -22 and 22 from joystick.
  Seems like the central value for polulu controller is 6000 and values cannot be negative.*/
  position = uint16_t(angle*45+6000);
  cmd[1] = 5; // Channel hardcoded to nr 5
  cmd[2] = position & 0x7F;
  cmd[3] = position >> 7 & 0x7F;

  cout << "Position to be set: " << position << endl;
  //printf("Set target (angle) command to be sent: %02x, %02x, %02x, %02x\n",
  //        cmd[0], cmd[1], cmd[2], cmd[3]);

  current_steering_angle = angle;
  return 1; //send_packet(cmd, len);
}

int set_steering_speed(float angle_velocity) {
  const int len = 4; // set speed command is 4 bytes long
  uint8_t cmd[len] = {0x87, 0x00, 0x00, 0x00}; // 0x87 == set speed
  uint8_t mc_angle_velocity = uint8_t(angle_velocity);

  cmd[1] = 5; // Channel hardcoded to nr 5
  cmd[2] = mc_angle_velocity & 0x7F;

  cout << "Speed for angle to be set: " << angle_velocity << endl;
  printf("Set speed (angle) command to be sent: %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3]);

  return 1; //send_packet(cmd, len);
}

int current_brake(int32_t brake_current) {
  const int len = 5; // cmd is 5 bytes long
  /*
    cmd[0]: what command (0x03 == COMM_SET_CURRENT_BRAKE
    cmd[1-4]: current in mA
  */
  uint8_t cmd[len] = {COMM_SET_CURRENT_BRAKE, 0x00, 0x00, 0x00, 0x00};
  cmd[1] = brake_current >> 24;
  cmd[2] = brake_current >> 16;
  cmd[3] = brake_current >> 8;
  cmd[4] = brake_current;

  cout << "Brake current to be set: " << brake_current << "mA" << endl;
  printf("Servo move command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  This puts the motor to a full stop.
  Maybe only use this for emergencies.
*/
int set_duty(int32_t duty) {
  const int len = 5; // cmd is 5 bytes long
  /*
    cmd[0]: what command (0x01 == COMM_SET_DUTY
    cmd[1-4]: duty in ?
  */
  uint8_t cmd[len] = {COMM_SET_DUTY, 0x00, 0x00, 0x00, 0x00};
  cmd[1] = duty >> 24;
  cmd[2] = duty >> 16;
  cmd[3] = duty >> 8;
  cmd[4] = duty;

  cout << "Duty to be set: " << duty << endl;
  printf("Duty command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  Requests motor controller data from the car.
*/
void get_values() {
  unsigned char cmd[1] = {0x00}; // 0x00 == COMM_GET_VALUES
  //send_packet(cmd, 1);
  //recv_packet();
}

/*
  Sends an "I'm alive"-message to the car.
  The car is automatically stopped if it has not received anything for 500ms.
*/
void send_alive() {
  uint8_t cmd[1] = {COMM_ALIVE}; // 0x12 == COMM_ALIVE
  //send_packet(cmd, 1);
}

/*
  Sends a command to the motor controller over serial. Appends a checksum.
  From bldc-tool/packetinterface.cpp.
*/
int send_packet(const unsigned char *data, int len) {
  unsigned char buffer[len];
//  buffer[0] = 2;
//  buffer[1] = len;

  memcpy(buffer, data, len);

//  unsigned short crc = crc16(data, len);
//  buffer[len + 2] = crc >> 8;
//  buffer[len + 3] = crc;
//  buffer[len + 4] = 3;

  try {
    mc->writeString(buffer, len);
  } catch (boost::system::system_error& e) {
    cout << "Error: " << e.what() << endl;
    return 1;
  }
  return 0;
}

/*
  Receives a packet from the motor controller over serial.
  From bldc-tool/packetinterface.cpp.
*/
int recv_packet() {
  std::stringstream ss;

  uint8_t status;
  uint8_t len;
  uint8_t crc_high;
  uint8_t crc_low;

  unsigned char recv[256];

  try {
    status = mc->readChar();
    if (status != 2) {
      return -1;
    }
    len = mc->readChar();
    for (int i=0; i<len; i++) {
      recv[i] = mc->readChar();
    }
    crc_high = mc->readChar();
    crc_low = mc->readChar();

    if (mc->readChar() == 3) {
      if (crc16(recv, len) == ((unsigned short)crc_high << 8
          | (unsigned short)crc_low)) {
        // Packet received!
        process_packet(recv, len);
      }
    }
  } catch(boost::system::system_error& e) {
    ss << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}

/*
  Processes a packet from the motor controller.
*/
void process_packet(const unsigned char *data, int len) {
  if (!len) {
    return;
  }

  COMM_PACKET_ID packet_id;
  int32_t ind = 0;

  packet_id = (COMM_PACKET_ID) data[0];
  data++;
  len--;

  pololu_mc::MCValues msg;

  switch (packet_id) {
    case COMM_GET_VALUES: // 0x00 == COMM_GET_VALUES
      ind = 0;
      msg.TEMP_MOS1 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS2 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS3 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS4 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS5 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS6 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_PCB = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.current_motor = ((double)buffer_get_int32(data, &ind)) / 100.0;
      msg.current_in = ((double)buffer_get_int32(data, &ind)) / 100.0;
      msg.duty_now = ((double)buffer_get_int16(data, &ind)) / 1000.0;
      msg.rpm = ((double)buffer_get_int32(data, &ind));
      msg.v_in = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.amp_hours = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.amp_hours_charged = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.watt_hours = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.watt_hours_charged = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.tachometer = ((double)buffer_get_int32(data, &ind));
      msg.tachometer_abs = ((double)buffer_get_int32(data, &ind));

      // This data is not from the car
      msg.current_steering_angle = current_steering_angle;
      msg.current_speed = current_speed;
      mc_values_pub.publish(msg);
      break;

    default:
      break;
  }
}


/*
  No comments.
*/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Pololu_Motor_Controller_Actuator");
  ros::NodeHandle n;

  control_effort = 0.0;

    ROS_INFO("ros-init and handle ok");  //Debugging

  /*if (init_mc() != 0) {
    // ERROR
    return -1;
  }
*/
    //ROS_INFO("init-mc ok");  //Debugging

    set_steering_speed(10);

  ros::NodeHandle node_priv("~");  //Not needed now, but cound maybe be useful later

    //ROS_INFO("nodehandle-params ok"); //Debugging

 // ros::Subscriber sub_mc_cmds = n.subscribe("mc_cmds", 1, callback); //for joy ackermann_msgs

  ros::Rate loop_rate(1000);
  mc_values_pub = n.advertise<pololu_mc::MCValues>("mc_values", 1000);

    //ROS_INFO("mc topics ok"); //Debugging

  // Advertise a plant state msg
  /*std_msgs::Float64 plant_state;
  ros::Publisher servo_state_pub = n.advertise<std_msgs::Float64>("state", 1);
  plant_state.data = 6000;*/

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = n.subscribe("control_effort", 1, ControlEffortCallback );

    //ROS_INFO("controller topics ok");

  int counter = 0;
  while (ros::ok()) {

    //ROS_INFO("Beginning of while-loop"); //Debugging

    // Get values every x ms
    if (counter % GET_VALUES_INTERVAL == 0) {
      get_values();
    }
    // Send COMM_ALIVE every x ms
    // (might not be needed since we're sending COMM_GET_VALUES)
    if (counter % SEND_ALIVE_INTERVAL == 0) {
      send_alive();
    }
    counter = (counter+1)%1000;

    ros::spinOnce();
    //ROS_INFO("Testing...");  //Debugging

  // Calculate a value between -22 and 22.
    set_steering_target(control_effort/7);

    ROS_INFO("steering: %f", control_effort/7);  //Debugging
    //plant_state.data = position;
    //servo_state_pub.publish(plant_state);

    // But process callbacks every loop
    ros::spinOnce();
    loop_rate.sleep();
    //ROS_INFO("End of while-loop");  //Debugging
  }
    ROS_INFO("Almost done...");  //Debugging
  return 0;
}

