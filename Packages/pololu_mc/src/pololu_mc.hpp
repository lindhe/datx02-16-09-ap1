/*
  Copyright 2014-2015 Viktor Nilsson and Herman Fransson.
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "pololu_mc/MCValues.h"

#include <unistd.h>
#include <iostream>
#include <sstream>
#include "datatypes.h"
#include "Serial.hpp"
#include "utils.hpp"

#define GET_VALUES_INTERVAL 20
#define SEND_ALIVE_INTERVAL 50

int init_mc();
int set_speed(uint16_t mc_speed);
int set_steering_target(uint16_t angle);
int set_steering_speed(uint8_t mc_angle_velocity);

int send_packet(const unsigned char *data, int len);
int recv_packet();
void process_packet(const unsigned char *data, int len);

Serial *mc;
ros::Publisher mc_values_pub;

uint16_t current_steering_angle;
uint16_t current_speed;
static float control_effort = 0;
