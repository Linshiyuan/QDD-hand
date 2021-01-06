
#include "ros/ros.h"

#include <bits/stdint-uintn.h>
#include <cstdio>
// #include <eigen3/Eigen/src/Geometry/Quaternion.h>

#include <iostream>
#include <math.h>
#include <ostream>
#include <string>

#include <serial/serial.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "robotis_control.hpp"

serial::Serial PC_ser, ins_motor;
void setTargetPos(uint16_t tPos, std::vector<uint8_t> &cmd) {
  cmd[6] = tPos;
  cmd[7] = tPos >> 8;
  cmd[8] = 0;
  for (int i = 2; i < 8; i++) {
    cmd[8] += cmd[i];
  }
     ROS_INFO(" tPos in void  is %d ", tPos);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "ins_motor_test");
  ros::NodeHandle n;

  try {
    ins_motor.setPort("/dev/ttyUSB6");
    ins_motor.setBaudrate(921600);
    // ros_ser.setBaudrate(2000000);
    // ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 1000
    // bool level=true;
    // ros_ser.setRTS(level);
    // ros_ser.setDTR(level);
    ins_motor.setTimeout(to);
    // ros_ser.setParity(serial::parity_even);
    ins_motor.open();

    PC_ser.setPort("/dev/ttyUSB3");
    PC_ser.setBaudrate(921600);
    serial::Timeout to2 = serial::Timeout::simpleTimeout(1000); // 1000
    PC_ser.setTimeout(to2);
    PC_ser.open();

  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (ins_motor.isOpen()) {
    ROS_INFO_STREAM("Serial Port opened");
  } else {
    return -1;
  }

  char get1 = 0x1a;
  char get2 = 0x58;
  char test_Char[] = "test_1234";
  // const char
  // cmd_start[]={static_cast<char>(0xa5),0x5a,04,01,05,static_cast<char>(0xaa)};
  // const unsigned char cmd_start[]={0xa5,0x5a,04,01,05,0xaa};
  std::string send1(1, get1);
  std::string send2(1, get2);
  send2 = "okok";
  std::vector<uint8_t> cmd_start{
      0xa5, 0x5a, 04, 01, 05, 0xaa}; // 55 aa 04 01 02 37 13 05 56
  std::vector<uint8_t> cmd_pos{0x55, 0xaa, 0x04, 0x01, 0x20,
                               0x37, 0x13, 0x05, 0x56};
  std::vector<uint8_t> cmd_pos2{0x55, 0xaa, 0x04, 0x01, 0x02,
                                0x37, 0x13, 0x05, 0x56};
  ins_motor.write(cmd_pos);

  // uint16_t ins_pos=2000;

  setTargetPos(2000, cmd_pos);
float counter=0,tPos=0;
  ros::Rate loop_rate(800);
  while (ros::ok()) {
      counter++;
      tPos=sin(counter/1000)*1000+1000;

 setTargetPos(tPos, cmd_pos);

    PC_ser.write(cmd_pos);
    ins_motor.write(cmd_pos);
    // ros_ser.write(cmd_start);
    // loop_rate.sleep();
    int n;
    n = ins_motor.available();
    ROS_INFO(" tPos  is %f ", tPos);
    if (n > 5) {
      //  ROS_INFO_STREAM("Reading from serial port");
      std_msgs::String serial_data;
      geometry_msgs::Pose imu_data;

      //   ROS_INFO("ros_ser.available() is  [%d] ",n);

      std::vector<uint8_t> buffer, u8v;
      uint8_t test1 = 0xaa;
      uint8_t test2 = 0;
      u8v.push_back(test1);
      u8v.push_back(test2);
      u8v.push_back(0x1f);
      auto readout = ins_motor.read(buffer, n);
      PC_ser.write(buffer);
      //   for (int i=0; i<n; i++) {
      //    std::cout<<std::hex<< buffer[i]<<std::endl;

      //   }

      // std::cout<<std::hex<< buffer[i]<<std::endl;

      double att_x, att_y, att_z;
      //   att_x = int16_t((buffer[3] << 8) + (buffer[4])) * 0.1;
      //   att_y = int16_t((buffer[5] << 8) + (buffer[6])) * 0.1;
      //   att_z = int16_t((buffer[7] << 8) + (buffer[8])) * 0.1;
      // xh540.send_test((float )att_y);
      //   imu_data.orientation.x = att_x;
      //   imu_data.orientation.y = att_y;
      //   imu_data.orientation.z = att_z;
      //   // ROS_INFO_STREAM("att_x is "<<att_x<<"att_y is "<<att_y<<"att_z is
      //   %f
      //   // "<<att_z);

      // ROS_INFO("att_x is %3f att_y is %3f att_z is %3f ", att_x, att_y,
      // att_z);
      ROS_INFO(" read once ");

      //   std::stringstream ss;
      //   ss << std::hex << readout;
      //   //   serial_data.data = ros_ser.read(ros_ser.available());
      //   serial_data.data = ss.str();
      //  ROS_INFO_STREAM("Read: " << serial_data.data);
      //  ROS_INFO_STREAM("Read: is ");
      //  std::cout<<ss.str()<<std::endl;
      //将串口数据发布到主题sensor
      // sensor_pub.publish(serial_data);
      // imu_pub.publish(imu_data);
    }

    // if (n < 9) {
    //   ros_ser.read(n);
    // }
    ros::spinOnce();
    loop_rate.sleep();
  }
}