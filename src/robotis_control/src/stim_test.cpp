
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

serial::Serial ros_ser,PC_ser;
//回调函数
void callback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO_STREAM("Write to serial port" << msg->data);
  ros_ser.write(msg->data);
}

ros::Publisher xsens_pub;
geometry_msgs::Pose xsens;
void xsens_callback(const sensor_msgs::Imu::ConstPtr &msg) {
  // ROS_INFO("xsens data is ",msg->orientation.x);
  Eigen::Quaternionf xsens_Q;

  xsens_Q.w() = msg->orientation.w;
  xsens_Q.x() = msg->orientation.x;
  xsens_Q.y() = msg->orientation.y;
  xsens_Q.z() = msg->orientation.z;
  auto eulerAngles = xsens_Q.toRotationMatrix().eulerAngles(2, 1, 0);
  for (int i = 0; i < 3; i++) {
    eulerAngles[i] *= (180 / 3.1415926);
  }
  xsens.orientation.x = eulerAngles[1];
  xsens.orientation.y = eulerAngles[2];
  xsens_pub.publish(xsens);
  
  // if (eulerAngles[0]>90)   {
  // eulerAngles[0]+=-180;
  // }
  // if (eulerAngles[1]>90)   {
  // eulerAngles[1]=180-eulerAngles[1];
  // }

  std::cout << "xsens is \n" << eulerAngles << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_serial_node");
  ros::NodeHandle n;
  
  robotisControl xh540(4000000,"/dev/ttyUSB1");
//   robotisControl xh540;
  //订阅主题command
  ros::Subscriber command_sub = n.subscribe("command", 1000, callback);
  // ros::Subscriber xsens_sub = n.subscribe("imu/data", 10, xsens_callback);
  //发布主题sensor
  ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
  ros::Publisher imu_pub = n.advertise<geometry_msgs::Pose>("VAST", 10);

  xsens_pub = n.advertise<geometry_msgs::Pose>("xsens_data", 10);
  try {
    ros_ser.setPort("/dev/ttyUSB6");
    ros_ser.setBaudrate(921600);
    // ros_ser.setBaudrate(2000000);
    // ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(10); // 1000
    bool level=true;
    // ros_ser.setRTS(level);
    // ros_ser.setDTR(level);
    ros_ser.setTimeout(to);
    // ros_ser.setParity(serial::parity_even);
    ros_ser.open();

    PC_ser.setPort("/dev/ttyUSB3");
    PC_ser.setBaudrate(921600);

    serial::Timeout to2 = serial::Timeout::simpleTimeout(1000); // 1000
    // bool level=true;

    PC_ser.setTimeout(to2);
    PC_ser.open();


  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (ros_ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port opened");
  } else {
    return -1;
  }
char get1=0x1a;
char get2=0x58;
char test_Char[]="test_1234";
// const char cmd_start[]={static_cast<char>(0xa5),0x5a,04,01,05,static_cast<char>(0xaa)};
// const unsigned char cmd_start[]={0xa5,0x5a,04,01,05,0xaa};
std::string send1(1,get1);
std::string send2(1,get2);
send2="okok";
 std::vector<uint8_t> cmd_start{0xa5,0x5a,04,01,05,0xaa};

ros_ser.write(cmd_start);


  ros::Rate loop_rate(3000);
  while (ros::ok()) {


    // ros_ser.write(cmd_start);
    // loop_rate.sleep();
    int n ;
    n= ros_ser.available();
     ROS_INFO("n available is %d",n);
    if (n > 37) {
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
      auto readout = ros_ser.read(buffer, n);
      PC_ser.write(buffer);
    //   for (int i=0; i<n; i++) {
    //    std::cout<<std::hex<< buffer[i]<<std::endl;
  
    //   }
  
  // std::cout<<std::hex<< buffer[i]<<std::endl;


      double att_x, att_y, att_z;
      att_x = int16_t((buffer[3] << 8) + (buffer[4])) * 0.1;
      att_y = int16_t((buffer[5] << 8) + (buffer[6])) * 0.1;
      att_z = int16_t((buffer[7] << 8) + (buffer[8])) * 0.1;
// xh540.send_test((float )att_y);
    //   imu_data.orientation.x = att_x;
    //   imu_data.orientation.y = att_y;
    //   imu_data.orientation.z = att_z;
    //   // ROS_INFO_STREAM("att_x is "<<att_x<<"att_y is "<<att_y<<"att_z is %f
    //   // "<<att_z);

        // ROS_INFO("att_x is %3f att_y is %3f att_z is %3f ", att_x, att_y,
        // att_z);

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