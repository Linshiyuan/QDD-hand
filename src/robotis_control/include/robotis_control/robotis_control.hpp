#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
// #include <Eigen/Dense>
#include <Eigen/Geometry>


#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
// #define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
// #define ADDR_PRO_LED_RED                65
// #define ADDR_PRO_GOAL_POSITION          116
// #define ADDR_PRO_PRESENT_POSITION       132

#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                65
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        4000000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0            // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2048              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     5                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b



class robotisControl
{

    public:
    //   robotisControl();
     robotisControl(int baudrate_,  char const *port_name);
    void send_test(float angle);


    private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
  // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite;
  // Initialize GroupBulkRead instance
    dynamixel::GroupSyncRead groupSyncRead;


};