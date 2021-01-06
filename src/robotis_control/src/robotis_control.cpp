#include "robotis_control.hpp"

// robotisControl::robotisControl(){};

robotisControl::robotisControl(int baudrate_, char const *port_name)
    :

      portHandler(dynamixel::PortHandler::getPortHandler(port_name)),

      // Initialize PacketHandler instance
      // Set the protocol version
      // Get methods and members of Protocol1PacketHandler or
      // Protocol2PacketHandler
      packetHandler(
          dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),

      // Initialize GroupSyncWrite instance
      groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION,
                     LEN_PRO_GOAL_POSITION),

      // Initialize Groupsyncread instance for Present Position
      groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION,
                    LEN_PRO_PRESENT_POSITION)

{

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL; // Communication result
  bool dxl_addparam_result = false;   // addParam result
  bool dxl_getdata_result = false;    // GetParam result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE,
                              DXL_MAXIMUM_POSITION_VALUE}; // Goal position

  uint8_t dxl_error = 0; // Dynamixel error
  uint8_t param_goal_position[4];
  int32_t dxl1_present_position = 0,
          dxl2_present_position = 0; // Present position

  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  } else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    // getch();
    // return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(baudrate_)) {
    printf("Succeeded to change the baudrate!\n");
  } else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    // getch();
    // return 0;
  }
  dxl_comm_result =
      packetHandler->write1ByteTxRx(portHandler, 1, 11, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    printf(" #%d  mode set failed  \n", 1);
  } else {
    printf("Dynamixel#%d mode set  successfully \n", 1);
  }

  dxl_comm_result =
      packetHandler->write1ByteTxRx(portHandler, 1, 11, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    printf(" #%d  mode set failed  \n", 1);
  } else {
    printf("Dynamixel#%d mode set  successfully \n", 1);
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packetHandler->getRxPacketError(dxl_error);
  } else {
    printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
  } else if (dxl_error != 0) {
    packetHandler->getRxPacketError(dxl_error);
  } else {
    printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
  }

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true) {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    // return 0;
  }

  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true) {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    // return 0;
  }
}

void robotisControl::send_test(float angle) {


    
  int goalPostion;



//   for (int i = 0; i < 12; i++) {
//     double motor_goal = 2048;
//     double theta = jointCMD[i] * 180 / 3.1415926;
//     motor_goal += (theta) / 0.088;
//     goalPostion[i] = (int)motor_goal;
//   }

      double motor_goal = 2048;
    // double theta = jointCMD[i] * 180 / 3.1415926;
    motor_goal += (angle) / 0.088;
    goalPostion = (int)motor_goal;






  int DXL_Len = 2;

  int dxl_goal_position[2];
  uint8_t param_goal_position[2];
  bool dxl_getdata_result;  // GetParam result
  bool dxl_addparam_result; // addParam result
   int dxl_comm_result;      



  for (int DXL_ID = 1; DXL_ID < 3; DXL_ID++) {
    // int goal_position = goalPostion_order[DXL_ID - 1];

    // std::cout << goal_position << std::endl;

    param_goal_position[0] = DXL_LOBYTE(goalPostion);
    param_goal_position[1] = DXL_HIBYTE(goalPostion);
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_goal_position);
    if (dxl_addparam_result != true) {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID);
    //   return false;
    }
  }
  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // return false;
  }

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}