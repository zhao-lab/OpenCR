/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "../../include/turtlebot3/turtlebot3_motor_driver2.h"

Turtlebot3MotorDriver2::Turtlebot3MotorDriver2()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  front_left_wheel_id_(DXL_FRONT_LEFT_ID),
  front_right_wheel_id_(DXL_FRONT_RIGHT_ID),
  back_left_wheel_id_(DXL_BACK_LEFT_ID),
  back_right_wheel_id_(DXL_BACK_RIGHT_ID)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

Turtlebot3MotorDriver2::~Turtlebot3MotorDriver2()
{
  close();
}

bool Turtlebot3MotorDriver2::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

bool Turtlebot3MotorDriver2::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_FRONT_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_FRONT_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_BACK_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_BACK_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

bool Turtlebot3MotorDriver2::getTorque()
{
  return torque_;
}

void Turtlebot3MotorDriver2::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

bool Turtlebot3MotorDriver2::readEncoder(int32_t &front_left_value, int32_t &front_right_value, int32_t &back_left_value, int32_t &back_right_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(front_left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(front_right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(back_left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(back_right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(front_left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(front_right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(back_left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(back_right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  front_left_value  = groupSyncReadEncoder_->getData(front_left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  front_right_value = groupSyncReadEncoder_->getData(front_right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  back_left_value  = groupSyncReadEncoder_->getData(back_left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  back_right_value = groupSyncReadEncoder_->getData(back_right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver2::writeVelocity(int64_t front_left_value, int64_t front_right_value, int64_t back_left_value, int64_t back_right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t front_left_data_byte[4] = {0, };
  uint8_t front_right_data_byte[4] = {0, };
  uint8_t back_left_data_byte[4] = {0, };
  uint8_t back_right_data_byte[4] = {0, };


  front_left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(front_left_value));
  front_left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(front_left_value));
  front_left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(front_left_value));
  front_left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(front_left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(front_left_wheel_id_, (uint8_t*)&front_left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  front_right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(front_right_value));
  front_right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(front_right_value));
  front_right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(front_right_value));
  front_right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(front_right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(front_right_wheel_id_, (uint8_t*)&front_right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  back_left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(back_left_value));
  back_left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(back_left_value));
  back_left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(back_left_value));
  back_left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(back_left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(back_left_wheel_id_, (uint8_t*)&back_left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  back_right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(back_right_value));
  back_right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(back_right_value));
  back_right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(back_right_value));
  back_right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(back_right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(back_right_wheel_id_, (uint8_t*)&back_right_data_byte);
  if (dxl_addparam_result != true)
    return false;


  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver2::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  // TODO - currently, both wheels on left + right will behave the same at all times. Fix later if necessary.

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT], (int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}