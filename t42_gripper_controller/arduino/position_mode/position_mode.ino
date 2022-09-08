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

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t RIGHT_DXL_ID = 1;
const float RIGHT_DXL_PROTOCOL_VERSION = 1.0;

const uint8_t LEFT_DXL_ID = 1;
const float LEFT_DXL_PROTOCOL_VERSION = 2.0;
float right_val = 0.0;
float left_val = 0.0;
DynamixelShield right_dxl;

bool led= true;
DynamixelShield left_dxl;
//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  right_dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  right_dxl.setPortProtocolVersion(RIGHT_DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  right_dxl.ping(RIGHT_DXL_ID);
  // Turn off torque when configuring items in EEPROM area
  right_dxl.torqueOff(RIGHT_DXL_ID);
  right_dxl.setOperatingMode(RIGHT_DXL_ID, OP_POSITION);
  right_dxl.torqueOn(RIGHT_DXL_ID);
  
  left_dxl.begin(57600);
  left_dxl.setPortProtocolVersion(LEFT_DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  left_dxl.ping(LEFT_DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  left_dxl.torqueOff(LEFT_DXL_ID);
  left_dxl.setOperatingMode(LEFT_DXL_ID, OP_POSITION);
  left_dxl.torqueOn(LEFT_DXL_ID);
}

//void control_motor(int motor_idx, int protocol_version, float degree)
//{
//  dxl.setPortProtocolVersion(protocol_version);
//  dxl.setGoalPosition(motor_idx, degree, UNIT_DEGREE);
//}

void loop() {
  String data = "";

  while (DEBUG_SERIAL.available())
  {
    char character = DEBUG_SERIAL.read(); // Receive a single character from the software serial port
    data.concat(character); // Add the received character to the receive buffer
    if (character == '\n')
    {
      int comma_idx = data.indexOf(",");
      String right = data.substring(0,comma_idx);
      String left = data.substring(comma_idx+1);
      int tmp = right.indexOf(":");
      right_val = right.substring(tmp+1).toFloat();
      tmp = left.indexOf(":");
      left_val = left.substring(tmp+1).toFloat();      
      
      // Clear receive buffer so we're ready to receive the next line
      data = "";
    }
  }
  right_dxl.setGoalPosition(RIGHT_DXL_ID, right_val, UNIT_DEGREE);
  left_dxl.setGoalPosition(LEFT_DXL_ID, left_val, UNIT_DEGREE);
//  control_motor(RIGHT_DXL_ID, RIGHT_DXL_PROTOCOL_VERSION, right_val);
//  control_motor(LEFT_DXL_ID, LEFT_DXL_PROTOCOL_VERSION, left_val);
  // put your main code here, to run repeatedly:

  float tmp_left_val = left_dxl.getPresentPosition(LEFT_DXL_ID, UNIT_DEGREE);
  float tmp_right_val = right_dxl.getPresentPosition(RIGHT_DXL_ID, UNIT_DEGREE);
  String tmpstr = "Right:";
  tmpstr.concat(tmp_right_val);
  tmpstr.concat(",Left:");
  tmpstr.concat(tmp_left_val);
  DEBUG_SERIAL.println(tmpstr);
  delay(250);
}
