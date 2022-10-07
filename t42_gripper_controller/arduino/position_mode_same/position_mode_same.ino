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
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t RIGHT_DXL_ID = 2;
const float PROTOCOL_VERSION = 2.0;

const uint8_t LEFT_DXL_ID = 1;
float right_val = 0.0;
float left_val = 0.0;
int counter = 0;

String right, left;
bool led= true;
DynamixelShield dxl;
//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(9600);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(1,048,575);
  // Get DYNAMIXEL information
  dxl.ping(RIGHT_DXL_ID);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(RIGHT_DXL_ID);
  dxl.setOperatingMode(RIGHT_DXL_ID, OP_POSITION);
  dxl.torqueOn(RIGHT_DXL_ID);
  
  dxl.setPortProtocolVersion(LEFT_DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(LEFT_DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_DXL_ID);
  dxl.setOperatingMode(LEFT_DXL_ID, OP_POSITION);
  dxl.torqueOn(LEFT_DXL_ID);
}

void control_motor(int motor_idx, int protocol_version, float degree)
{
  dxl.setPortProtocolVersion(protocol_version);
  dxl.setGoalPosition(motor_idx, degree, UNIT_DEGREE);
}

float read_motor(int motor_idx, int protocol_version)
{
  dxl.setPortProtocolVersion(protocol_version);
  return dxl.getPresentPosition(motor_idx, UNIT_DEGREE);
}

void loop() {
  String data = "";
  int tmp;
  String tmpstr = "Right:";
  while (DEBUG_SERIAL.available())
  {
    char character = DEBUG_SERIAL.read(); // Receive a single character from the software serial port
    data.concat(character); // Add the received character to the receive buffer
    if (character == '\n')
    {
      data.trim();
      int comma_idx = data.indexOf(",");
      right = data.substring(0,comma_idx);
      left = data.substring(comma_idx+1);
      tmp = right.indexOf(":");
      right_val = right.substring(tmp+1).toFloat();
      tmp = left.indexOf(":");
      left_val = left.substring(tmp+1).toFloat();      
      
      // Clear receive buffer so we're ready to receive the next line
      data = "";
    }
  }
  // dxl.setGoalPosition(RIGHT_DXL_ID, right_val, UNIT_DEGREE);
  // dxl.setGoalPosition(LEFT_DXL_ID, left_val, UNIT_DEGREE);
  control_motor(RIGHT_DXL_ID, RIGHT_DXL_PROTOCOL_VERSION, right_val);
  control_motor(LEFT_DXL_ID, LEFT_DXL_PROTOCOL_VERSION, left_val);
  // put your main code here, to run repeatedly:
  delay(250);
  if (counter % 4 == 0) // every .5s
  {
    float tmp_left_val = read_motor(LEFT_DXL_ID, LEFT_DXL_PROTOCOL_VERSION);
    float tmp_right_val = read_motor(RIGHT_DXL_ID, RIGHT_DXL_PROTOCOL_VERSION);
  
    tmpstr.concat(tmp_left_val);
    tmpstr.concat(",Left:");
    tmpstr.concat(tmp_right_val);
    DEBUG_SERIAL.println(tmpstr);
  }
  
  counter +=1 ;
}
