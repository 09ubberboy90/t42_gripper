// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example Environment
//
// - DYNAMIXEL: X series
//              ID = 1 & 2, Baudrate = 57600bps, DYNAMIXEL Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/
//
// Author: David Park

#include <Dynamixel2Arduino.h>
#include <stdlib.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif


/* syncRead
  Structures containing the necessary information to process the 'syncRead' packet.

  typedef struct XELInfoSyncRead{
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed)) XELInfoSyncRead_t;

  typedef struct InfoSyncReadInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncReadInst_t;
*/

/* syncWrite
  Structures containing the necessary information to process the 'syncWrite' packet.

  typedef struct XELInfoSyncWrite{
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed)) XELInfoSyncWrite_t;

  typedef struct InfoSyncWriteInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncWriteInst_t;
*/

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR_POSE = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN_POSE = 4;
// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR_LOAD = 126;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN_LOAD = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_data;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data_load[DXL_ID_CNT];
sr_data_t sr_data_pose[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos_pose;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_pose[DXL_ID_CNT];

DYNAMIXEL::InfoSyncReadInst_t sr_infos_load;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr_load[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position[2] = {0, 0};
uint8_t goal_position_index = 0;
int counter = 0;
String right, left;

void fill_sr_info(DYNAMIXEL::InfoSyncReadInst_t *sr_infos, DYNAMIXEL::XELInfoSyncRead_t *info_xels_sr, sr_data_t *sr_data, uint16_t addr, uint16_t addr_length){
  sr_infos->packet.p_buf = user_pkt_buf;
  sr_infos->packet.buf_capacity = user_pkt_buf_cap;
  sr_infos->packet.is_completed = false;
  sr_infos->addr = addr;
  sr_infos->addr_length = addr_length;
  sr_infos->p_xels = info_xels_sr;
  sr_infos->xel_count = 0;  

  for(uint8_t i = 0; i < DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos->xel_count++;
  }
  sr_infos->is_info_changed = true;

}

void setup() {
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(9600);
  dxl.begin(115200);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

// Prepare the SyncRead structure
  for(i = 0; i < DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  fill_sr_info(&sr_infos_pose, info_xels_sr_pose, sr_data_pose, SR_START_ADDR_POSE, SR_ADDR_LEN_POSE);
  fill_sr_info(&sr_infos_load, info_xels_sr_load, sr_data_load, SR_START_ADDR_LOAD, SR_ADDR_LEN_LOAD);

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

void loop() {
  String data = "";
  int tmp;
  String tmpstr = "";

  // put your main code here, to run repeatedly:
  uint8_t i, recv_cnt;
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
      sw_data[1].goal_position = right.substring(tmp+1).toInt();
      tmp = left.indexOf(":");
      sw_data[0].goal_position = left.substring(tmp+1).toInt();      
      
      // Clear receive buffer so we're ready to receive the next line
      data = "";
    }
  }
  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  
  // Build a SyncWrite Packet and transmit to DYNAMIXEL  
  dxl.syncWrite(&sw_infos);
  delay(100);



  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  if (counter % 6 == 0) // every .5s
  { 
    recv_cnt = dxl.syncRead(&sr_infos_load, 100);
    tmpstr.concat("Load/");
    if(recv_cnt > 0)
    {
      tmpstr.concat("Right:");
      tmpstr.concat(sr_data_load[1].present_data);
      tmpstr.concat(",Left:");
      tmpstr.concat(sr_data_load[0].present_data);
    }
    tmpstr.concat("/Received:");
    tmpstr.concat(recv_cnt);
    DEBUG_SERIAL.println(tmpstr);
    // tmpstr.concat("/Error0:");
    // tmpstr.concat(info_xels_sr_load[0].error);
    // tmpstr.concat("/Error1:");
    // tmpstr.concat(info_xels_sr_load[1].error);
    // tmpstr.concat("/Error2:");
    // tmpstr.concat(dxl.getLastLibErrCode());

  }
  if (counter % 6 == 3) // every .5s
  { 
    recv_cnt = dxl.syncRead(&sr_infos_pose, 100);
    tmpstr.concat("Pose/");
    if(recv_cnt > 0)
    {
      tmpstr.concat("Right:");
      tmpstr.concat(sr_data_pose[1].present_data);
      tmpstr.concat(",Left:");
      tmpstr.concat(sr_data_pose[0].present_data);
      DEBUG_SERIAL.println(tmpstr);
    }
    tmpstr.concat("/Received:");
    tmpstr.concat(recv_cnt);
    DEBUG_SERIAL.println(tmpstr);

  }
  counter +=1;

}
