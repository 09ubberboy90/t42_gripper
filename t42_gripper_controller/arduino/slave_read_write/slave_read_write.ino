// I2C, Slave, Board 8

#include <Wire.h>
#include <Stream.h>
typedef struct {
  int right;
  int left;
} motor_control_t;

typedef struct {
  int load_right;
  int load_left;
  int pose_right;
  int pose_left;
} motor_states_t;

volatile motor_states_t motor_states;
volatile motor_control_t motor_control;
bool newData = false;
bool error = false;

int counter = 0;
String tmpstr;
String data;
String right,left;
int tmp;

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(2000000); // start serial for output
  // Serial.println(sizeof(motor_states));
  // Serial.println(sizeof(motor_control));
}

void loop()
{
  while (Serial.available())
  {
    data = Serial.readStringUntil('\n');
    data.trim();
    int comma_idx = data.indexOf(",");
    right = data.substring(0,comma_idx);
    left = data.substring(comma_idx+1);
    tmp = right.indexOf(":");
    motor_control.right = right.substring(tmp+1).toInt();
    tmp = left.indexOf(":");
    motor_control.left = left.substring(tmp+1).toInt();      
    // Clear receive buffer so we're ready to receive the next line
  }

  if(newData)
  {
    tmpstr = "Load/Right:";
    tmpstr.concat(motor_states.load_right);
    tmpstr.concat(",Left:");
    tmpstr.concat(motor_states.load_left);
    tmpstr.concat("\r\nPose/Right:");
    tmpstr.concat(motor_states.pose_right);
    tmpstr.concat(",Left:");
    tmpstr.concat(motor_states.pose_left);
    Serial.println(tmpstr);
    newData = false;
  }
  delay(10);
  
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  if(newData)
  {
    error = true;
  }
  Wire.readBytes((byte *)&motor_states, sizeof(motor_states));
  newData = true;
}

void requestEvent()
{
  Wire.write((byte *)&motor_control, sizeof(motor_control));  /*send string on request */
}
