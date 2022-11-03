// I2C, Master, Board 7
// Reference https://dronebotworkshop.com/i2c-arduino-arduino/
// Notes on Wire.requestFrom https://forum.arduino.cc/index.php?topic=624356.0

#include <Wire.h>

#define SLAVE_ADDR 9

#define ANSWERSIZE 5

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

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);  // start serial for output
  // Serial.println(sizeof(motor_states));
  // Serial.println(sizeof(motor_control));
  motor_control.right = -1;
}

void loop()
{
  String tmpstr = "";
  int n = Wire.requestFrom(4, sizeof(motor_control));    
  Wire.readBytes((byte*)&motor_control, sizeof(motor_control));

  motor_states.pose_right = motor_control.right + 1;
  // tmpstr = motor_control.right;
  tmpstr = "hi:";
  // tmpstr.concat(motor_control.left);
  // tmpstr.concat(",Right:");
  tmpstr.concat(motor_control.right);
  Serial.println(tmpstr);

  Wire.beginTransmission(4); // transmit to device #4
  Wire.write((byte *)&motor_states, sizeof(motor_states));  /*send string on request */
  Wire.endTransmission();    // stop transmitting

  delay(10);

}