#include <ros.h>
#include <std_msgs/String.h>

#include <SoftwareSerial.h>
#define rxWheelPin 3  // pin 3 connects to smcSerial TX  (not used in this example)
#define txWheelPin 4  // pin 4 connects to smcSerial RX
SoftwareSerial smcWheelSerial = SoftwareSerial(rxWheelPin, txWheelPin);

#define rxBrakePin 6  // pin 3 connects to smcSerial TX  (not used in this example)
#define txBrakePin 7  // pin 4 connects to smcSerial RX
SoftwareSerial smcBrakeSerial = SoftwareSerial(rxBrakePin, txBrakePin);

#define VCC2 13 //second 5V
#define GND2 12 //second GND

const int analogWheelInPin = A1;
const int analogBrakeInPin = A2;
int sensorWheelValue = 0;        // value read from the feedback potentiometer
int sensorBrakeValue = 0;        // value read from the feedback potentiometer

ros::NodeHandle nh;
String var = "r";
  
// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcWheelSerial.write(0x83);
  smcBrakeSerial.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setWheelMotorSpeed(int speed)
{
  if (speed < 0)
  {
    smcWheelSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcWheelSerial.write(0x85);  // motor forward command
  }
  smcWheelSerial.write(speed & 0x1F);
  smcWheelSerial.write(speed >> 5);
}

// speed should be a number from -3200 to 3200
void setBrakeMotorSpeed(int speed)
{
  if (speed < 0)
  {
    smcBrakeSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcBrakeSerial.write(0x85);  // motor forward command
  }
  smcBrakeSerial.write(speed & 0x1F);
  smcBrakeSerial.write(speed >> 5);
}

void messageCb(const std_msgs::String &msg)
{
  var=msg.data;
  
  sensorWheelValue = analogRead(analogWheelInPin);
  sensorBrakeValue = analogRead(analogBrakeInPin);
  
  if (sensorWheelValue < 750 && var == "d") {
    setWheelMotorSpeed(1000);  // right
    delay(100);
    setWheelMotorSpeed(0);
  }
  else if (sensorWheelValue > 145 && sensorWheelValue < 1000 && var == "a") {
    setWheelMotorSpeed(-1000);  // left
    delay(100);
    setWheelMotorSpeed(0);
  }
  else if (sensorBrakeValue < 500 && var == "s") {
    setBrakeMotorSpeed(-3200);
    delay(100);
    setBrakeMotorSpeed(0);
  }
}

ros::Subscriber<std_msgs::String> sub("motor", &messageCb);

void setup()
{
  pinMode(VCC2,OUTPUT);
  digitalWrite(VCC2, HIGH); // set the pin as HIGH so it acts as 5V

  pinMode(GND2,OUTPUT);
  digitalWrite(GND2, LOW); // set the pin as LOW so it acts as GND1
  
   // initialize software serial object with baud rate of 19.2 kbps
  smcWheelSerial.begin(19200);
  smcBrakeSerial.begin(19200);
 
  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);
 
  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  smcWheelSerial.write(0xAA);  // send baud-indicator byte
  smcBrakeSerial.write(0xAA);  // send baud-indicator byte
 
  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run
  exitSafeStart();  // clear the safe-start violation and let the motor run

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  setBrakeMotorSpeed(3200);
  nh.spinOnce();
  delay(200);
}
