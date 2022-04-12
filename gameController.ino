#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

ros::NodeHandle nh;
String var = "r"; //begin as reset command

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void messageCb(const std_msgs::String &msg)
{
  var=msg.data;
  char text = *var.c_str();
  
  radio.write(&text, 1);
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageCb);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop()
{
  nh.spinOnce();
  delay(200);
}
