#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

const int startSpeed = 1050;

int x = startSpeed;

void setup() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  dac.begin(0x62);
  dac.setVoltage(startSpeed, false);
}

void loop() {
  
  if (radio.available()){
    char text[32] = "";
    radio.read(&text, sizeof(text));

    if (*text == 'w'){
      dac.setVoltage(x, false);
      if (x < 3000){
        x = x + 30;
      }
      delay(100);
      dac.setVoltage(startSpeed, false);
    }
    else if(*text == 's'){
      x = startSpeed;
    }
  }
}
