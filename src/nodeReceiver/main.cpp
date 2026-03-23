// Receiver node code for LoRa communication example

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define SS 5
#define RST 14
#define DIO0 26

void setup()
{

  Serial.begin(115200);

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6))
  {
    Serial.println("LoRa init failed");
    while (1)
      ;
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);

  Serial.println("NodeReceiver Ready");
}

void loop()
{

  int packetSize = LoRa.parsePacket();

  if (packetSize)
  {

    Serial.print("Received: ");

    while (LoRa.available())
    {
      Serial.print((char)LoRa.read());
    }

    Serial.println();

    delay(500);

    Serial.println("Send Reply");

    LoRa.beginPacket();
    LoRa.print("Hello Node1");
    LoRa.endPacket();
  }
}