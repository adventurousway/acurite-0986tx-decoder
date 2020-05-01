#include <Arduino.h>
#include <stdio.h>

#include "Acurite0986TXDecoder.h"

#define RF_DATA_PIN 32

Acurite0986TXDecoder decoder = Acurite0986TXDecoder(RF_DATA_PIN);

void ICACHE_RAM_ATTR handleInterrupt() {
  decoder.handleInterrupt();
}

void setup()
{
  Serial.begin(9600);

  pinMode(digitalPinToInterrupt(RF_DATA_PIN), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF_DATA_PIN), handleInterrupt, CHANGE);
}

void loop()
{
  if (decoder.isPacketAvailable())
  {
    int sensorNumber = decoder.getSensorNumber();
    int temperature = decoder.getTemperature();

    decoder.resetPacket();

    printf("Temperature from sensor #%d: %d°F\n", sensorNumber, temperature);
  }
}
