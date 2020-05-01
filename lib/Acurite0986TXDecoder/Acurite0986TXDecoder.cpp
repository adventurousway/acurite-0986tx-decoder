#include "Arduino.h"
#include "Acurite0986TXDecoder.h"

Acurite0986TXDecoder::Acurite0986TXDecoder(int pin)
{
  _pin = pin;

  _previousTime = 0;

  resetPacket();
}

void Acurite0986TXDecoder::resetPacket()
{
  for (int i = 0; i < PACKET_LENGTH; i++)
  {
    _packet[i] = 0;
  }

  _numSyncs = 0;
  _packetIndex = 0;
}

int Acurite0986TXDecoder::getSensorNumber()
{
  return _packet[24] + 1;
}

int Acurite0986TXDecoder::getTemperature()
{
  int temperature = 0;

  for (int i = 0; i < 7; i++)
  {
    temperature |= _packet[i] << i;
  }

  return temperature;
}

bool Acurite0986TXDecoder::isPacketAvailable()
{
  return _packetIndex == PACKET_LENGTH;
}

void ICACHE_RAM_ATTR Acurite0986TXDecoder::handleInterrupt()
{
  unsigned long time = micros();

  if (isPacketAvailable())
  {
    return;
  }

  int _duration = time - _previousTime;
  _previousTime = time;

  int state = digitalRead(_pin);

  if (_numSyncs != 4)
  {
    if (_duration > TIMING_NOISE_LONG || _duration < TIMING_NOISE_SHORT)
    {
      return;
    }

    if (state == LOW)
    {
      if (abs(_duration - TIMING_SYNC_LOW) < 20)
      {
        _numSyncs++;
      }
      else
      {
        _numSyncs = 0;
      }
    }
    return;
  }

  if (state == HIGH)
  {
    if (_packetIndex == 0 && _duration > 1200)
    {
      // this is the final high from the SYNC
      return;
    }

    _packet[_packetIndex] = abs(_duration - TIMING_DATA_1) < 100;

    _packetIndex++;
  }
}
