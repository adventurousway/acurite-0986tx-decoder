#ifndef Acurite0986TXDecoder_h
#define Acurite0986TXDecoder_h

#include "Arduino.h"

#define PACKET_LENGTH 40

#define TIMING_SYNC_LOW 1630

#define TIMING_DATA_1 920

#define TIMING_NOISE_LONG 2000
#define TIMING_NOISE_SHORT 200

class Acurite0986TXDecoder
{
public:
  Acurite0986TXDecoder(int pin);

  void ICACHE_RAM_ATTR handleInterrupt();

  bool isPacketAvailable();
  void resetPacket();

  int getSensorNumber();
  int getTemperature();

private:
  int _pin;

  unsigned long _previousTime;

  int _numSyncs;

  int _packetIndex;
  bool _packet[PACKET_LENGTH];
};

#endif
