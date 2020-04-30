#include <Arduino.h>

#define RF_DATA_PIN 32

#define TIMING_SYNC_LOW 1630

#define TIMING_DATA_1 920

#define TIMING_NOISE_LONG 2000
#define TIMING_NOISE_SHORT 200

#define PACKET_LENGTH 40

unsigned long previousTime = 0;
int duration = 0;

int countLongSyncs = 0;
int bitIndex = 0;

bool hasSynced = false;

bool bitArray[PACKET_LENGTH];
bool isMeasurementAvailable = false;

void resetCounters()
{
  countLongSyncs = 0;
  hasSynced = false;
}

void resetBitArray()
{
  bitIndex = 0;
  for (int i = 0; i < PACKET_LENGTH; i++)
  {
    bitArray[i] = 0;
  }
  isMeasurementAvailable = false;
}

void ICACHE_RAM_ATTR handleInterrupt()
{
  unsigned long time = micros();

  duration = time - previousTime;
  previousTime = time;

  int state = digitalRead(RF_DATA_PIN);

  if (!hasSynced)
  {
    if (duration > TIMING_NOISE_LONG || duration < TIMING_NOISE_SHORT)
    {
      return;
    }

    if (state == LOW)
    {
      if (abs(duration - TIMING_SYNC_LOW) < 20)
      {
        countLongSyncs++;

        if (countLongSyncs == 4)
        {
          hasSynced = true;
          isMeasurementAvailable = false;
          resetBitArray();
        }

        return;
      }
      else
      {
        resetCounters();
      }
    }
    return;
  }

  if (state == HIGH)
  {
    if (bitIndex == 0 && duration > 1200)
    {
      // this is the final high from the SYNC
      return;
    }

    if (bitIndex == PACKET_LENGTH)
    {
      resetCounters();
      isMeasurementAvailable = true;
      return;
    }

    bitArray[bitIndex] = abs(duration - TIMING_DATA_1) < 100;

    bitIndex++;
  }
}

int getSensorNumber(bool packet[PACKET_LENGTH])
{
  return packet[24] + 1;
}

int getTemperature(bool packet[PACKET_LENGTH])
{
  int temperature = 0;

  for (int i = 0; i < 7; i++)
  {
    temperature |= packet[i] << i;
  }

  return temperature;
}

void setup()
{
  Serial.begin(9600);

  resetBitArray();

  pinMode(digitalPinToInterrupt(RF_DATA_PIN), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF_DATA_PIN), handleInterrupt, CHANGE);
}

void loop()
{
  if (isMeasurementAvailable)
  {
    bool packet[PACKET_LENGTH];

    std::copy(std::begin(bitArray), std::end(bitArray), std::begin(packet));
    resetBitArray();

    int sensorNumber = getSensorNumber(packet);
    int temperature = getTemperature(packet);

    Serial.print("Reading from sensor #");
    Serial.print(sensorNumber);
    Serial.print(": ");
    Serial.print(temperature);
    Serial.println("°F");
  }
}
