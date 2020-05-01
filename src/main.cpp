#include <Arduino.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#include <stdio.h>
#include <string.h>

#include "Acurite0986TXDecoder.h"

#include "../config.h"

#define ONBOARD_LED 2

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

Acurite0986TXDecoder decoder = Acurite0986TXDecoder(RF_DATA_PIN);

void ICACHE_RAM_ATTR handleInterrupt()
{
  decoder.handleInterrupt();
}

void connectToWiFi()
{
  if (!WiFi.isConnected())
  {
    Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.print("WiFi connected, IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;

  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi connection lost");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;

  default:
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.printf("Connected to MQTT broker: %s:%d (%s)\n", MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setClientId(MQTT_CLIENT_ID);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  WiFi.onEvent(WiFiEvent);

  connectToWiFi();

  pinMode(digitalPinToInterrupt(RF_DATA_PIN), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF_DATA_PIN), handleInterrupt, CHANGE);

  pinMode(ONBOARD_LED, OUTPUT);
}

void loop()
{
  if (decoder.isPacketAvailable())
  {
    digitalWrite(ONBOARD_LED, HIGH);

    int sensorNumber = decoder.getSensorNumber();
    int temperature = decoder.getTemperature();

    decoder.resetPacket();

    // Serial.printf("Temperature from sensor #%d: %d°F\n", sensorNumber, temperature);

    char payload[50];
    sprintf(payload, "{\"sid\":%d,\"tem\":%d,\"unt\":\"F\"}", sensorNumber, temperature);

    char topic[100];
    sprintf(topic, "smartrvlab/things/fridge-controller/inside-temperature/%d", sensorNumber);

    mqttClient.publish(topic, 1, true, payload);

    digitalWrite(ONBOARD_LED, LOW);
  }
}
