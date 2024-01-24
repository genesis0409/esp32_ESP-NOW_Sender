/*
// MAC Address

#include <Arduino.h>
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
#include "WiFi.h"

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}

void loop()
{
}
*/

// ESP32 Receiver Sketch (ESP-NOW)
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
/*
  esp_now_init() : ESP-NOW 초기화 ESP-NOW 초기화하기 전에 Wi-Fi 초기화해야 함
  esp_now_add_peer() : 장치를 페어링하고 피어 MAC 주소를 인수로 전달
  esp_now_send() : ESP-NOW 통해 데이터 송신
  esp_now_register_send_cb() : 데이터 [전송 시] 실행되는 콜백 함수 등록. 메시지가 전송되면 함수가 호출됨. 전달 성공했는지 여부를 반환.
  esp_now_register_rcv_cb() : 데이터 [수신 시] 실행되는 콜백 함수를 등록. ESP-NOW를 통해 데이터가 수신되면 함수가 호출됨.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <esp_wifi.h>

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// Digital pin connected to the DHT sensor
#define DHTPIN 4

// Uncomment the type of sensor in use:
// #define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x94, 0xB5, 0x55, 0x26, 0xA2, 0x68}; // Receiver Mac : 94:B5:55:26:A2:68

// 보내려는 데이터 유형이 포함된 구조체
// Must match the receiver structure
typedef struct struct_message
{
  // char a[32];
  // int b;
  // float c;
  // bool d;
  int id;
  float temp;
  float hum;
  unsigned int readingId;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo; // peer의 정보를 저장하는 esp_now_peer_info_t형 변수

unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 3000;       // Interval at which to publish sensor readings

// 전송된 판독값 수를 추적
unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "dinfo";
// 수신기의 wifi 채널을 가져옴 -> 동일한 채널을 송신기 보드에 자동 할당 가능 <유용>
int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks())
  {
    for (uint8_t i = 0; i < n; i++)
    {
      if (!strcmp(ssid, WiFi.SSID(i).c_str()))
      {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

float readDHTTemperature()
{
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  // float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else
  {
    Serial.println(t);
    return t;
  }
}

float readDHTHumidity()
{
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h))
  {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else
  {
    Serial.println(h);
    return h;
  }
}

// 콜백 함수 정의 callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  // 메시지 전달 여부 출력
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  dht.begin();

  // Set device as a Wi-Fi Station
  // 스테이션 모드
  WiFi.mode(WIFI_STA);
  /*wifi/web 비활성화
  // set channel: 수신기의 wifi 채널과 일치하도록 채널 설정
  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after
*/
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
    Serial.println("ESP-NOW Init OK");
  // 초기화 후 메시지 전송 시 호출될 콜백 함수 등록 ->
  // OnDataSent()

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb() : 데이터 [전송 시] 실행되는 콜백 함수 등록. 메시지가 전송되면 함수가 호출됨. 전달 성공했는지 여부를 반환.
  esp_now_register_send_cb(OnDataSent);

  // 데이터를 전송하려면 다른 ESP-NOW 장치와 페어링해야 함
  // Register peer

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  // Add peer
  // esp_now_add_peer() : 장치를 페어링하고 피어 MAC 주소를 인수로 전달
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // 가장 최신 판독값을 저장
    previousMillis = currentMillis;
    // Set values to send
    myData.id = BOARD_ID;
    myData.temp = readDHTTemperature();
    myData.hum = readDHTHumidity();
    myData.readingId = readingId++;

    // [Send message] via ESP-NOW
    // esp_now_send() : ESP-NOW 통해 데이터 송신
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
      Serial.println(result);
    }
  }
}