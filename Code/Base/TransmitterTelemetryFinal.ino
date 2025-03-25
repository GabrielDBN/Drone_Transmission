//DiY Electronics//
//~~~~~~~~~~~~~~//

//Transmitter Code

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define RXD2 16
#define TXD2 17
Adafruit_BMP280 bmp;
HardwareSerial neogps(1);
TinyGPSPlus gps;

//Fill the MAC adress of the reciever board
uint8_t RxMACaddress[] = {0x40, 0x22, 0xD8, 0xE7, 0x79, 0x54};

//defining the data messages we want to sent to reciever
typedef struct struct_message
{
  int bat;
  float a;
  float b;
  float c;
  int d;
  float e;
 
}struct_message;
struct_message sentData;

//-------------------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) //callback function
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//-------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
   neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);
  WiFi.mode(WIFI_STA);
  //------------------------------------------------------------------------------------
  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //-------------------------------------------------------------------------------------
  esp_now_register_send_cb(OnDataSent);
  //-------------------------------------------------------------------------------------
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, RxMACaddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //-------------------------------------------------------------------------------------
  if(esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}
//======================================================================================
void loop()
{
  sentData.bat = analogRead(A0);  //reading voltage on 10k resistor and sending to reciever
  
  float temp = bmp.readTemperature(); //reading temperature and sending to reciever
  sentData.a = temp;

  float alt = bmp.readAltitude(1013.25); //adjust barometric pressure in your location
  sentData.b = alt;

  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //reading gps data and sending to reciever
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
    float sp = gps.speed.kmph();
    sentData.c = sp;
    int sat = gps.satellites.value();
    sentData.d = sat;
    float alt2 = gps.altitude.meters();
    sentData.e = alt2;
  }
  
  //-------------------------------------------------------------------------------------
  esp_err_t result = esp_now_send(RxMACaddress, (uint8_t *) &sentData, sizeof(sentData));
  //-------------------------------------------------------------------------------------
  if (result == ESP_OK) Serial.println("Sent with success");
  else Serial.println("Error sending the data");
  //-------------------------------------------------------------------------------------
  delay(500);
}
