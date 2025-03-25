//DiY Electronics//
//~~~~~~~~~~~~~~//

//Reciever Code//

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter batkf(5, 0.002, 0.003);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//Defining data messages we want to recieve
typedef struct struct_message
{
  int bat;
  float a;
  float b;
  float c;
  int d;
  float e;
}struct_message;
struct_message receivedData;
//-------------------------------------------------------------------------------------
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&receivedData, incomingData, sizeof(receivedData));
}
//-------------------------------------------------------------------------------------


void setup()
{
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  //-------------------------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
//-------------------------------------------------------------------------------------
void loop() //recieving all the data and displaying it on the screen
{

  float estimated_bat = batkf.updateEstimate(receivedData.bat);
  display.setTextColor(WHITE); display.clearDisplay();
  display.setTextSize(1); display.setCursor(0,0); display.print("Battery:");

  display.print(int(estimated_bat*6*3.3*100/(17.5*4095))); //suitable for 4s battery. ajdust if you have different battery.
  display.print("%");


  display.setTextSize(1); display.setCursor(0,10); display.print("Temp:");
  display.print(receivedData.a); display.print("C");

  display.setTextSize(1); display.setCursor(0,20); display.print("Altitude:");
  display.print(receivedData.b); display.print("m");

  display.setTextSize(1); display.setCursor(0,30); display.print("speed:");
  display.print(receivedData.c); display.print("km/h");

  display.setTextSize(1); display.setCursor(0,40); display.print("sat num:" );
  display.print(receivedData.d); 

  display.setTextSize(1); display.setCursor(0,50); display.print("alt2:");
  display.print(receivedData.e); display.print("m");
  
  display.display();
  delay(500);
}
