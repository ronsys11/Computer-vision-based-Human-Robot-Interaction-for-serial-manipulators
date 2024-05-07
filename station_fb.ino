#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <Adafruit_NeoPixel.h>
#include <geometry_msgs/Twist.h>

// Server settings
IPAddress server(192,168, 80, 239); //192, 168, 48, 239, 32 for Madhan // 192, 168, 133, 239 for Ronak
uint16_t serverPort = 11433;
const char*  ssid = "Madhan";
const char*  password = "123456789";

float frame_value = 0.0;
float ws_value = 0.0;

#define PIN 15
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

void frrr_cb(const geometry_msgs::Twist& value_frrr) {
  frame_value = value_frrr.linear.x;
  //z = value_frame.angular.z;
}
void ws_cb(const geometry_msgs::Twist& value_ws) {
  ws_value = value_ws.linear.x;
  //z = value_ws.angular.z;
}

void setupWiFi(){  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { rainbow(20);  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}


ros::NodeHandle nh;
geometry_msgs::Twist value_frrr;
geometry_msgs::Twist value_ws;
ros::Subscriber<geometry_msgs::Twist> frrr_sub("/frame_fb_topic", &frrr_cb);
ros::Subscriber<geometry_msgs::Twist> ws_sub("/ws_fb_topic", &ws_cb);

void setup() {
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(frrr_sub);
  nh.subscribe(ws_sub);
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
}

void loop(){
  while (WiFi.status() == WL_CONNECTED) {
    while (nh.connected()) {
      while (frame_value == 2.0){
        if(ws_value == 2.0){
          colorWipe(strip.Color(0, 255, 0), 1); // Green within WS
          ws_value = 1.0;
          delay(1000);
        }
        else if(ws_value ==0.0){
          colorWipe(strip.Color(255, 0, 0), 1); // Red Outside WS
          ws_value =1.0;
          delay(1000);
        }
        colorWipe(strip.Color(0, 0, 255), 1); // Blue
        nh.spinOnce();
        delay(100);
      }
      if(ws_value == 2.0){
        colorWipe(strip.Color(0, 255, 0), 1); // blue
        ws_value = 1.0;
        delay(1000);
      }
      else if(ws_value ==0.0){
        colorWipe(strip.Color(255, 0, 0), 1); // red
        ws_value =1.0;
        delay(1000);
      }
      colorWipe(strip.Color(0, 0, 0), 1); // off
      delay(200);
      nh.spinOnce();
    }
    colorWipe(strip.Color(255, 255, 255), 1); // White
    nh.spinOnce();
  }
  rainbow(20);
}

void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}