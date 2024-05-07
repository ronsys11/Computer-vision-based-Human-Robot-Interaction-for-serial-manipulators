#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Twist.h>

// Server settings
IPAddress server(192, 168, 80, 239); //192, 168, 48, 239, 32 for Madhan // 192, 168, 133, 239 for Ronak
uint16_t serverPort = 11411;
const char*  ssid = "Madhan";
const char*  password = "123456789";

//OLED 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Esp-pins
#define T_Button 14 //Input_pulldown
#define Square_Button 25
#define Triangle_Button 33
#define O_Button 32
#define Reset_Button 27
#define HapticMotor 26  //DAC // two i2c devices connencted on scl & sda

//Interrupt flags
byte Flag = 0;

// ISR
void IRAM_ATTR T_ButtonISR(){Flag = 1;}
void IRAM_ATTR Square_ButtonISR(){Flag = 2;}
void IRAM_ATTR Triangle_ButtonISR(){Flag = 3;}
void IRAM_ATTR O_ButtonISR(){Flag = 4;}
void IRAM_ATTR Reset_ButtonISR(){Flag = 5;}


void feedback_cb( const geometry_msgs::Twist& velocity_msg){

    //T_acknowledgment();// subscriber callback
}
void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { Setuptext(); }
   Disconnected();
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}

ros::NodeHandle  nh;
std_msgs::Byte trigger_msg;
ros::Publisher pointer_node("pointer_triggers", &trigger_msg );
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &feedback_cb );
void setup()
{ 
    Serial.begin(115200);
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // SSD1306_SWITCHCAPVCC = generate display voltage
    Serial.println(F("SSD1306 allocation failed")); 
    for(;;); // Don't proceed, loop forever
    } 
    Start_animation(); 
    VisionHRItext();
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(pointer_node);
    nh.subscribe(sub);
    //Esp pinMode
    pinMode(T_Button, INPUT_PULLDOWN);
    pinMode(Square_Button, INPUT_PULLDOWN);
    pinMode(Triangle_Button, INPUT_PULLDOWN);
    pinMode(O_Button, INPUT_PULLDOWN);  
    pinMode(Reset_Button, INPUT_PULLDOWN);
    pinMode(HapticMotor, OUTPUT);
    //Interrupt config
    attachInterrupt(T_Button, T_ButtonISR, FALLING);
    attachInterrupt(Square_Button, Square_ButtonISR, FALLING);
    attachInterrupt(Triangle_Button, Triangle_ButtonISR, FALLING);
    attachInterrupt(O_Button, O_ButtonISR, FALLING);
    attachInterrupt(Reset_Button, Reset_ButtonISR, FALLING);
}

void loop(){ 
  while (WiFi.status() == WL_CONNECTED) {
    while (nh.connected()) {
      switch(Flag) {
        case 1:
          trigger_msg.data = 1;
          pointer_node.publish(&trigger_msg); 
          Flag = 0;
          delay(1000);
          break;
        case 2:
          trigger_msg.data = 2;
          pointer_node.publish(&trigger_msg);
          Flag = 0;
          delay(1000);
          break;
        case 3:
          trigger_msg.data = 3;
          pointer_node.publish(&trigger_msg);
          Flag = 0; 
          delay(1000);
          break;
        case 4:
          trigger_msg.data = 4;
          pointer_node.publish(&trigger_msg);
          Flag = 0;
          delay(1000);
          break;
        case 5:
          trigger_msg.data = 5;
          pointer_node.publish(&trigger_msg);
          Flag = 0; 
          delay(1000);
          break;
      }
      nh.spinOnce();
      delay(100);
      Readydisp();
    }
    nh.spinOnce();
    delay(10);
    Disconnected();
  }
  Setuptext();
}
// Display functions
void Start_animation() {
  int16_t i;
  display.clearDisplay(); // Clear display buffer
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);
  display.clearDisplay();
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(500);
}

void VisionHRItext() {
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F(" Hello, world!"));

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,15);
  display.println(F("Vision HRi"));
  display.display();
  delay(3000);
 
}
void Setuptext(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F(">WiFi init<"));
  display.display();
}
void Readydisp(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F(" Ready # "));
  display.display();
}
void T_pressed(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setCursor(0, 0);
  display.println(F(" Selected "));
  display.display();
}
void T_acknowledgment(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setCursor(0, 0);
  display.println(F(" Recieved "));
  display.display();
  delay(1000);
}
void Disconnected(){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F(" >ROS init<"));
  display.display();
 
}

