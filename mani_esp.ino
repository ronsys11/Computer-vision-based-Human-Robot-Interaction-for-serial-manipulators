#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <ESP32Servo.h>

// Server settings
IPAddress server(192,168,43,151); //192, 168, 48, 239, 32 for Madhan // 192, 168, 133, 239 for Ronak
uint16_t serverPort = 11433;
const char*  ssid = "Madhan";
const char*  password = "123456789";

int joint1 = 0;
int joint2 = 0;
int joint3 = 0;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;


void frame_cb(const std_msgs::Int16MultiArray& value_frame) {
  joint1 = value_frame.data[0];
  joint2 = value_frame.data[1];
  joint3 = value_frame.data[2];
  Serial.println(joint1);
  Serial.println(joint2);
  Serial.println(joint3);

}


void setupWiFi(){  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { Serial.print(".");delay(500);}
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}


ros::NodeHandle nh;
std_msgs::Int16MultiArray value_frame;
ros::Subscriber<std_msgs::Int16MultiArray> frame_sub("/tvec2", &frame_cb);



void setup() {
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(frame_sub);
  myservo1.attach(12);
  myservo2.attach(14);
  myservo3.attach(27);
  myservo4.attach(26);
}
void loop(){
  while (nh.connected()){
    myservo1.write(joint1);
    myservo2.write(joint2); 
    myservo3.write(180 - joint2);  
    myservo4.write(joint3); 
    nh.spinOnce();
  }
  nh.spinOnce();
}