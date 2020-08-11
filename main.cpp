#include <ESP8266WiFi.h>
#include <ros.h>
#include<Adafruit_NeoPixel.h>
#include<geometry_msgs/Vector3.h>
#include <Adafruit_PWMServoDriver.h>
#include<math.h>
// motor
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int DC_OFF = 0;
const int DC_MIN = 1096;
const int DC_MAX = 4096;
const int DC_DIFF = DC_MAX - DC_MIN;
const int MOTORS = 4;
const int FREQUENCY = 100;
// left  to  right
const int forward[] = {13, 15, 1, 2};
const int backward[] = {12, 14, 0, 3};

// LED
const int LEDS = 16;
Adafruit_NeoPixel ring = Adafruit_NeoPixel(LEDS, D4, NEO_GRB + NEO_KHZ800);

// WLAN
const char* ssid = "Felsit";
const char* password = "3sUdo#k8-9";

// ROS
IPAddress server(192, 168, 178, 3); // ip of your ROS server
IPAddress ip_address;
WiFiClient client;

bool activated = false;
int timeout = 0;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    return client.read(); //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
};

void indexwise_add(const double a[],const double b[], double ans[]){
  for (int i = 0; i < MOTORS; i++)
    ans[i] = a[i] + b[i];
}

void reset_engine(){
  for(int i = 0; i < LEDS; i++){
    pwm.setPin(i, DC_OFF);
    ring.setPixelColor(i, ring.Color(0, 0, 0));
    ring.show();
  }
}

void breaking(){
  //break
  for (int i = 0; i < MOTORS; i++){
    pwm.setPin(forward[i], DC_OFF);
    pwm.setPin(backward[i], DC_OFF);
  }
  for (int i = 0; i < LEDS; i++)
    ring.setPixelColor(i, ring.Color(255, 0, 0));
  ring.show();
  activated = true;
}

void directional_LEDs(const geometry_msgs::Vector3& directions){
  //directional LED
  int i = -1;
  if(direction.x == 0){
    if(direction.y > 0)
      i = 8;
    else
      i = 0;
  }
  else if (direction.y == 0){
    if (direction.x > 0)
      i = 4;
    else
      i = 12;
  }
  else{
  double rad = atan(direction.y/direction.x);
  i = ((int)(16 + rad * 16 / (2*M_PI)))%16;
  }
  // intensity according to aceleration
  double l = sqrt(direction.x*direction.x + direction.y*direction.y);
  Serial.println(l);
  int brightness = 0;
  if (l >= 1)
    brightness = 255;
  else
    brightness = 255*l;
  ring.setPixelColor(i, ring.Color(0, brightness, 0));
  ring.show();
}

void set_acceleration(const double acc[]){
  for (int i = 0; i < MOTORS; i++){
    if (acc[i] >= 0){
      pwm.setPin(forward[i], DC_MIN + acc[i]*DC_DIFF);
      pwm.setPin(backward[i], DC_OFF);
    }
    else{
      pwm.setPin(backward[i], DC_MIN - acc[i]*DC_DIFF);
      pwm.setPin(forward[i], DC_OFF);
    }
  }
}

void directionsCallback(const geometry_msgs::Vector3& direction) {
  Serial.print("direction");
  // led closest to direction
  
  if(direction.x == 0 && direction.y == 0){
      breaking();
  }
  else{
    double x = direction.x;
    double y = direction.y;
    double xy = abs(direction.x) + abs(direction.y);
    if(xy > 1){
      x = direction.x / xy;
      y = direction.y / xy;
    }
    double acceleration_front[] = {y, y, y, y};
    double acceleration_right[] = {x, x, -x, -x};
    double acceleration[] = {0, 0, 0, 0};
    indexwise_add(acceleration_front, acceleration_right, acceleration);
    set_acceleration(acceleration);
    directional_LEDs(direction);  
  }
  activated = true;
  Serial.printf("activated");
  timeout = 2;
}

ros::Subscriber<geometry_msgs::Vector3> sub("directions", &directionsCallback);

ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print("Could not connect to roscore at "); Serial.println(ssid);
  }
        
  Serial.println("Connected!");
  Serial.print("my address is ");
  Serial.println(WiFi.localIP());
}

void setupPWM(){
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

void setup() {
  Serial.begin(115200);
  setupPWM();
  ring.begin();
  reset_engine();
  setupWiFi();
  delay(1000);
  nh.initNode();
  nh.subscribe(sub);
  timeout = 2;
}

void loop() {
  nh.spinOnce();
  Serial.println("post spin");
  delay(200);
  if (timeout > 0)
    timeout -= 1;
  Serial.print("timeout = ");
  Serial.println(timeout);
  
  if (timeout <= 0 && activated){
    for (int i = 0; i < 16; i++)
      ring.setPixelColor(i, ring.Color(0, 0, 0));
    ring.show();
    breaking();
    activated = false;
  }
}

