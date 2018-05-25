#include <NewPing.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

SoftwareSerial mySerial(11, 12);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define TRIGGER_PIN2  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE2 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar2(TRIGGER_PIN2 , ECHO_PIN2 , MAX_DISTANCE2);

#define TRIGGER_PIN3  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE3 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar3(TRIGGER_PIN3 , ECHO_PIN3 , MAX_DISTANCE3);

const int trigPin = 7;
const int echoPin = 4;

const int trigPin2 = 10;
const int echoPin2 = 8;

const int controlPin1 = 6;
const int controlPin2 = 3;

const int enablePin2 = 13;
const int controlPin3 = A2;
const int controlPin4 = A1;

const int buttonPin = 2;

int t;
int distance; //straight ahead
int distance2; //left
int distance3; //right

boolean usingInterrupt = false;
void useInterrupt(boolean); 

int buttonState = 0;
int y;
int a;

float XC = 2.5;
float YC = 0.72;
float ZC = 29.2;

double goalLat = 0;
double goalLong = 0;

void setup(){
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);

  pinMode(controlPin3, OUTPUT);
  pinMode(controlPin4, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  
  pinMode(buttonPin, INPUT);
  
  pinMode(A3, OUTPUT);
  pinMode(A0, OUTPUT);
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  
  Serial.begin(9600);
  Serial.println("Compass Test"); Serial.println("");
  
  /* Initialise the sensor */
  mag.begin();
  accel.begin();
}
float Pi = 3.14159;
  
sensors_event_t magEvent; 
sensors_event_t accelEvent;

float Pitch(){ 
 float total = 0;
 for(int i = 0; i > 9; i++){
   accel.getEvent(&accelEvent);
   total = total + accelEvent.acceleration.x;
  }   
  float Pitch; 
  Pitch = asin(-total / 8 / 9.8);
  return Pitch;
}
  
float Roll(){
  float total = 0;
  for(int i = 0; i > 9; i++){
    accel.getEvent(&accelEvent);
    total = total + accelEvent.acceleration.y;
  }   
    float Roll;
    Roll = asin(total / 8 / 9.8 / cos(Pitch()) );
    return Roll;
  }
  
  float compHeading(){
    float X = (magEvent.magnetic.x + XC) * cos (Pitch()) + (magEvent.magnetic.z + ZC) * sin (Pitch());
    float Y = (magEvent.magnetic.x + XC) * sin (Roll()) * sin (Pitch()) + (magEvent.magnetic.y + YC) * cos (Roll()) - (magEvent.magnetic.z + ZC) * sin (Roll()) * cos (Pitch());
    float head2 = atan2 (Y,X); 
  
  if(head2 < 0)
    head2 += 2*Pi;
  if(head2 > 2*Pi)
    head2 -= 2*Pi;
  head2 = head2 * 180/Pi - 11;
  return head2;
}

float heading(){
  float head2 = atan2 ((magEvent.magnetic.y + YC), (magEvent.magnetic.x + XC));
  if(head2 < 0)
    head2 += 2*Pi;
  if(head2 > 2*Pi)
    head2 -= 2*Pi;
  head2 = head2 * 180/Pi;
  return head2;
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
 
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
   
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop(){
  mag.getEvent(&magEvent);
  accel.getEvent(&accelEvent);
  readGPS();
  if (GPS.fix == true){
    buttonState = digitalRead(buttonPin);
    if(buttonState == HIGH){
      Serial.println("Geting Coordinates");
      readGPS();
      goalLat = GPS.latitudeDegrees;
      goalLong = GPS.longitudeDegrees;      
      if(goalLat > 0 || goalLat < 0){
        digitalWrite(A0, HIGH);
        digitalWrite(A3, HIGH);
        delay(1000);
        digitalWrite(A0, LOW);
        digitalWrite(A3, LOW);
        delay(500);
        digitalWrite(A0, HIGH);
        digitalWrite(A3, HIGH);
        delay(1000);
        digitalWrite(A0, LOW);
        digitalWrite(A3, LOW);
        Serial.println(goalLat);
        Serial.println(goalLong);
      }
    }
    
     
  int A = compHeading() - courseTo(GPS.latitudeDegrees,GPS.longitudeDegrees,goalLat,goalLong);
  
  if (distanceBetween(GPS.latitudeDegrees,GPS.longitudeDegrees,goalLat,goalLong) > 5){
    
  distance = sonar.ping_cm();// Forward Sensor
  if(distance == 0) distance = 300;  
  
  distance2 = sonar2.ping_cm(); // Left Sensor
  if(distance2 == 0) distance2 = 300;  
  
  distance3 = sonar3.ping_cm(); // Right Sensor
  if(distance3 == 0) distance3 = 300; 
  
 if (distance < 25){
    int t = millis();
    backupRight();
    Straight();  
 }
 else if (distance2 < 40){
    digitalWrite(A0, HIGH);
    int t = millis();
    backupLeft();
    digitalWrite(A0, LOW);
    Straight();
 }
 else if (distance3 < 40){
    digitalWrite(A3, HIGH);
    int t = millis();
    backupRight();
    digitalWrite(A3, LOW);
    Straight();
 }
 if(distance2 < 100){
    digitalWrite(A0, HIGH);
    moveForward(50);           
    rightTurn();
    delay(300);
    digitalWrite(A0, LOW);
    Straight();
 }                                          
 else if (distance3 < 100){ 
    digitalWrite(A3, HIGH);
    moveForward(50);
    leftTurn();
    delay(300);
    digitalWrite(A3, LOW);
    Straight();
 }                                               // Median Distance
 else if (distance2 < 150){
    digitalWrite(A0, HIGH);
    moveForward(60);
    rightTurn();
    delay(150);
    digitalWrite(A0, LOW);
    Straight();
 }                                                                                
 else if (distance3 < 150){
    digitalWrite(A3, HIGH);
    moveForward(60);
    leftTurn();
    delay(150);
    digitalWrite(A3, LOW);
    Straight();
 }
 else if (A < 15 && A > -15 && millis() - t > 7500){
   moveForward(60);
   Straight();
   delay(50);
 }
 else if (A > 180 && millis() - t > 5000){
    moveForward(60);
    rightTurn();
    delay(50);
  }
 else if (-180 < A && A < 0 && millis() - t > 7500){
    moveForward(60);
    rightTurn();
    delay(50);
  }
 else if (A < 180 && millis() - t > 7500){
    moveForward(60);
    leftTurn();
    delay(50);
  }
 else if (-180 > A && A < 0 && millis() - t > 7500){
    moveForward(60);
    leftTurn();
    delay(50);
    }
  }
  else{
    stopCar();
    Straight();
    Serial.println("Arrived");
    delay(1000);
}
}
else{
   stopCar();
   Straight();
   Serial.println(GPS.fix);
   Serial.println("Waiting");
   delay(1000);
  }
}



void backupRight(){//Backup turn right
    //moveBackward(100);
    //delay(100);
    moveBackward(55);
    rightTurn();
    delay(1250);
}

void backupLeft(){//Backup turn left
    //moveBackward(100);                        
    //delay(100);
    moveBackward(55);
    leftTurn();
    delay(1250);
}

void stopCar(){//Stop car
  digitalWrite(controlPin1, HIGH);
  digitalWrite(controlPin2, HIGH);
}

void moveForward(float x){
  y = 200 - (200*(x/100));
  digitalWrite(controlPin1, HIGH);//Move foreward
  analogWrite(controlPin2, y);
}
void moveBackward(float b){
  a = 200 - (200*(b/100));
  digitalWrite(controlPin2, HIGH);//Move backward
  analogWrite(controlPin1, a);
}

void rightTurn(){
  digitalWrite(enablePin2, HIGH);
  digitalWrite(controlPin3, 0);//Right turn
  digitalWrite(controlPin4, 100);
}

void leftTurn(){
  digitalWrite(enablePin2, HIGH);
  digitalWrite(controlPin3, 100);//Left turn
  digitalWrite(controlPin4, 0);
}

void Straight(){
  digitalWrite(enablePin2, LOW);
  digitalWrite(controlPin3, 0);//Straighten wheels
  digitalWrite(controlPin4, 0);
}

void readGPS(){
 if (! usingInterrupt) {
    char c = GPS.read();
  }
 if (GPS.newNMEAreceived()) {
   if (!GPS.parse(GPS.lastNMEA()))   
      return;
  }
}

double distanceBetween(double lat1, double long1, double lat2, double long2){
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795 * 3.28;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}


