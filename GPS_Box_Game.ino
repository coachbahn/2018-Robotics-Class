#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <ServoTimer2.h>

SoftwareSerial mySerial(6, 7);

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); 
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int switchState = 0;
int switchState2 = 0;
double goalLat;
double goalLon;
ServoTimer2 lock;
void setup()  {
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  lcd.begin(16, 2);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
 
  useInterrupt(true);

  delay(1000);
    pinMode(8,INPUT);
    pinMode(A1,INPUT);
    lock.attach(9);
    pinMode(A2,OUTPUT);
    lock.write(1200);
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
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop(){
  switchState2 = digitalRead(A1);
  switchState = digitalRead(8);
  while(GPS.fix == false){
    digitalWrite(A2, HIGH);
    checkGps();
    lcd.setCursor(0,0);
    lcd.print("Wait for signal");
    delay(500);
  }
  
    digitalWrite(A2, LOW);
   // lcd.clear();
   if(goalLat == 0){  
   digitalWrite(A2, HIGH);
   switchState2 = digitalRead(A1);
   lcd.setCursor(0,0);
   lcd.write("Push Button to");
   lcd.setCursor(0,1);
   lcd.print("Set Location");
   delay(100);
     }
     lcd.clear();
    //add function to check goalLet and print pussh button to set
    
  if (switchState2 == HIGH){
    checkGps();
    setGPS();
    delay (2000);
    lcd.clear();
    digitalWrite(A2, LOW);
  }
  
  if (switchState == HIGH){
    lock.write(2200);
    checkGps();
    printDistance();
    delay(2000);
    lcd.clear();
    digitalWrite(A2, LOW);
  }
}


double distanceBetween(double lat1, double long1, double lat2, double long2){
  // returns distance in feet between two positions
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

void checkGps(){
  if (! usingInterrupt) {
    char c = GPS.read();
  }
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;  
    }
  }
    
void printDistance(){
  if (GPS.fix){
      
    if (timer > millis())  timer = millis();

  
      if (millis() - timer > 1000) { 
         timer = millis(); // reset the timer
         
         Serial.print(GPS.latitudeDegrees, 4);
         Serial.print(", "); 
         Serial.println(GPS.longitudeDegrees, 4);
         digitalWrite(A2, HIGH); 
         Serial.print("DistanceToGoal: "); 
         lcd.print((int)distanceBetween(GPS.latitudeDegrees,GPS.longitudeDegrees,goalLat,goalLon));
         lcd.print(" ft");
        
     
  }
  if (distanceBetween(GPS.latitudeDegrees,GPS.longitudeDegrees,goalLat,goalLon) < 20){
    openBox();
  }
  else {
     lcd.setCursor(0,1);
         lcd.print("Try again in ");
         countdown2();
  }
  }
else {
  digitalWrite(A2, HIGH); 
  lcd.print("Wait for signal");

}
}


void setGPS(){
 
 if (GPS.fix){
   digitalWrite(A2, HIGH);
   lcd.print(" Setting Goal");
   goalLat =  GPS.latitudeDegrees;
   goalLon = GPS.longitudeDegrees;
   lock.write(1200);
   lcd.setCursor(0,1);
   lcd.print("Locking in");
   countdown();
    
   
   //add/modify to indlue countdown
 
   lock.write(2200);

 }
 else{
 digitalWrite(A2, HIGH);
 lcd.print("Wait for signal");
    }
  } 
  
 void openBox(){
   lcd.begin(16, 2);
   digitalWrite(A2, HIGH);
   lcd.print("Open Box");
   lcd.setCursor(0, 1);
   lock.write(1200);
   
 }
 
 void locked(){
   lcd.noDisplay();
 delay(300);
 }
 
 void countdown(){
   lcd.setCursor(11,1);
   lcd.print("10");
   delay(1000);
   lcd.setCursor(11,1);
   lcd.print("  ");
   lcd.setCursor(11,1);
   lcd.print("9");
   lcd.setCursor(11,1);
   delay(1000);
   lcd.print("8");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("7");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("6");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("5");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("4");
  lcd.setCursor(11,1); 
    delay(1000);
   lcd.print("3");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("2");
   lcd.setCursor(11,1);
    delay(1000);
   lcd.print("1");
   delay(1000);
 }
 
 void countdown2(){
   lcd.setCursor(13,1);
   lcd.print("10");
   delay(1000);
   lcd.setCursor(13,1);
   lcd.print("  ");
   lcd.setCursor(13,1);
   lcd.print("9");
   lcd.setCursor(13,1);
   delay(1000);
   lcd.print("8");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("7");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("6");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("5");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("4");
  lcd.setCursor(13,1); 
    delay(1000);
   lcd.print("3");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("2");
   lcd.setCursor(13,1);
    delay(1000);
   lcd.print("1");
   delay(1000);
 }
