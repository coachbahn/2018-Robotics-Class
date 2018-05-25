#include <PololuMaestro.h>
/*
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif
*/
#define maestroSerial Serial3

MiniMaestro maestro(maestroSerial);

const int shoulder = 0;  //constant for maestro controller
const int knee = 1;  //constant for maestro controller
const int shoulder2 = 2;
const int knee2 = 3;
const int shoulder3 = 4;
const int knee3 = 5;
const int shoulder4 = 6;
const int knee4 = 7;

float s = 2.5;          //length of path of foot travel while in contact with the ground (in inches)
float h = -5.75;         //position of foot during ground contact relative to the upper servo (in inches)

const double L1 = 3.00;  //upper leg length (inches), from center of shoulder horn to center of knee horn
const double L2 = 3.00;  //lower leg length (inches), from center of knee horn to foot

double shoulderAngle = 0;  //angle shoulder servo should be at, measured from perpendicular to servo
double kneeAngle = 0;  //angle knee should be at, measured from upper leg
double t = 0;  //time value, iterates through Tau
double x = 0;  //x coordinate of foot
double y = 0;  //y coordinate of foot

double shoulderMicroseconds = 0;  //placeholder for getShoulderMicroseconds
double kneeMicroseconds = 0;  //placeholder for getKneeMicroseconds
double shoulder2Microseconds = 0;
double knee2Microseconds = 0;
double shoulder3Microseconds = 0;
double knee3Microseconds = 0;
double shoulder4Microseconds = 0;
double knee4Microseconds = 0;

bool limit;

void setup() {
  maestroSerial.begin(115200);
  Serial.begin(19200);
  laydown();
  delay(2000);
  standard();
  //zero();
  delay(2000);
}

void loop() {
  noArrays();
  //delay(10);
}

double getX(double t) {           //This function will take t, and assign an X value based on linear parametric equations
  if(t >= 6.28) t = t - 6.28;
  if(t < (3 * 3.14) / 2) x = (2 * s * t) / (3 * 3.14) - (s / 2);  //change 8 and 2 for different size gait for x
  else x = (5 * s / 8) * cos(1.185 * t - 4.94);  //change r for recovery cycle
  return x;
  //x = 1;
}

double getY(double t) {           //This function will take t, and assign an X value based on linear parametric equations
  if(t >= 6.28) t = t - 6.28;
  if(t < (3 * 3.14) / 2) y = h;   //change h to shift gait up
  else y = (5 * s / 8) * sin(1.185 * t - 4.94) - (3 * s / 8) + h;  //0.5 was added to make the return stroke a bit higher...
  return y;
  //y = -5.85;
}

double getShoulderAngle(double x, double y) {
  //returns θ1 (angle of shoulder) given x,y-coordinate values
  shoulderAngle = (180/3.141592)*(-1*acos(((((pow(L1, 3))*x)+((L1*x)*(-1*(sq(L2))+(sq(x))+(sq(y)))))+sqrt(((-1*sq(L1))*(sq(y)))*((pow(L1, 4)+sq(((-1*sq(L2))+(sq(x))+(sq(y)))))-((2*(sq(L1)))*((sq(L2))+(sq(x))+(sq(y)))))))/((2*(sq(L1)))*((sq(x))+(sq(y))))));
  return shoulderAngle;
}

double getKneeAngle(double x, double y) {
  //returns θ2 (angle of knee) given x,y-coordinate values
  kneeAngle = (180/3.141592)*(-1*acos((((-1*(sq(L1)))-(sq(L2)))+((sq(x))+(sq(y))))/(2*L1*L2)));
  return kneeAngle;
}

double getShoulderMicroseconds(double angle) {
  //converts precise angle to quatermicrosecond value needed for servo controller (specific to servo and joint)
  //shoulderMicroseconds = (43.1*angle) + 9871;
  shoulderMicroseconds = (43.1*angle) + 10071;
  return shoulderMicroseconds;
}

double getKneeMicroseconds(double angle) {
  //converts precise angle to quatermicrosecond value needed for servo controller (specific to servo and joint)
  //kneeMicroseconds = (41.6*angle) + 9441;
  kneeMicroseconds = (41.6*angle) + 8961;
  return kneeMicroseconds;
}

double getShoulder2Microseconds(double angle) {
  //shoulder2Microseconds = (44.1*angle) + 10031;
  shoulder2Microseconds = (44.1*angle) + 10031;
  return shoulder2Microseconds;
}

double getKnee2Microseconds(double angle) {
  //knee2Microseconds = (41.8*angle) + 9526;
  knee2Microseconds = (41.8*angle) + 9526;  //(9226)
  return knee2Microseconds;
}

double getShoulder3Microseconds(double angle) {
  //shoulder3Microseconds = (44.9*(-angle-180)) + 10006;
  shoulder3Microseconds = (44.9*(-angle-180)) + 10206;
  return shoulder3Microseconds;
}

double getKnee3Microseconds(double angle) {
  knee3Microseconds = (39.1*(-angle-180)) + 8961;
  return knee3Microseconds;
}

double getShoulder4Microseconds(double angle) {
  //shoulder4Microseconds = (44.1*(-angle-180)) + 9960;
  shoulder4Microseconds = (44.1*(-angle-180)) + 10160;
  return shoulder4Microseconds;
}

double getKnee4Microseconds(double angle) {
  //knee4Microseconds = (40.9*(-angle-180)) + 9256;
  knee4Microseconds = (40.9*(-angle-180)) + 9056;
  return knee4Microseconds;
}

void zero() {
  //horn should be perpendicular to servo, leg back
  maestro.setTarget(shoulder, getShoulderMicroseconds(0));
  //horn should be in line with servo, leg straight
  maestro.setTarget(knee, getKneeMicroseconds(0));
  
  maestro.setTarget(shoulder2, getShoulder2Microseconds(0));
  maestro.setTarget(knee2, getKnee2Microseconds(0));
  maestro.setTarget(shoulder3, getShoulder3Microseconds(0));
  maestro.setTarget(knee3, getKnee3Microseconds(0));
  maestro.setTarget(shoulder4, getShoulder4Microseconds(0));
  maestro.setTarget(knee4, getKnee4Microseconds(0));
}

void standard() {
  //standard positions for both joints
  for (double q = 0; q<100; q=q+1) {
    double standardShoulderAngle = ((-45.00/100.00)*q);
    double standardKneeAngle = (((90.00/100.00)*q)-180.00);
    maestro.setTarget(shoulder, getShoulderMicroseconds(standardShoulderAngle));
    maestro.setTarget(knee, getKneeMicroseconds(standardKneeAngle));
    maestro.setTarget(shoulder2, getShoulder2Microseconds(standardShoulderAngle));
    maestro.setTarget(knee2, getKnee2Microseconds(standardKneeAngle));
    maestro.setTarget(shoulder3, getShoulder3Microseconds(standardShoulderAngle));
    maestro.setTarget(knee3, getKnee3Microseconds(standardKneeAngle));
    maestro.setTarget(shoulder4, getShoulder4Microseconds(standardShoulderAngle));
    maestro.setTarget(knee4, getKnee4Microseconds(standardKneeAngle));
    delay(10);
  }
}

void laydown() {
  maestro.setTarget(shoulder, getShoulderMicroseconds(0));
  maestro.setTarget(knee, getKneeMicroseconds(-180));
  maestro.setTarget(shoulder2, getShoulder2Microseconds(0));
  maestro.setTarget(knee2, getKnee2Microseconds(-180));
  maestro.setTarget(shoulder3, getShoulder3Microseconds(0));
  maestro.setTarget(knee3, getKnee3Microseconds(-180));
  maestro.setTarget(shoulder4, getShoulder4Microseconds(0));
  maestro.setTarget(knee4, getKnee4Microseconds(-180));
}

void noArrays() {
  for (double t; t < 6.2831; t=t+.035) {   //0.2 is close to correct.  reporting back might be better. -Bahn
    
    getX(t);
    getY(t);
    getShoulderAngle(x, y);
    maestro.setTarget(shoulder, getShoulderMicroseconds(shoulderAngle));
    getKneeAngle(x,y);
    maestro.setTarget(knee, getKneeMicroseconds(kneeAngle));
    
    getX(t+(3.1415/2.0000));
    getY(t+(3.1415/2.0000));
    getShoulderAngle(x, y);
    maestro.setTarget(shoulder3, getShoulder3Microseconds(shoulderAngle));
    getKneeAngle(x, y);
    maestro.setTarget(knee3, getKnee3Microseconds(kneeAngle));

    getX(t+3.1415);
    getY(t+3.1415);
    getShoulderAngle(x, y);
    maestro.setTarget(shoulder2, getShoulder2Microseconds(shoulderAngle));
    getKneeAngle(x, y);
    maestro.setTarget(knee2, getKnee2Microseconds(kneeAngle));
    
    getX(t+(3*3.1415/2.0000));
    getY(t+(3*3.1415/2.0000));
    getShoulderAngle(x, y);
    maestro.setTarget(shoulder4, getShoulder4Microseconds(shoulderAngle));
    getKneeAngle(x, y);
    maestro.setTarget(knee4, getKnee4Microseconds(kneeAngle));
    
    delay(1);
  }
}
