//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Code updated by Cara Nunez 4.17.19
//--------------------------------------------------------------------------
// Parameters that define what environment to render
//#define ENABLE_VIRTUAL_WALL
#define ENABLE_VIRTUAL_WALL_SYSTEM
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_NONLINEAR_FRICTION
//#define ENABLE_HARD_SURFACE
//#define ENABLE_BUMP_VALLEY
//#define ENABLE_TEXTURE

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Define kinematic parameters you may need
double rh = 0.0841;       // radius of handle [m]
double rp = 0.0091;       // radius of pully  [m]
double rs = 0.0762;       // radius of sector [m] 


// Virtual spring with Processing rendering 
double x_m = 0.01;
double acc_m = 0;
double vel_m = 0;
double prev_acc_m = 0;
double prev_vel_m = 0;

double F_m = 0;
double F_user = 0;

double F_spring = 0;
double F_damping = 0;

double x_wall = 0.05;

double m = 2;
double b = 1;
double k = 300;
double kuser = 1000;
double x_m_eq =  0.01 ;


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
    flipped = false;
  }
 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = updatedPos * 0.012767778 - 2.323735639;
     
  // Compute the position of the handle (in meters) based on ts (in radians)
  xh = ts * (PI / 180) * rh;

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
  
  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;
  
  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;


  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
   
#ifdef ENABLE_VIRTUAL_WALL

    // define wall position and spring constant
    double x_wall = 0.005;
    double k = 400.0;
    Serial.println(xh, 5);
    

    // if the handle position is past the wall position, then apply restorative spring force 
    if(xh > x_wall){
      force = -k * (xh - x_wall);
    } else {
      force = 0;
    }
    
    // max spring constant that feels good is k = 400.0 N/m
    
#endif


#ifdef ENABLE_VIRTUAL_WALL_SYSTEM  

    // print xh and xm to serial
    Serial.println(xh, 3);
    Serial.print('\t');
    Serial.println(x_m, 3);


    // store previous mass positions and velocities
    prev_acc_m = acc_m;
    prev_vel_m = vel_m;


    // if the handle position is past the mass position, then apply restorative spring force 
    if(xh > x_m){
      F_user = kuser * (xh - x_m);
    } else {
      F_user = 0;
    }

    // calculate spring force of mass attached to spring
    F_spring = - k * (x_m - x_m_eq);
    // calculate damping force
    F_damping = - b * vel_m;
    // calculate total force of mass
    F_m = F_user + F_spring + F_damping;
    // calculate mass acceleration
    acc_m = F_m / m;
    // integrate to calculate mass velocity and position
    vel_m += 0.5 * (prev_acc_m + acc_m) * 0.001;
    x_m += 0.5 * (prev_vel_m + vel_m) * 0.001;

    // apply force to the handle equal to negative of force applied to mass by user
    force = -F_user;
    
#endif


#ifdef ENABLE_LINEAR_DAMPING

    // define damping coefficient and apply negative damping force proportional to velocity
    double b = 10.0;
    force = -b * dxh_filt;
    
    // max damping coefficient that feels good is b = 10.0 Ns/m
    
#endif


#ifdef ENABLE_NONLINEAR_FRICTION
#endif


#ifdef ENABLE_HARD_SURFACE
#endif


#ifdef ENABLE_BUMP_VALLEY  

    // define position of bumps and valleys and corresponding widths and edges
    double x_bump = -0.03;
    double x_valley = 0.03;
    double width = 0.005;
    double x_bump_right = x_bump + width;
    double x_bump_left = x_bump - width;
    double x_valley_right = x_valley + width;
    double x_valley_left = x_valley - width;
    double scale_bump = 1.0;
    double scale_valley = 1.0;


    // if handle is in the valley, then apply force proportional to negative gradient of a valley
    if((xh > x_valley_left) && (xh < x_valley_right)) {
      double x = (xh - x_valley) / width;
      force = scale_valley * (pow(x, 3) - x);
    // if handle is in the bump, then apply force proportional to negative gradient of the bump
    } else if((xh > x_bump_left) && (xh < x_bump_right)) {
      double x = (xh - x_bump) / width;
      force = -scale_bump * (pow(x, 3) - x);
    } else {
      force = 0;
    }

#endif


#ifdef ENABLE_TEXTURE
#endif


// calculate torque needed to create desired force
Tp = ((rh * rp) / rs) * force;

  
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
