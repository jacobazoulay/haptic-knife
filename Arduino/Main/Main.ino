//------------------------------------------------------------------------------------------
// Parameters that define what environment to render
//#define ENABLE_VIRTUAL_WALL
//#define ENABLE_VIRTUAL_WALL_SYSTEM
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_BUMP_VALLEY
//#define BILATERAL
//#define CARROT_RESTORE
//#define CUTTING_BOARD
#define CUTTING_CARROT

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor


// Position tracking variables
//------------------------------------------------------------------------------------------
//Leader variables
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


// Kinematic variables
//------------------------------------------------------------------------------------------
// Leader Variables
float xh = 0;           // position of the handle [m]
float theta_s = 0;      // Angle of the sector pulley in deg
float xh_prev;          // Distance of the handle at previous time step
float dxh;              // Velocity of the handle
float dxh_prev;
float dxh_prev_prev;
float dxh_filt;         // Filtered velocity of the handle
float dxh_filt_prev;
float dxh_filt_prev_prev;

// Follower variables
float xh_remote = 0;
float xh_remote_prev = 0;
float dxh_remote;
float dxh_remote_prev;
float dxh_remote_prev_prev;
float dxh_remote_filt;
float dxh_remote_filt_prev;
float dxh_remote_filt_prev_prev;

// Define kinematic parameters you may need (Jacob)
double rh = 0.0841;       // radius of handle [m]
double rp = 0.0091;       // radius of pully  [m]
double rs = 0.0762;       // radius of sector [m]

bool Jacob = true;  //used for changing calibration settings depending on whose hapkit board is being used
double ts;

// Define virtual spring with Processing rendering variables
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

// Carrot Variables
double max_x;
double x_carrot_min = 2;
bool outside;


// Special variables for efficient transmission over serial
// We use a union so that the binary form of the integer shares the same memory space as the int form
typedef union {
   int integer;
   byte binary[2];
} binaryInt; 

binaryInt xh_bin; 
binaryInt xh_remote_bin;


// Force output variables
// ----------------------------------------------------------------------------------------------------
// Leader Variables
float force = 0;           // force at the handle
float Tp = 0;              // torque of the motor pulley
float duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

//Serial communication variables
bool sending = true; // flip the tx/rx flags
bool receiving = false;



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
  
    // initialize the values for the follower kinematics
    xh_bin.integer = 0;
    xh_remote_bin.integer = 0;
}



// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
    //*************************************************************
    //*** Section 1. Compute position in counts (do not change) ***  
    //*************************************************************
  
    // Leader
    //-------------------------------------------------------------------------------------------------------
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
    // Use user specific calibration measurements
     if (Jacob){
        ts = updatedPos * 0.012767778 - 2.323735639;
     } else {
        ts = 0.0169*updatedPos - 10.159;
     }
     
    // Compute the position of the handle (in meters) based on ts (in radians)
    xh = ts * (PI / 180) * rh;
    
    // Calculate velocity with loop time estimation
    dxh = (float)(xh - xh_prev) / 0.001;
    
    // Calculate the filtered velocity of the handle using an infinite impulse response filter
    dxh_filt = 0.9*dxh + 0.1*dxh_prev; 
      
    // Record the position and velocity
    xh_prev = xh;
    
    dxh_prev_prev = dxh_prev;
    dxh_prev = dxh;
    
    dxh_filt_prev_prev = dxh_filt_prev;
    dxh_filt_prev = dxh_filt;
  
  
    //*************************************************************
    //** Teleop Communication. Send and receive handle positions.**
    //*************************************************************
    //--------------------------------------------------------------------------------------------------------------------
//    if (sending && Serial.availableForWrite()> sizeof(int)){ //check that we have space in the serial buffer to write
//        xh_bin.integer = int(xh*100000.0); // save space by using a integer representation
//        Serial.write(xh_bin.binary,2); // write the integer to serial
//        sending = false; // flip the tx/rx flags
//        receiving = true;
//        Serial.flush(); // flush the serial for good measure
//    }
//    // read our follower position
//    if (receiving && Serial.available()> 1){ //if there is at least 2 bytes of data to read from the serial
//        xh_remote_prev = xh_remote; // backup old follower position
//        Serial.readBytes(xh_remote_bin.binary,2); // read the bytes in
//        xh_remote = (float)xh_remote_bin.integer/100000.0; // convert the integer back into a float
//        if (isnan(xh_remote)) //if xh_2 is corrupt, just use the old value
//          xh_remote = xh_remote_prev;
//        sending = true; // flip the tx/rx flags
//        receiving = false;
//        Serial.flush(); // flush the serial for good measure
//    }
    
    //*************************************************************
    //*** Section 3. Assign a motor output force in Newtons *******  
    //*************************************************************
    //*************************************************************
    //******************* Rendering Algorithms ********************
    //*************************************************************
  
    // Calculate remote velocity with loop time estimation
    dxh_remote = (float)(xh_remote - xh_remote_prev) / 0.001;
    // Calculate the filtered velocity of the handle using an infinite impulse response filter
    dxh_remote_filt = 0.9*dxh_remote + 0.1*dxh_remote_prev; 
      
    // Record the position and velocity
    xh_remote_prev = xh_remote;
    
    dxh_remote_prev_prev = dxh_remote_prev;
    dxh_remote_prev = dxh_remote;
    
    dxh_remote_filt_prev_prev = dxh_remote_filt_prev;
    dxh_remote_filt_prev = dxh_remote_filt;
  
  
    
    #ifdef BILATERAL
         double pos_scale = 1.0;
         double kp = 20;
         double kd = 1;
      
         // implement basic PD controller using (scaled) remote xh as reference point
         force = kp*((xh_remote*pos_scale) - xh) + kd*(dxh_remote - dxh);
    #endif
  
  
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


     #ifdef CARROT_RESTORE

         force = 0.3;
         Serial.println(xh, 5);

     #endif


     
     #ifdef CUTTING_BOARD

        // define wall position and spring constant
        double x_wall = 0.000;
        double k = 400.0;
//        Serial.println(xh, 5);
        
        // if the handle position is past the wall position, then apply restorative spring force 
        if(xh > x_wall){
            force = -k * (xh - x_wall);
        } 

     #endif


     #ifdef CUTTING_CARROT

        // define wall position and spring constant
        double x_carrot = -0.03;
        double k_carrot = 1000.0;
        double thick = 0.01;
        Serial.println(xh, 5);
        

          // cut rendering for single stationary x position
          // checks if cut has been made and if carrot has been pushed passed current max depth
          if(xh < x_carrot){
            if(xh_remote < x_carrot_min) {
                outside = true;
            }
              max_x = x_carrot;
          }  else if(xh > x_carrot+thick){
              outside= false;
         
          }
  
//          if(xh > max_x && xh < 0.0){
//              max_x = xh;
//              double b_carrot = 1.0;
//              force = -1.0;
//          }
          
          // if the handle position is past the wall position, then apply restorative spring force 
          if(xh > x_carrot && xh < x_carrot + thick && outside){
              force = -k_carrot * (xh - x_carrot);
             
          } else {
            force = 0;
        }

          if(xh_remote < x_carrot_min) {
//            x_carrot_min = xh_remote;
          }

     #endif

    //*************************************************************
    //*** Section 3. Assign a motor output force in Newtons *******  
    //*************************************************************
    
    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
    Tp = force*rh*rp/rs;

  
    //*************************************************************
    //*** Section 4. Force output (do not change) *****************
    //*************************************************************
  
    // Leader
    //------------------------------------------------------------------------------
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
