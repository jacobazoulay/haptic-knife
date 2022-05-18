// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 9600 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return

*/

import processing.serial.*;
import jssc.*;

// The serial port
Serial myPort;

//initialize variables
String x_in = " ";
String xh_string = " ";
float  xh = 0.0;
float  xh_screen;

float  xh_prev = 0.0;

String xm_string = " ";
float  xm = 0.0;
float  xm_screen;

float  xm_prev = 0.0;

float  x_wall_in = 0.05;
float  x_wall;


void setup () {
  // set the window size:
  size(600, 400); 

  // List all the available serial ports
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[3], 115200);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  //myPort.bufferUntil('\n');
    myPort.buffer('\n');
  background(0);      // set inital background:
}

void draw () {
  // everything happens in the serialEvent()
  background(0); 
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  // START EDITING HERE
  
// Visual trick. If handle is past position of mass, don't show it popping through
  if (xh_screen < xm_screen - 24) {  
    circle(xh_screen, height / 2, 20);
  } else {
    circle(xm_screen - 24, height / 2, 20);
  }
  
  // draw wall, spring, and mass
  line(x_wall, 0.0, x_wall, height);
  line(x_wall, height / 2, xm_screen, height / 2);
  circle(xm_screen, height / 2, 20);
  
    // Mass Spring Damper
  
  //draw the wall
  //draw a line from the wall to the xMass
  //draw an ellipse to represent the mass of the spring-mass-damper
  //draw an ellipse to represent the user
  
  
  // Virtual Wall
  // map the wall position from units of Arduino simulation to the screen width.
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels
  // draw the wall as a line
  // draw an ellipse to represent user position

}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  // read the input string
  // HINT: use myPort.readStringUntil() with the appropriate argument
  // trim and convert string to a number
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width & update previous value variable


// read Serial data
   x_in = myPort.readStringUntil('\n');
   
   // determine if xh or xm and store
   if (x_in != null) {
     if (x_in.indexOf('\t') != -1) {
       xm_string = x_in.substring(1);
     } else {
       xh_string = x_in;
     } 
   }
  
   
   xh = float(xh_string);
   if (Float.isNaN(xh)) {
     xh = xh_prev;
   }
   xh_prev = xh;
   
   
   xm = float(xm_string);
   if (Float.isNaN(xm)) {
     xm = xm_prev;
   }
   xm_prev = xm;
  
   // mapp x positions to pixel dimensions
   xh_screen = map(xh, -0.055, 0.055, 0, width);
   xm_screen = map(xm, -0.055, 0.055, 0, width);
   x_wall = map(x_wall_in, -0.055, 0.055, 0, width);
  
  //STOP EDITING HERE
}
