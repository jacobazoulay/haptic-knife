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
String xh_string;
float  xh = 0.0;
float  xh_prev = 0.0;
float  x_wall_in = 0.005;
int    xh_screen;
int    x_wall;

void setup () {
  // set the window size:
  size(600, 400); 

  // List all the available serial ports
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[3], 115200);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}

void draw () {
  // everything happens in the serialEvent()
  background(0); 
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  // START EDITING HERE
  
  // map the raw x positions of handle and wall to pixel sizes
  xh_screen = int(map(xh, -0.055, 0.055, 0, width));
  x_wall = int(map(x_wall_in, -0.055, 0.055, 0, width));
  
  // Visual trick. If handle position is greater than wall, don't show it popping through
  if (xh_screen < x_wall - 4) {  
    circle(xh_screen, height / 2, 5);
  } else {
    circle(x_wall - 4, height / 2, 5);
  }
  
  
  // draw the wall
  line(x_wall, 0.0, x_wall, height);
  
  
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
  
  // store previous xh value and read Serial data
   xh_prev = xh;
   xh_string = myPort.readStringUntil('\n');
   xh = float(xh_string);
   
   // make sure xh from serial is not NaN or null
   if (Float.isNaN(xh)) {
     xh = xh_prev;
   }
  
  //STOP EDITING HERE
}
