
import processing.sound.*;
import processing.serial.*;

Serial myPortKnife;
Serial myPortCarrot;

PImage full_body;
PImage chopping_board;
PImage knife;
float x_L = (float) 0;
float x_L_prev;
float x_H;
float thresh = 150;
float chop_x = 0;
ArrayList<PImage> carrot_img_arr = new ArrayList<PImage>();
ArrayList<Integer> chops_x_coords = new ArrayList<Integer>();
SoundFile chop_sound;
boolean stop = false;

float y_knife;
float y_knife_prev;
float x_carrot;
float x_carrot_prev;

float xh_knife_in;
float xh_carrot_in;

float y_carrot;

boolean can_cut = false;

float[] rots ={};
float[] trans_xs ={};
float[] trans_ys ={};


void setup() {
  size(1280, 720);
  full_body = loadImage("full_body.png");
  chopping_board= loadImage("chopping_board1.png");
  knife = loadImage("knife1.png");
  knife.resize(width/5, (knife.height)/(knife.width) * (width/5));
  //cursor(knife,0,0);
  
  chopping_board.resize(width, height);
  background(chopping_board);
  
  chop_sound = new SoundFile(this, "cut_carrot.mp3");
  x_L = width - full_body.width;
  x_H = width - 100;
  imageMode(CORNER);
  float rat = full_body.width / full_body.height;
  full_body.resize((int) rat*height/5, (int) height/5);
  image(full_body, width - full_body.width, height/2 - full_body.height/2, full_body.width, full_body.height);
  
  y_carrot = int(map(-0.03, -0.07, 0.01, 0, height / 2));
  carrot_img_arr.add(full_body);
  
  // List all the available serial ports
  //println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
  myPortKnife = new Serial(this, Serial.list()[1], 115200);  //make sure baud rate matches Arduino
  myPortCarrot = new Serial(this, Serial.list()[2], 115200);  //make sure baud rate matches Arduino
  myPortKnife.buffer('\n');
  myPortCarrot.buffer('\n');
}
  
void draw() {
  
  /*
  xCARROT = map(x_carrot, (float) -0.04650, (float) 0.05284, (float) 0, width);
  xKNIFE = min( map(y_knife, (float) -0.04650, (float) 0.05284, (float) 0, height), height/2 + full_body.height/2 );
  
  image(knife, xCARROT, xKNIFE);
  
  if ( (xCARROT  > x_L) && (xCARROT < x_H) && ((x_H-x_L) > thresh) && (xKNIFE <= (height/2 + full_body.height/2)))
  {
    if (x_L != (float) 0) {
      x_L_prev = x_L;
    }
    chop_x = xCARROT;
    chop_sound.play();
    PImage c = get((int) x_L, (int) (height/2 - full_body.height/2), (int) (chop_x-x_L), (int) full_body.height);
    chops.add(c);
    chops_x_coords.add((int) x_L);
    x_L = chop_x;
  }
  
  */
  
  y_knife = int(map(xh_knife_in, -0.07, 0.01, 0, height / 2));
  x_carrot = int(map(xh_carrot_in, 0.0, -0.14, -width * 0.3, width * 0.8));
  background(chopping_board);
  //image(carrot_img_arr.get(0), x_carrot, height/2 - full_body.height/2, full_body.width, full_body.height);
  for (int i = 1; i < carrot_img_arr.size(); i = i+1){
    pushMatrix();
    translate(10 , 0);
    rotate(rots[i -1]); // in rad
    image(carrot_img_arr.get(i), width / 2 + trans_xs[i-1], trans_ys[i-1]);
    popMatrix();
  }
  image(carrot_img_arr.get(0), x_carrot, height/2 - full_body.height/2 - 50);
  //image(full_body, 0, height/2 - full_body.height/2, full_body.width, full_body.height);
  image(knife, width * 0.5, y_knife);
  
  if (abs(x_carrot - x_carrot_prev) > 3 && y_knife < y_carrot && x_carrot + full_body.width > width * 0.5) {
    can_cut = true;
  }
  
  if (y_knife > y_carrot + 50 && can_cut) {
    can_cut = false;
    chop_sound.play();
    chops();
  }
  
  x_carrot_prev = x_carrot;
}


void chops() {
  //float slice_x = x_carrot + carrot_img_arr.get(0).width - (width / 2);
  //copy(carrot_img_arr.get(0), (int) slice_x, (int) 0, (int) carrot_img_arr.get(0).width, (int) carrot_img_arr.get(0).height, (int) slice_x, (int) 0, (int) carrot_img_arr.get(0).width, (int) carrot_img_arr.get(0).height);
  try {
  PImage temp = carrot_img_arr.get(0);
  carrot_img_arr.set(0, temp.get((int) 0, (int) 0, (int) (width / 2 - x_carrot), (int) 500));
  carrot_img_arr.add(temp.get((int) (width / 2 - x_carrot), (int) 0, (int) 500, (int) 500));
  
  float rot = random(0, PI/10);
  float trans_x = random(150, 400);
  float trans_y = random(25, 125);
  
  rots = append(rots, rot);
  trans_xs = append(trans_xs, trans_x);
  trans_ys = append(trans_ys, trans_y);
  
  //PImage c = temp.get((int) slice_x, (int) 0, (int) (temp.width - (width / 2 - x_carrot)), (int) temp.height);
  //carrot_img_arr.add(c);
  } catch (NegativeArraySizeException e) {
    
  }
}

void serialEvent (Serial thisPort) {
  // get the ASCII string:
  // read the input string
  // HINT: use myPort.readStringUntil() with the appropriate argument
  // trim and convert string to a number
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width & update previous value variable


// read Serial data


  // read the incoming serial data:
  String inString = thisPort.readStringUntil('\n');

  // if the string is not empty, do stuff with it:
  if (inString != null) {
    // if the string came from serial port one:
    if (thisPort == myPortKnife) {
      //print ("Data from port one: ");
      xh_knife_in = float(inString);
    }
    // if the string came from serial port two:
    if (thisPort == myPortCarrot) {
      //print ("Data from port two: ");
      xh_carrot_in = float(inString);
    }
    // print the string:
    //println(inString);
  }
}
