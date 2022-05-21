
import processing.sound.*;
import processing.serial.*;

Serial myPort;

PImage full_body;
PImage chopping_board;
PImage knife;
float x_L = (float) 0;
float x_L_prev;
float x_H;
float thresh = 150;
float chop_x = 0;
ArrayList<PImage> chops = new ArrayList<PImage>();
ArrayList<Integer> chops_x_coords = new ArrayList<Integer>();
SoundFile chop_sound;
boolean stop = false;

float x_knife;
float x_knife_prev;
float x_carrot;
float x_carrot_prev;

void setup() {
  size(1280, 720);
  full_body = loadImage("full_body.png");
  chopping_board= loadImage("chopping_board.png");
  knife = loadImage("knife.png");
  knife.resize(width/5, (knife.height)/(knife.width) * (width/5));
  cursor(knife,0,0);
  
  chopping_board.resize(width, height);
  background(chopping_board);
  
  chop_sound = new SoundFile(this, "cut_carrot.mp3");
  x_L = width - full_body.width;
  x_H = width - 100;
  imageMode(CORNER);
  float rat = full_body.width / full_body.height;
  full_body.resize((int) rat*height/3, (int) height/3);
  image(full_body, width - full_body.width, height/2 - full_body.height/2, full_body.width, full_body.height);
  
  // List all the available serial ports
  //println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
  myPort = new Serial(this, Serial.list()[1], 115200);  // also make sure baud rate matches Arduino
  
  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
}
  
void draw() {
  
  /*
  xCARROT = map(x_carrot, (float) -0.04650, (float) 0.05284, (float) 0, width);
  xKNIFE = min( map(x_knife, (float) -0.04650, (float) 0.05284, (float) 0, height), height/2 + full_body.height/2 );
  
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
  
  if (chops.size() > 0) {
    full_body = full_body.get( (int) (x_L - (width - full_body.width)), (int) (0), (int) (width - x_L) , (int) (full_body.height) );
    background(chopping_board);
    image(full_body, width - full_body.width, height/2 - full_body.height/2, full_body.width, full_body.height);
    
    for (int i = 0; i < chops.size(); i++) {
      PImage c_new = chops.get(i);
      pushMatrix();
      translate(chops_x_coords.get(i) + c_new.width/2, (height/2 - full_body.height/2) + c_new.height/2);
      rotate(-PI/10); // in rad
      image(c_new, -c_new.width, -c_new.height/2, c_new.width, c_new.height);
      popMatrix();
    }
  }
}

void serialEvent (Serial myPort) {
  
  if (myPort.available() > 0) {
    String xk_and_xc = myPort.readStringUntil((int) 10);
    String[] xArray = xk_and_xc.split(",");
    x_knife = parseFloat(xArray[0]);
    x_carrot = parseFloat(xArray[1]);
    
    if (Float.isNaN(x_knife))
    {
      x_knife = x_knife_prev;
    }
    
    else
    {
      draw();
      x_knife_prev = x_knife;
    }
    
    if (Float.isNaN(x_carrot))
    {
      x_carrot = x_carrot_prev;
    }
    
    else
    {
      draw();
      x_carrot_prev = x_carrot;
    }
  }
}
