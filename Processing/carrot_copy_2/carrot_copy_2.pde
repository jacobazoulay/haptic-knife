
import processing.sound.*;

PImage full_body;
PImage chopping_board;
float x_L = (float) 0;
float x_L_prev;
float x_H;
float thresh = 150;
float chop_x = 0;
ArrayList<PImage> chops = new ArrayList<PImage>();
ArrayList<Integer> chops_x_coords = new ArrayList<Integer>();
SoundFile chop_sound;

boolean stop = false;

void setup() {
  size(1280, 720);
  full_body = loadImage("full_body.png");
  chopping_board= loadImage("chopping_board.png");
  chopping_board.resize(width, height);
  background(chopping_board);
  
  chop_sound = new SoundFile(this, "cut_carrot.mp3");
  x_L = width - full_body.width;
  x_H = width;
  imageMode(CORNER);
  float rat = full_body.width / full_body.height;
  full_body.resize((int) rat*height/3, (int) height/3);
  image(full_body, width - full_body.width, height/2 - full_body.height/2, full_body.width, full_body.height);
}
  
void draw() {
  
  if (chops.size() > 0) {
    full_body = full_body.get( (int) (x_L - (width - full_body.width)), (int) (0), (int) (width - x_L) , (int) (full_body.height) );
    background(chopping_board);
    image(full_body, width - full_body.width, height/2 - full_body.height/2, full_body.width, full_body.height);
    
    for (int i = 0; i < chops.size(); i++) {
      PImage c_new = chops.get(i);
      pushMatrix();
      translate(chops_x_coords.get(i) + c_new.width/2, (height/2 - full_body.height/2) + c_new.height/2);
      rotate(-PI/10); // in rad
      //image(c_new, chops_x_coords.get(i), (height/2 - full_body.height/2), c_new.width, c_new.height);
      image(c_new, -c_new.width, -c_new.height/2, c_new.width, c_new.height);
      popMatrix();
    }
    
  }
}

void mouseClicked(){
  strokeWeight(2);
  strokeCap(PROJECT);
 
  if ((mouseX > x_L) && (mouseX < x_H) && ((x_H-x_L) > thresh)) {
    if (x_L != (float) 0) {
      x_L_prev = x_L;
    }
    chop_x = mouseX;
    chop_sound.play();
    PImage c = get((int) x_L, (int) (height/2 - full_body.height/2), (int) (chop_x-x_L), (int) full_body.height);
    chops.add(c);
    chops_x_coords.add((int) x_L);
    x_L = chop_x;
  }
}
