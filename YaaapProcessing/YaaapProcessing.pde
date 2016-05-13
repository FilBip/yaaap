import processing.serial.*;
Serial myPort;

int newLine = 13; // new line character in ASCII
float yaw;
float pitch;
float roll;
float yawd;
float pitchd;
float rolld;
String message;
String [] ypr = new String [3];
void setup()
{
  size(600, 500, P3D);
  
  /*Set my serial port to same as Arduino, baud rate 9600*/
  myPort = new Serial(this, Serial.list()[2], 115200); // if you have only ONE COM port active
  //myPort = new Serial(this, "COM5", 9600);  // if you know the COM port

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white

  translate(width/2, height/2); // set position to centre
  
  pushMatrix(); // begin object
  
  rotateY(-yaw); // yaw
  rotateX(-pitch); // RotateX pitch value
  rotateZ(roll); // roll
  
  drawArduino(); // function to draw rough Arduino shape
  popMatrix(); // end of object
  // textFont(font, 20);

if (yaw < 0) yawd=360+yaw*180/PI; 
else yawd=yaw*180/PI;

if (roll <0)  rolld=roll*180/PI+180;
else rolld=-180+roll*180/PI;

  textSize(16);
  textAlign(LEFT, TOP);
  fill(0); // Black text
  text("Heading",-70,100);
  text(nfp(yawd,3,1),0,100);
  text("Heeling",-70,130);
  text(nfp(rolld,0,1),0,130);

// Draw a compass to view north ahead
 fill(200);
 ellipse(200,-100,60,10);
  textAlign(CENTER, TOP);
 fill(100);
 textSize(18);
 text("N",200,-130);
 textSize(20);
 text("W",160,-120); 
 text("E",240,-120); 
 textSize(22);
 text("S",200,-110);
 
  // Print values to console
  print(round(yaw*180/PI));   
  print("\t");
  print(round(pitch*180/PI));
  print("\t");
  print(round(roll*180/PI)); 
  println("\t");

} 

void serialEvent()
{
 do {  message = myPort.readStringUntil(newLine); // read from port until new line (ASCII code 13)
 }  while (message == null);
    ypr = split(message, ","); // split message by commas and store in String array 
    yaw = float(ypr[0]); // convert to float yaw
    pitch = float(ypr[1]); // convert to float pitch
    roll = float(ypr[2]); // convert to float roll
}
void drawArduino() {
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 130, 130); // set outline colour to darker teal
  fill(0, 200, 120); // set fill colour to lighter teal
  box(100, 30, 200); // draw hull shape

  stroke(80); // set outline colour
  fill(170); // set fill colour

  translate(00, 20, 20); 
  box(60, 20, 80); // draw roof

  translate(0, 90, -60); // set position to other edge of Arduino box
  box(5, 200, 5); // draw a mast
}