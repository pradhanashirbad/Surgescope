/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Ashirbad Pradhan, Ana Lucia
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 
ControlP5 cp5;


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 25.0;
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 38;  
float             worldHeight                         = 22; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
int               index = 0;
float             gravityAcceleration                 = 0;//980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

//define ball location
float             g_x=5.0;
float             g_y=8.0; 
float             mags;
float             sgn_x;
float             sgn_y;
float             f_x;
float             f_y;


/* define game ball */
FBox              g1;
FBox              g2;
FBox              wall;
FCircle           g4;
FCircle           g5;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

//timer
int m;
int counter =1;
int seconds;
int seconds_real;
int timeS;
int timeS_real;
int oldtimes;
float timeelapsed= 0.0;

//images
PImage hand;
PImage crosshairOFF1;
PImage crosshairON1;
PImage crosshairOFF2;
PImage crosshairON2;
PImage[] stage = new PImage[6];
PImage[] brain_stage = new PImage[6];
int counter_stage=0;

//depth axis
float depth=0.0;

//mouse 
boolean mouseHasBeenPressed= false;

//background
PShape left_back;
PShape right_back;
PShape[] target_mil;
PShape start_point;
PShape check_point;

//mousetracker
//P
boolean drawMode =false;
ArrayList<PVector> points;
float[] xArr;
float[] yArr;
int counter_points = 0;
//PVector val = new PVector(0, 0);//last values storing
float val_x;
float val_y;
float lastval_x=0.0;
float lastval_y=0.0;
boolean force_condition;

//startx starty
float start_x=0.0;
float start_y=0.0;

//checkpoint_x
float checkpoint_x=0.0;
float checkpoint_y=0.0;
boolean check_condition=false;

//target 
 float radius = 90;
 PShape targetA;
 PShape targetB;
 PShape targetB1;
 float factor = 0.0;

//FSR
float[] fsr;
float knobval;

//Slider
ControlP5 controlP5;
Slider c3;

//targetFSR
Textlabel FSRtxt;
Textlabel tasktxt;
Textlabel timertxt;
Textlabel overshoottxt;
PShape targetFSR;
int[] y_targetFSR;
int from = color(255, 0, 0);
int to = color(0, 120, 200);
float counter_overshoot=0;
float averageval =0.0;
float averageval_previous=0.0;

//milling condition
float[] mil_x;
float[] mil_y;
float[] milcond_x;
float[] milcond_y;
int[] miltimes;
int[] miltimes_temp;

//smoothing
int numReadings=30;
float[] readings;      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

//Parinaz's new codes starts here ***********************************************************************************************************
Button Tool1 ,Tool2, Tool3, Tool4; //names of buttons for each tool
PImage Tool1_vertical, Tool2_vertical, Tool3_vertical, Tool4_vertical; //images that appear on the right screen for each tool
int Tool_displacements[]=new int[6];
PImage button00, button01 ; //Images for the first button (Tool1)
PImage button10, button11 ; //Images for the first button (Tool1)
PImage button20, button21; //Images for the first button (Tool1)
PImage button30, button31;  //Images for the first button (Tool1) 


float pm=0;
float   pseconds ;
  float pseconds_real;
  float pdelta_t;
  float pdepth=0;
  
  double force;

//Parinaz's new codes finish here *************************************************************************************************************

//Start Devyani's code ***********************************************************************************************************
String img_file_path = "../img/";
int[] stage_damping; // damping force to apply at each stage 
int curr_damping; //current damping applied at the stage
FCircle c; //used it to test out coordinates
FPoly cut_target0;

//End Devyani's code*************************************************************************************************************


/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
/* put setup code here, run once: */

/* screen size definition */
size(1300, 550);

/* GUI setup */

/* set font type and size */
f                   = createFont("Arial", 16, true);

/* device setup */

/**  
 * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
 * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
 * to explicitly state the serial port will look like the following for different OS:
 *
 *      windows:      haplyBoard = new Board(this, "COM10", 0);
 *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
 *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
 */
haplyBoard          = new Board(this, Serial.list()[0], 0); //new Board(this, Serial.list()[0], 0);
widgetOne           = new Device(widgetOneID, haplyBoard);
pantograph          = new Pantograph();

widgetOne.set_mechanism(pantograph);

widgetOne.add_actuator(1, CCW, 2);
widgetOne.add_actuator(2, CW, 1);
 
widgetOne.add_encoder(1, CCW, 241, 10752, 2);
widgetOne.add_encoder(2, CW, -61, 10752, 1);

widgetOne.add_analog_sensor("A2");  
widgetOne.add_pwm_pin(4);  
widgetOne.device_set_parameters();


/* 2D physics scaling and world creation */
hAPI_Fisica.init(this); 
hAPI_Fisica.setScale(pixelsPerCentimeter ); 
world               = new FWorld();

 /* XY Plane */
brain_stage[0] = loadImage(img_file_path + "brain_stage0.png"); 
brain_stage[1] = loadImage(img_file_path + "brain_stage1.png"); 
brain_stage[2] = loadImage(img_file_path + "brain_stage2.png");
brain_stage[3] = loadImage(img_file_path + "brain_stage3.png");
brain_stage[4] = loadImage(img_file_path + "brain_stage4.png");
brain_stage[5] = loadImage(img_file_path + "brain_stage5.png");

for (int i=0; i < brain_stage.length; i++){
brain_stage[i].resize((int)(hAPI_Fisica.worldToScreen(18)), (int)(hAPI_Fisica.worldToScreen(18)));
}

/* Z Box */
stage[0] = loadImage(img_file_path + "stage0.png"); 
stage[1] = loadImage(img_file_path + "stage1.png"); 
stage[2] = loadImage(img_file_path + "stage2.png");
stage[3] = loadImage(img_file_path + "stage3.png");
stage[4] = loadImage(img_file_path + "stage4.png");
stage[5] = loadImage(img_file_path + "stage5.png");
for (int i=0; i < stage.length; i++){
stage[i].resize((int)(hAPI_Fisica.worldToScreen(12)), (int)(hAPI_Fisica.worldToScreen(12)));
}

//crosshair
hand = loadImage("../img/hand.png"); 
hand.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
crosshairOFF1 = loadImage("../img/crosshair2_OFF1.png"); 
crosshairOFF1.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
crosshairON1 = loadImage("../img/crosshair2_ON1.png"); 
crosshairON1.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
crosshairOFF2 = loadImage("../img/crosshair2_OFF2.png"); 
crosshairOFF2.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
crosshairON2 = loadImage("../img/crosshair2_ON2.png"); 
crosshairON2.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));


 /* put setup code here, run once: */
//Parinaz's new codes starts here
/* Load images for tools*/
 
 Tool1_vertical=loadImage(img_file_path + "Tool1.png");
 Tool1_vertical.resize(int(355/2),int(355/2));
 
 Tool2_vertical=loadImage(img_file_path + "Tool2.png");
 Tool2_vertical.resize(int(390/2),int(391/2));
 
 Tool3_vertical=loadImage(img_file_path + "Tool3.png");
 Tool3_vertical.resize(int(98*1.2),int(116*1.2));

 Tool4_vertical=loadImage(img_file_path + "Tool4.png");
 Tool4_vertical.resize(int(156),int(259));
     
 
 
 button00=loadImage(img_file_path + "button00.png");
 button00.resize(int(267/3),int(234/3));
 button01=loadImage(img_file_path + "button01.png");
 button01.resize(int(267/3),int(234/3));
 
 
 button10=loadImage(img_file_path + "button10.png");
 button10.resize(int(294/3),int(257/3));
 button11=loadImage(img_file_path + "button11.png");
 button11.resize(int(294/3),int(257/3));

 button20=loadImage(img_file_path + "button20.png");
 button20.resize(int(350/3),int(180/3));
 button21=loadImage(img_file_path + "button21.png");
 button21.resize(int(350/3),int(180/3));
 button30=loadImage(img_file_path + "button30.png");
 button30.resize(int(156/3),int(259/3));
 button31=loadImage(img_file_path + "button31.png");
 button31.resize(int(156/3),int(259/3));

/*how much does the tool move upon pressing q*/    
Tool_displacements [0] = 0;
Tool_displacements [1] = 35;
Tool_displacements [2] = 90;
Tool_displacements [3] = 130;
Tool_displacements [4] = 160;
Tool_displacements [5] = 200;

 // Parinaz's codes finish here

/* XY Box */
g1                  = new FBox(40, 19);
g1.setPosition(17, 11);
g1.setDensity(100);
g1.setFill(150,150,230);
g1.setName("Widget");
g1.setSensor(true);
g1.setStatic(true);
g1.setGrabbable(false);
g1.attachImage(brain_stage[0]);
world.add(g1);

g2                  = new FBox(10, 12);
g2.setPosition(45, 12);
g2.setDensity(100);
g2.setFill(230,230,0);
g2.setName("Widget");
g2.setSensor(true);
g2.setStatic(true);
g2.attachImage(stage[0]);
world.add(g2);

wall                  = new FBox(1, 22);
wall.setPosition(37, 11);
wall.setDensity(100);
wall.setFill(0,0,0);
wall.setName("Widget");
//g2.setSensor(true);
wall.setStatic(true);
world.add(wall);


/* Setup the Virtual Coupling Contact Rendering Technique */
s                   = new HVirtualCoupling((0.75)); 
s.h_avatar.setDensity(4); 
s.h_avatar.setFill(255,0,0); 
s.h_avatar.setSensor(true);
s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
s.h_avatar.attachImage(hand);

/* World conditions setup */
world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)

/* setup framerate speed */
frameRate(baseFrameRate);

/* setup simulation thread to run at 1kHz */
SimulationThread st = new SimulationThread();
scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);

  smooth();
cp5 = new ControlP5(this);
  cp5.addButton("ResetDevice")
 .setValue(0)
 .setPosition(10,480)
 .setSize(100,50)
 ;
//cp5.draw();
FSRtxt=cp5.addTextlabel("Prop")
                .setText(str(0.0))
                .setPosition(700,50)
                .setColorValue(color(170,20,20))
                .setFont(createFont("Georgia",40))
                ;               
tasktxt=cp5.addTextlabel("Task")
                .setText("Step 1/5")
                .setPosition(100,20)
                .setColorValue(color(170,20,20))
                .setFont(createFont("Georgia",40))
                ;             
 timertxt=cp5.addTextlabel("Time")
                .setText("Times "+str(0.0))
                .setPosition(815,10)
                .setColorValue(color(50,155,50))
                .setFont(createFont("Georgia",40))
                ;
 overshoottxt=cp5.addTextlabel("Overshoot")
                .setText("Over# 0.00")
                .setPosition(540,10)
                .setColorValue(color(50,155,50))
                .setFont(createFont("Georgia",40))
                ;
                

 /* new buttons for tools */
//Parinaz's codes start here**********************************************************************
Tool1=cp5.addButton("Tool1")
 .setPosition(800,100)
 .setImages(button00, button00, button01)
 .updateSize();
// change the behavior of a button with setSwitch 
// so it behaves like a switch, by default the state of the
// switch is false.
Tool1.setSwitch(true);
// set the state of the switch to true by using Button.setOn(), set it to
// false using Button.setOff()
 // b.setOn();
Tool2=cp5.addButton("Tool2")
 .setPosition(810,200)
 .setImages(button10, button10, button11) 
 .updateSize();
Tool2.setSwitch(true);

Tool3=cp5.addButton("Tool3")
 .setPosition(790,320)
 .setImages(button20, button20, button21) 
 .updateSize();
Tool3.setSwitch(true);

Tool4=cp5.addButton("Tool4")
 .setPosition(830,420)
 .setImages(button30, button30, button31) 
 .updateSize();
Tool4.setSwitch(true);
//Parinaz's codes finish here**********************************************************************************************************

//section for gradient   
controlP5 = new ControlP5(this);

//change the original colors
controlP5.setColorForeground(lerpColor(from, to, 0.5));
controlP5.setColorBackground(color(150, 158, 159));
controlP5.setColorActive(lerpColor(from, to, 0.5));

c3=controlP5.addSlider("Force(%)")
.setRange(0, 2000)
.setValue(20)
.setPosition(700, 100)
.setSize(60, 400)
.setColorValue(200)
.setColorLabel(200);

// draw controls manually so that you can draw on top of them
controlP5.setAutoDraw(false);
 

right_back = createShape(RECT, 920,0, 500, 550);
right_back.setStroke(color(50));
right_back.setFill(color(100,100,100));

//moustracker
points = new ArrayList<PVector>();

//target

////Start Devyani's code*********************************************************************************
//cut_target0 = new FPoly();

//drawFPolyTarget(0, radius, factor, 2*3.14); 
////cut_target[0].setFill(100,45,45,90); //fourth input sets transparency
////End Devyani's code*************************************************************************************

 targetA = createShape();
 targetB = createShape(GROUP);
 for (float i=0.02; i < 2*3.14 ; i=i+0.12){
 targetB1 = createShape(LINE,cos(i-0.04)*(radius-factor)+355, sin(i-0.04)*(radius-15)+215, cos(i-0.025)*(radius-factor)+355, sin(i-0.025)*(radius-15)+215);
 targetB1.setStrokeWeight(4);
 targetB1.setStroke(color(255,255,255));;
 targetB.addChild(targetB1);
 
//Start Devyani's code********************************************************************************* 
//cut_target.vertex((cos(i-0.04)*(radius-factor)+355)/25, (sin(i-0.04)*(radius-15)+215)/25); //you just had to convert back to cm!!!!
//cut_target.vertex(cos(i-0.025)*(radius-factor)+3, sin(i-0.025)*(radius-15)+2);
//End Devyani's code*************************************************************************************
 
 }
 
 targetA.beginShape();  
 for (float i=0.02; i < 2*3.14 ; i=i+0.02){
 targetA.stroke(0);
 targetA.strokeWeight(4);
 targetA.noFill();
 targetA.vertex(cos(i-0.02)*(radius-factor)+355, sin(i-0.02)*(radius-15)+215);
 }
 targetA.endShape(); 
 
//start_point
start_x=(radius-factor)+355;
start_y=215;
start_point= createShape(ELLIPSE, start_x,start_y, 15, 15);
start_point.setStroke(color(100,0,0));
start_point.setFill(false);
lastval_x=(radius-factor)+355;
lastval_y=215;

//check_point
checkpoint_x=-(radius-factor)+355;
checkpoint_y=215;
check_point= createShape(ELLIPSE, checkpoint_x,checkpoint_y, 15, 15);
check_point.setStroke(color(100,0,0));
check_point.setFill(false);

//mil_target
mil_x=new float[3];
mil_y=new float[3];
//milling condition
milcond_x=new float[3];
milcond_y=new float[3];
target_mil = new PShape[3];
for (int i=0; i < 3 ; i=i+1){
mil_x[i]= cos(2*i-(3.14/2))*(radius-16)+357; 
mil_y[i]= sin(2*i-(3.14/2))*(radius-15-16)+215;
target_mil[i]= createShape(ELLIPSE, mil_x[i],mil_y[i], 15, 15);
target_mil[i].setStroke(color(100,0,0));
target_mil[i].setFill(false);
}

 
 //targetFSR
targetFSR = createShape(RECT, 0,0, 60, 80);
targetFSR.setStroke(color(0));
targetFSR.setFill(false);

//Start Devyani's code********************************************************************************* 
//world.add(cut_target);
//End Devyani's code*************************************************************************************

//target
y_targetFSR=new int[6];
y_targetFSR[0] = 10;
y_targetFSR[1] = 50;
y_targetFSR[2] = 50;
y_targetFSR[3] = 10;
y_targetFSR[4] = 5;
y_targetFSR[5] = 5;
xArr = new float[10000];
yArr = new float[10000];

//fsr smoorthing
readings=new float[numReadings];
miltimes=new int[3];
for (int i=0; i < 3 ; i=i+1){
miltimes[i]=0;
}
miltimes_temp=new int[3];
for (int i=0; i < 3 ; i=i+1){
miltimes_temp[i]=0;
}


//Start Devyani's code*********************************************************************************

stage_damping = new int[6];
stage_damping[0] = 500; // skin
stage_damping[1] = 950; // skull
stage_damping[2] = 700; // dura (?) 
stage_damping[3] = 800; // brain turmor
stage_damping[4] = 1000; // ? 
stage_damping[5] = 1000; // ?

c = new FCircle(2);
c.setPosition(51,22);
c.setFill(100,200,200);
//world.add(c);
  

//End Devyani's code*************************************************************************************
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw(){
 m=millis();
 seconds = m/50;//200 Hz timer
 seconds_real=m/1000;
 
 
/* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
if(renderingForce == false){
background(255);
textFont(f, 22);
cp5.draw();
shape(right_back);
right_back.setFill(color(100,100,100));
world.draw();
targetA.setStroke(color(255,0,0));  
shape(targetA);
shape(targetB);
if (drawMode==true){
noFill();
stroke(0,230,0);
strokeWeight(3);  
beginShape();
if (counter_points>0) {
for (int i = 0; i < counter_points; i++) {
  curveVertex(xArr[i], yArr[i]);      
}
val_x=xArr[counter_points-1];
val_y=yArr[counter_points-1];
}
endShape();
if (counter_stage!=1){
check_point.setStrokeWeight(3);
shape(check_point);
}
}else{
  noFill();
  stroke(0,230,0);
  strokeWeight(3);  
  beginShape();
  if (counter_points>0) {
  for (int i = 0; i < counter_points; i++) {
  curveVertex(xArr[i], yArr[i]);      
  }
  }
  endShape();
}

 //shape(cal);
/*drawing tools on the right screen*/
//Parinaz's codes start here*********************************************************************************
 
if (Tool1.isOn()==true){   
  pdepth=0;
    if (counter_stage>=1) {
    image(Tool1_vertical,240+800,-16+Tool_displacements[counter_stage]);}
    else {
      image(Tool1_vertical,240+800,-16+Tool_displacements[0]);
    }
}    
    if (Tool2.isOn()==true){   
      pdepth=0;
    if (counter_stage>=1) {
    image(Tool2_vertical,230+800,-30+Tool_displacements[counter_stage]);}
    else {
    image(Tool2_vertical,239+800,-30+Tool_displacements[0]);
    }       
}    
   if (Tool3.isOn()==true){ 


   float delta_t=timeS_real-oldtimes;
   println("delta_t:"+delta_t);
   //force=Math.random()*30+20;
   force=average/2000*100; // force is the amount we get from the fsr
   
   double rpm=500; //500 rpm
   double N=2*PI*rpm/60;
   double CT=134.6097;
   double CN=-0.3327;
   double Cf=0.5189;
   double CD=1.1841;
   double D=1700; //density for bone= Math.pow(x, i);
   double v;
 
   v= Math.pow((force*Math.pow(N,-CN)*Math.pow(D,-CD))/CT,-Cf);  //feedrate
   println("force:"+ force + "  time:"+ seconds_real+"  velocity:"+ v); 
   
   float new_depth=(float)(delta_t*v*0.0002); //change the number later
   println("New depth: "+delta_t*   v*0.0002+"   rounded: "+new_depth);
   if (force>50){   //condition for force 
    // depth= depth+int(delta_t*force*0.002);
     pdepth= pdepth+new_depth;
   }

   println("pdepth"+pdepth);
    image(Tool3_vertical,265+800,50+pdepth);
  
}
   if (Tool4.isOn()==true){ 
     pdepth=0;
    if (counter_stage>=1) {
    image(Tool4_vertical,250+800,-90+Tool_displacements[counter_stage]);}
    else {
    image(Tool4_vertical,250+800,-90  +Tool_displacements[0]);
    }
}
//Parinaz'z codes finish here**************************************8********************************************
//slider
// slider 2: render default with cp5.draw() then draw gradient on top  
controlP5.draw();
//Controller c2 = controlP5.getController("grad2");
slideGradient(c3);
 pushMatrix();
translate(700, 400-4*y_targetFSR[counter_stage]);
shape(targetFSR);
targetFSR.setStrokeWeight(3);  
popMatrix();
if (counter_stage!=1){
start_point.setStroke(color(100,0,0));
start_point.setStrokeWeight(3);
shape(start_point); 
}
else{
  for (int i=0; i < 3 ; i=i+1){
  target_mil[i].setStroke(color(100,0,0));
  target_mil[i].setStrokeWeight(3);
  shape(target_mil[i]);
}
}
}
}
/* end draw section ****************************************************************************************************/


void slideGradient(Slider c) {
float[] p = c.getPosition();
float amt = c.getValue()/(c.getMax()-c.getMin());
slideGradient(int(p[0]), int(p[1]), c.getWidth(), c.getHeight(), from, to, amt);
}
void slideGradient(int x, int y, float w, float h, color c1, color c2, float amt) {
int pos = int((y+h)-(h*amt));
for (int i = int(y+h); i >= pos; i--) {
float inter = map(i, y, y+h, 0, 1);
color cc = lerpColor(c1, c2, inter);
stroke(cc);
line(x, i, x+w, i);
}
}


void keyPressed() {
if (key == 'q') {   
  counter_overshoot=0;//refresh overshoot
  oldtimes=timeS_real;//refresh timer
  overshoottxt.setText("Over# " + str(counter_overshoot/100)); 
  
  float circ = 2*3.14;
  float n=0;//millingtargets
  
  // changes the stage
  counter_stage=counter_stage+1;
  if (counter_stage==6){
  counter_stage=0;
  }   

g1.attachImage(brain_stage[counter_stage]);
g2.attachImage(stage[counter_stage]);
tasktxt.setText("Step " + str(counter_stage+1)+"/5"); 

 /* work in progress ash ******************************************************************************************* */
 //target  
 targetA = createShape();
 targetB = createShape(GROUP);
 factor=counter_stage*16-4; 
if (counter_stage==3){
  factor = (counter_stage-1)*16-4;
}
 if (counter_stage==2){
  factor = (counter_stage-1)*16-4;
}
 start_x=(radius-factor)+355;
 start_y=215;
  checkpoint_x=-(radius-factor)+355;
 checkpoint_y=215;
 lastval_x=(radius-factor)+355;
 lastval_y=215; 
if (counter_stage==4){
   circ = 0.02;
   start_x=355;  
   checkpoint_x=355;
   lastval_x=355;
} 
if (counter_stage==1){
  circ = 0.02;
}
 if (counter_stage==5){
  tasktxt.setText("Task Complete");
  factor=0;
  start_x=355; 
  checkpoint_x=355;
} 
/* work in progress ash ******************************************************************************************* */
 for (float i=0.02; i < circ ; i=i+0.12){
 targetB1 = createShape(LINE,cos(i-0.04)*(radius-factor)+357, sin(i-0.04)*(radius-15-factor)+215, cos(i-0.025)*(radius-factor)+357, sin(i-0.025)*(radius-15-factor)+215);
 //cut_target0.vertex((cos(i-0.04)*(radius-factor)+355)/25, (sin(i-0.04)*(radius-15)+215)/25);
 targetB1.setStrokeWeight(4);
 targetB1.setStroke(color(255,255,255));;
 targetB.addChild(targetB1);
 }
 
//  //Start Devyani's code*********************************************************************************
//      //world.removeBody(cut_target3);
//  drawFPolyTarget(counter_stage, radius, factor, circ);

//  curr_damping = stage_damping[counter_stage];

////End Devyani's code*************************************************************************************
 
 targetA.beginShape();  
 for (float i=0.02; i < circ ; i=i+0.02){
 targetA.stroke(0);
 targetA.strokeWeight(4);
 targetA.noFill();
 targetA.vertex(cos(i-0.02)*(radius-factor)+357, sin(i-0.02)*(radius-15-factor)+215);
 }
 targetA.endShape();
 //startpoint
 start_point= createShape(ELLIPSE, start_x,start_y, 15, 15);
 start_point.setStroke(color(100,0,0));
 check_point= createShape(ELLIPSE, checkpoint_x,checkpoint_y, 15, 15);
 check_point.setStroke(color(100,0,0));
}else if (key == 'w') {
depth += 20;
c3.setValue(depth);
}
else if (key == 'a') {
if (drawMode==true){
  drawMode=false;
}
else{
  drawMode=true;
}
}
}
public void loadSlider(float c) {    
c3.setValue(c);
}

public void ResetDevice(int theValue) {    
widgetOne.device_set_parameters();
//s.setToolPosition(edgeTopLeftX+worldWidth/2, edgeTopLeftY); 
//s.updateCouplingForce();
}
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{

public void run(){
/* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

renderingForce = true;

if(haplyBoard.data_available()){
  /* GET END-EFFECTOR STATE (TASK SPACE) */
  widgetOne.device_read_data();
  //FSR to Slider
  fsr= widgetOne.get_sensor_data();  
  
  knobval=fsr[0];  
  
//Start Devyani's code*********************************************************************************
  if (s.h_avatar.isTouchingBody(cut_target0)) {
    if (counter_stage == 0) {
      
      s.h_avatar.setDamping(stage_damping[0]); // in the beginning when 'q' has never been pressed
      
    }
    s.h_avatar.setDamping(curr_damping);
  }
  else {
    
    s.h_avatar.setDamping(3);
    
  }
//End Devyani's code*************************************************************************************

  angles.set(widgetOne.get_device_angles()); 
  posEE.set(widgetOne.get_device_position(angles.array()));
  posEE.set(posEE.copy().mult(200));  
 // float xE = -1*pixelsPerCentimeter * posEE.x + 400;
 //200 Hz timer
  if (seconds>timeS){
  timeS=seconds;
  //fsr smoothing
  // subtract the last reading:
  total = total - readings[readIndex]; // read from the sensor:
  readings[readIndex] = knobval;// add the reading to the total:
  total = total + readings[readIndex];// advance to the next position in the array:
  readIndex = readIndex + 1;// if we're at the end of the array...
  if (readIndex >= numReadings) {// ...wrap around to the beginning:
    readIndex = 0;
  }// calculate the average:
  average = total / numReadings;
  //load data to display slider
  loadSlider(average);
  
  averageval=average/2000*100;
  FSRtxt.setText((nf((average/2000*100), 0, 2)));
  float compareval=y_targetFSR[counter_stage]+5;//+5
  if (averageval>compareval && averageval<(compareval+20)){ 
      targetFSR.setStroke(color(0,255,0));
      //s.h_avatar.attachImage(hand); 
      widgetOne.set_pwm_pulse(4, 200);
      force_condition=true;
  }else if(averageval>compareval+20){
      targetFSR.setStroke(color(0,0,0)); 
      counter_overshoot=counter_overshoot+1;
      overshoottxt.setText("Over# " + str(counter_overshoot/100)); 
      //s.h_avatar.attachImage(hand);
      widgetOne.set_pwm_pulse(4, 0);  
      force_condition=false;
  }else{
      targetFSR.setStroke(color(0,0,0)); 
      //s.h_avatar.attachImage(hand);
      widgetOne.set_pwm_pulse(4, 0);  
      force_condition=false;
  }  
  
  //drawing
  float xE = (edgeTopLeftX+worldWidth/2-(posEE).x)*pixelsPerCentimeter ;
  float yE = (edgeTopLeftY+(posEE).y-7) * pixelsPerCentimeter;
  float startcond_x = abs(start_x - xE);
  float startcond_y = abs(start_y - yE);
  float checkcond_x = abs(checkpoint_x - xE);
  float checkcond_y = abs(checkpoint_y - yE);
  float runcond_x = abs(lastval_x - xE);  
  float runcond_y = abs(lastval_y - yE);
  for (int i=0; i < 3 ; i=i+1){
  milcond_x[i]= abs(mil_x[i] - xE); 
  milcond_y[i]= abs(mil_y[i] - yE);
  }
  //println(milcond_x[0] + " " + milcond_x[1] + " " + milcond_x[2]);
  if (startcond_x<3 && startcond_y<3){
    s.h_avatar.attachImage(crosshairOFF1);
  }   
  if (checkcond_x<3 && checkcond_y<3){
    check_condition=true;
    check_point.setStroke(color(0,255,0));
  }   
  if (runcond_x<5 && runcond_y<5){
    s.h_avatar.attachImage(crosshairOFF1);
  }
  else{
    s.h_avatar.attachImage(hand);
  }
  //if ((milcond_x[0]<3 && milcond_y[0]<3)||(milcond_x[1]<3 && milcond_y[1]<3)||(milcond_x[2]<3 && milcond_y[2]<3)){
  //  s.h_avatar.attachImage(crosshairOFF1);
  //}
  for (int i=0; i < 3 ; i=i+1){//milling condition
  if ((milcond_x[i]<3 && milcond_y[i]<3) && counter_stage==1){
    s.h_avatar.attachImage(crosshairOFF1);   
    if (force_condition==true){      
      println("Depth completed: " + (timeS-miltimes[i])+"/150");  
      miltimes_temp[i] = timeS-miltimes[i];      
    }
    else{
      miltimes[i]=timeS-miltimes_temp[i];
    }    
  }
  }
  if (runcond_x<5 && runcond_y<5 && force_condition==true){//for the actual cutting
    s.h_avatar.attachImage(crosshairON1);
    drawMode=true;
  }
  else{
    drawMode=false;
  }
  
  if (drawMode==true){    
    float movement_points = abs(val_x - xE);
    //println(timeS  + " " + xE + " " + xArr.length + " " + movement_points);    
    //add a condition that if endeffector is steady no points are added
    if (movement_points>0.01){
    xArr[counter_points]=(edgeTopLeftX+worldWidth/2-(posEE).x)*pixelsPerCentimeter;
    yArr[counter_points]=(edgeTopLeftY+(posEE).y-7) * pixelsPerCentimeter;
    counter_points=counter_points+1;
    //points.add(new PVector(xE, yE));
    lastval_x=(edgeTopLeftX+worldWidth/2-(posEE).x)*pixelsPerCentimeter;
    lastval_y=(edgeTopLeftY+(posEE).y-7) * pixelsPerCentimeter;
  }
  }
  else {        
  }
}
 if (seconds_real>timeS_real){
  timeS_real=seconds_real;
  timertxt.setText("Time " + nf(timeS_real-oldtimes,3,0));
 }
}
//println((edgeTopLeftX+worldWidth/2-(posEE).x)*25); 
s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
s.updateCouplingForce();

  if(s.h_avatar.isTouchingBody(g2)){
  float shake = random(-4, 4);
  fEE.set(-s.getVirtualCouplingForceX()/100000 + 3*shake, s.getVirtualCouplingForceY()/100000 + 0.03*shake);
} else{
  fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());  
  fEE.div(100000);
}
//dynes to newtons      
torques.set(widgetOne.set_device_torques(fEE.array()));
widgetOne.device_write_torques();    
world.step(1.0f/1000.0f);  
renderingForce = false;
}
}
/* end simulation section **********************************************************************************************/
/*To control different events (Tools' appearance on the screen)*/
//Parinaz's codes start here**************************************************************************************************************
public void controlEvent(ControlEvent theEvent) {
//println(theEvent.getController().getName());
 
if (theEvent.getController().getName()=="Tool1"){
if (Tool1.isOn()==true){
                Tool2.setOff();
          Tool3.setOff();
          Tool4.setOff();
}
}
 if (theEvent.getController().getName()=="Tool2"){
          if (Tool2.isOn()==true){
          Tool1.setOff();
          Tool3.setOff();
          Tool4.setOff();
        }
}
   if (theEvent.getController().getName()=="Tool3"){
          if (Tool3.isOn()==true){
          Tool1.setOff();
          Tool2.setOff();
          Tool4.setOff();    
        }
}
     if (theEvent.getController().getName()=="Tool4"){
          if (Tool4.isOn()==true){
          Tool1.setOff();
          Tool2.setOff();
          Tool3.setOff();
        }
}
}

//Parinaz's codes finish here**********************************************************************************************************



//Start Devyani's code*********************************************************************************


public void drawFPolyTarget(int curStage, float radius, float factor, float circ) {
 if (cut_target0 != null) {
   System.out.println("HELELELLELELELELELE"); 
   world.remove(cut_target0);
   cut_target0 = null;
 }
 cut_target0 = new FPoly();
 cut_target0.setFill(100,45,45,90);
 cut_target0.setNoStroke();
 for (float i=0.02; i < 2*3.14 ; i=i+0.12){
    
   //cut_target0.vertex((cos(i-0.04)*(radius-factor)+355)/pixelsPerCentimeter, (sin(i-0.04)*(radius-15)+215)/pixelsPerCentimeter); //you just had to convert back to cm!!!!
   cut_target0.vertex((cos(i-0.02)*(radius-factor+10)+357)/pixelsPerCentimeter, (sin(i-0.02)*(radius-15-factor+10)+215)/pixelsPerCentimeter);
 }
 cut_target0.setFill(100,45,45, 90);
 if (cut_target0 != null) {
 world.add(cut_target0);
  
 }

}


//End Devyani's code*************************************************************************************
