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
PImage stage2_1 ;
PImage stage2_2 ;
PImage powerOFF;PImage powerON;
PImage redoOFF; PImage redoON;

PImage[] brain_stage = new PImage[6];
int counter_stage=0;

//depth axis
float depth=0.0;

//mouse 
boolean mouseHasBeenPressed= false;

//background
PShape left_back;
PShape right_back;
PShape top_back;
PShape below_back;
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
 float radius = 105;
 PShape targetA;
 PShape targetB;
 PShape targetB1;
 float[] factor;

//FSR
float[] fsr;
float knobval;

//Strings
String[] disptxt;

//Slider
ControlP5 controlP5;
Slider c3;

//targetFSR
Textlabel FSRtxt;
Textlabel tasktxt;
Textlabel starttxt;
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

//power buttons and redo buttons
boolean gamemode=false;
Toggle powerbtn;
Button redobtn;
int Setuptime;

//completion
int[] milcomplete;
int[] stagecomplete;

//smoothing
int numReadings=25;  //PArinaz changed this
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
  float Tool1_depth=0;
  float Tool2_depth=0;
  float Tool3_depth=0;
  
  double force;
  int counter_tool1=1;
int counter_tool3=1;

float new_depth1;
float new_depth2;
float new_depth3;

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
//power buttons
 powerOFF=loadImage(img_file_path + "powerOFF.png");
 powerOFF.resize(int(180),int(60)); 
 redoOFF=loadImage(img_file_path + "redoOFF.png");
 redoOFF.resize(int(180),int(60));
 powerON=loadImage(img_file_path + "powerON.png");
 powerON.resize(int(180),int(60)); 
 redoON=loadImage(img_file_path + "redoON.png");
 redoON.resize(int(180),int(60));

//Parinaz's new codes*************************************
stage2_1 = loadImage(img_file_path + "stage2-1.png");
stage2_2 = loadImage(img_file_path + "stage2-2.png");
stage2_1.resize((int)(hAPI_Fisica.worldToScreen(12)), (int)(hAPI_Fisica.worldToScreen(12)));
stage2_2.resize((int)(hAPI_Fisica.worldToScreen(12)), (int)(hAPI_Fisica.worldToScreen(12)));
//Parinaz's new codes********************************************


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
 Tool1_vertical.resize(int(460/2),int(460/2));
 
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
Tool_displacements [3] = 90;
Tool_displacements [4] = 130;
Tool_displacements [5] = 130;

 // Parinaz's codes finish here

/* XY Box */
g1                  = new FBox(40, 19);
g1.setPosition(21, 12);
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

//cp5.draw();
FSRtxt=cp5.addTextlabel("Prop")
                .setText(str(0.0))
                .setPosition(235,500)
                .setColorValue(color(210,230,213))
                .setFont(createFont("Georgia",25))
                ;   
starttxt=cp5.addTextlabel("Starttxt")
                .setText("Click Start")
                .setPosition(400,300)
                .setColorValue(color(210,230,213))
                .setFont(createFont("Georgia",35))
                ;                        
tasktxt=cp5.addTextlabel("Task")
                .setText("Step 1/5")
                .setPosition(245,5)
                .setColorValue(color(210,230,213))
                .setFont(createFont("Georgia",35))
                ;             
 timertxt=cp5.addTextlabel("Time")
                .setText("Time     "+str(0.0))
                .setPosition(25,370)
                .setColorValue(color(210,230,213))
                .setFont(createFont("Georgia",25))
                ;
 overshoottxt=cp5.addTextlabel("Overshoot")
                .setText("Over#    0.00")
                .setPosition(25,400)
                .setColorValue(color(210,230,213))
                .setFont(createFont("Georgia",25))
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
 .setPosition(790,320)
 .setImages(button10, button10, button11) 
 .updateSize();
Tool2.setSwitch(true);

Tool3=cp5.addButton("Tool3")
 .setPosition(795,210)
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

c3=controlP5.addSlider("")
.setRange(0, 2000)
.setValue(20)
.setPosition(250, 100)
.setSize(60, 400)
.setColorValue(200)
.setColorLabel(200);

// draw controls manually so that you can draw on top of them
controlP5.setAutoDraw(false);
 
//right background
right_back = createShape(RECT, 920,0, 500, 550);
right_back.setStroke(color(50));
right_back.setFill(color(75,94,119));
//stats background
left_back = createShape(RECT, 20,350, 180, 120);
left_back.setStroke(color(50));
left_back.setFill(color(75,94,119));
//top background
top_back = createShape(RECT, 245,10, 500, 40);
top_back.setStroke(color(50));
top_back.setFill(color(75,94,119));
//below background
below_back = createShape(RECT, 235, 505, 85, 30);
below_back.setStroke(color(50));
below_back.setFill(color(75,94,119));

//moustracker
points = new ArrayList<PVector>();

//target
//factor
factor=new float[6];
factor[0] = 0;
factor[1] = 20;
factor[2] = 20;
factor[3] = 40;
factor[4] = 40;
factor[5] = 0;

////Start Devyani's code*********************************************************************************
//cut_target0 = new FPoly();

//drawFPolyTarget(0, radius, factor, 2*3.14); 
////cut_target[0].setFill(100,45,45,90); //fourth input sets transparency
////End Devyani's code*************************************************************************************

 targetA = createShape();
 targetB = createShape(GROUP);
 for (float i=0.02; i < 2*3.14 ; i=i+0.12){
 targetB1 = createShape(LINE,cos(i-0.04)*(radius-factor[0])+460, sin(i-0.04)*(radius-15)+210, cos(i-0.025)*(radius-factor[0])+460, sin(i-0.025)*(radius-15)+210);
 targetB1.setStrokeWeight(4);
 targetB1.setStroke(color(255,255,255));;
 targetB.addChild(targetB1);
 
//Start Devyani's code********************************************************************************* 
//cut_target.vertex((cos(i-0.04)*(radius-factor)+460)/25, (sin(i-0.04)*(radius-15)+210)/25); //you just had to convert back to cm!!!!
//cut_target.vertex(cos(i-0.025)*(radius-factor)+3, sin(i-0.025)*(radius-15)+2);
//End Devyani's code*************************************************************************************
 
 }
 
 targetA.beginShape();  
 for (float i=0.02; i < 2*3.14 ; i=i+0.02){
 targetA.stroke(0);
 targetA.strokeWeight(4);
 targetA.noFill();
 targetA.vertex(cos(i-0.02)*(radius-factor[0])+460, sin(i-0.02)*(radius-15)+210);
 }
 targetA.endShape(); 
 
//start_point
start_x=(radius-factor[0])+460;
start_y=210;
start_point= createShape(ELLIPSE, start_x,start_y, 15, 15);
start_point.setStroke(color(100,0,0));
start_point.setFill(false);
//lastval_x=(radius-factor)+460;
//lastval_y=210;

//check_point
checkpoint_x=-(radius-factor[0])+460;
checkpoint_y=210;
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
mil_x[i]= cos(2*i-(3.14/2))*(radius-factor[2])+460; 
mil_y[i]= sin(2*i-(3.14/2))*(radius-15-factor[2])+210;
target_mil[i]= createShape(ELLIPSE, mil_x[i],mil_y[i], 20, 20);
target_mil[i].setStroke(color(100,0,0));
target_mil[i].setFill(false);
}

//display
disptxt =new String[7];
disptxt[0] = "Step 0/5  Cranitotomy Surgery";
disptxt[1] = "Step 1/5  Scalp Incision";
disptxt[2] = "Step 2/5  Drill 3 Burr Holes";
disptxt[3] = "Step 3/5  Skull Incision";
disptxt[4] = "Step 4/5  Dura Mater Incision";
disptxt[5] = "Step 5/5  Tumor Extraction";
disptxt[6] = "Task Complete";
 
 //targetFSR
targetFSR = createShape(RECT, 0,0, 60, 160);
targetFSR.setStroke(color(0));
targetFSR.setFill(false);

  powerbtn=cp5.addToggle("power")
         .setValue(0)
         .setPosition(20,150)
         .setSize(180,60)
         .setImages(powerOFF,powerOFF,powerON)
 ;
 powerbtn.setState(false); 
 gamemode=false;
 tasktxt.setText(disptxt[0]);
 redobtn=cp5.addButton("redo")
         .setValue(0)
         .setPosition(20,220)
         .setSize(180,60)
         .setImages(redoOFF,redoOFF,redoON)
 ;

//Start Devyani's code********************************************************************************* 
//world.add(cut_target);
//End Devyani's code*************************************************************************************

//target
y_targetFSR=new int[6];
y_targetFSR[0] = 10;
y_targetFSR[1] = 30;
y_targetFSR[2] = 30;
y_targetFSR[3] = 10;
y_targetFSR[4] = 20;
y_targetFSR[5] = 60;
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

//completion
milcomplete = new int[3];
for (int i=0; i < 3 ; i=i+1){
  milcomplete[i]=0;
}
stagecomplete = new int[6];
for (int i=0; i < 6 ; i=i+1){
  stagecomplete[i]=0;
}
Setuptime=millis();
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
c.setFill(100,210,210);
//world.add(c);
  

//End Devyani's code*************************************************************************************
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw(){
 m=millis();
 seconds = m/50;//210 Hz timer
 seconds_real=m/1000;
/* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
if(renderingForce == false){
background(210,230,213);
textFont(f, 22);
cp5.draw();
top_back.setFill(color(75,94,119));
shape(top_back);
right_back.setFill(color(75,94,119));
shape(right_back);
left_back.setFill(color(75,94,119));
shape(left_back);
if(gamemode == true){
below_back.setFill(color(75,94,119));
shape(below_back);
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
check_point.setFill(false);   
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
//slider
// slider 2: render default with cp5.draw() then draw gradient on top  
controlP5.draw();
//Controller c2 = controlP5.getController("grad2");
slideGradient(c3);
 pushMatrix();
translate(250, 400-4*y_targetFSR[counter_stage]-4*15);
shape(targetFSR);
targetFSR.setStrokeWeight(3);  
popMatrix();
if (counter_stage!=1){
start_point.setStroke(color(100,0,0));
start_point.setStrokeWeight(3);
start_point.setFill(false);
shape(start_point); 
}
else{
  for (int i=0; i < 3 ; i=i+1){
  //target_mil[i].setStroke(color(100,0,0));
  target_mil[i].setStrokeWeight(3);
  shape(target_mil[i]);
}
}
}
 //shape(cal);
/*drawing tools on the right screen*/
//Parinaz's codes start here*********************************************************************************
 
if (Tool1.isOn()==true){   
  Tool3_depth=0;
  
   float delta_t=timeS_real-oldtimes;
   //println("delta_t:"+delta_t);
   //force=Math.random()*30+20;
   force=average/2000*100; // force is the amount we get from the fsr
   
   double  v= 0.1;  //feedrate
  // println("counter stage"+counter_stage+"force1:"+ force + "  time1:"+ seconds_real+"  velocity1:"+ v); 
   
   new_depth1=(float)(delta_t*v); //change the number later
   //println("New depth1: "+delta_t*  v+"   rounded1: "+new_depth1);

 
    
             if (counter_tool1==1 &&counter_stage==0){    
            image(Tool1_vertical,210+800,-70+Tool_displacements[counter_stage]+Tool1_depth);
          }

              
              if (counter_stage>0 ){     
            image(Tool1_vertical,210+800,-70+Tool_displacements[counter_stage]);}
            //println("you should select another tool");
              }

}
    if (Tool2.isOn()==true){   
      Tool3_depth=0;
       Tool1_depth=0;
       
       
       
          float delta_t=timeS_real-oldtimes;
  // println("delta_t:"+delta_t);
   //force=Math.random()*30+20;
   force=average/2000*100; // force is the amount we get from the fsr
   
   double  v= 0.1;  //feedrate
  // println("counter stage"+counter_stage+"force1:"+ force + "  time1:"+ seconds_real+"  velocity1:"+ v); 
   
   new_depth2=(float)(delta_t*v); //change the number later
   //println("New depth1: "+delta_t*  v+"   rounded1: "+new_depth2);
       
    if (counter_stage>=1) {
    image(Tool2_vertical,230+800,-36+Tool_displacements[counter_stage]+Tool2_depth);}
    else {
    image(Tool2_vertical,230+800,-36+Tool_displacements[0]);
    }       
}    
if (Tool3.isOn()==true){ 
    Tool1_depth=0;
   float delta_t=timeS_real-oldtimes;
   //println("delta_t:"+delta_t);
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
   //println("force:"+ force + "  time:"+ seconds_real+"  velocity:"+ v); 
   
   new_depth3=(float)(delta_t*v*0.0002); //change the number later
   //println("New depth: "+delta_t*   v*0.0002+"   rounded: "+new_depth3);
   
   if (counter_tool3==1){     
    image(Tool3_vertical,305+800,63+Tool3_depth);
   }
   if (counter_tool3==2){     
    image(Tool3_vertical,265+800,63+Tool3_depth);
     g2.attachImage(stage2_1);
   }
   if (counter_tool3==3){ 
     image(Tool3_vertical,225+800,63+Tool3_depth);
     g2.attachImage(stage2_2); 
   }
   if (counter_tool3==4 && counter_stage==2){ 
     image(Tool3_vertical,305+800,63+Tool3_depth);
     //counter_tool3=5;
     g2.attachImage(stage[2]); 
   } 
   println("counter_stage"+counter_stage);
}

   if (Tool4.isOn()==true){ 
     Tool3_depth=0;
       Tool1_depth=0;
    if (counter_stage>=1 ) {
    image(Tool4_vertical,250+800,-90+Tool_displacements[counter_stage]);}
    else {
    image(Tool4_vertical,250+800,-90  +Tool_displacements[0]);
    }
}
//Parinaz'z codes finish here**************************************8********************************************

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
if (key == 'q') { //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx will be taken down xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  for (int i=0; i < counter_points ; i=i+1){
    xArr[i]=0;
    yArr[i]=0;    
  }
  counter_points=0;
  counter_overshoot=0;//refresh overshoot
  oldtimes=timeS_real;//refresh timer
  overshoottxt.setText("Over#  " + str(counter_overshoot/100)); 
  
  float circ = 2*3.14;
  float n=0;//millingtargets
  

  
  // changes the stage
  counter_stage=counter_stage+1;
  if (counter_stage==6){
  counter_stage=0;
  }   

g1.attachImage(brain_stage[counter_stage]);
        g2.attachImage(stage[counter_stage]);
     // }
  


 /* work in progress ash ******************************************************************************************* */
 //target  
 targetA = createShape();
 targetB = createShape(GROUP);

 start_x=(radius-factor[counter_stage])+460;
 start_y=210;
  checkpoint_x=-(radius-factor[counter_stage])+460;
 checkpoint_y=210;
 //lastval_x=(radius-factor)+460;
 //lastval_y=210; 
if (counter_stage==4){
   circ = 0.02;
   start_x=460;  
   checkpoint_x=460;
   //lastval_x=460;
} 
if (counter_stage==1){
  circ = 0.02;
  start_x=0; 
}
 if (counter_stage==5){
  tasktxt.setText("Task Complete");
  start_x=460; 
  checkpoint_x=0;
} 
/* work in progress ash ******************************************************************************************* */
 for (float i=0.02; i < circ ; i=i+0.12){
 targetB1 = createShape(LINE,cos(i-0.04)*(radius-factor[counter_stage])+460, sin(i-0.04)*(radius-15-factor[counter_stage])+210, cos(i-0.025)*(radius-factor[counter_stage])+460, sin(i-0.025)*(radius-15-factor[counter_stage])+210);
 //cut_target0.vertex((cos(i-0.04)*(radius-factor)+460)/25, (sin(i-0.04)*(radius-15)+210)/25);
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
 targetA.vertex(cos(i-0.02)*(radius-factor[counter_stage])+460, sin(i-0.02)*(radius-15-factor[counter_stage])+210);
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

public void power(int theValue) {
  if (gamemode==false){
  gamemode=true;
  tasktxt.setText(disptxt[1]);
  starttxt.setColorValue(color(210,230,213));
  starttxt.setPosition(10,10);
  }else {
    gamemode=false;
    tasktxt.setText(disptxt[0]);
    starttxt.setColorValue(color(100,0,0));
    starttxt.setPosition(400,300);
  }
  counter_stage=5;
  gameflow();
  
  milcomplete = new int[3];
for (int i=0; i < 3 ; i=i+1){
  milcomplete[i]=0;
}
stagecomplete = new int[6];
for (int i=0; i < 6 ; i=i+1){
  stagecomplete[i]=0;
}
}

public void redo(int theValue) {
  counter_stage=counter_stage-1;  
  //tasktxt.setText(disptxt[counter_stage]);
  gameflow();
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
 //210 Hz timer
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
  //average = total / numReadings;
  //average = 0.8*(total / numReadings-1630); // Parinaz added this 
  
  average = (total / numReadings); // Parinaz added this 
  //average = total / numReadings; // Parinaz commented this out
  //load data to display slider
  loadSlider(average);
  
  averageval=average/2000*100;
  FSRtxt.setText((nf((average/2000*100), 0, 2))+"%");
  float compareval=y_targetFSR[counter_stage];//+5
  if (averageval>compareval && averageval<(compareval+40)){ 
      targetFSR.setStroke(color(0,255,0));
      //s.h_avatar.attachImage(hand); 
      widgetOne.set_pwm_pulse(4, 100);
      force_condition=true;
  }else if(averageval>compareval+30){
      targetFSR.setStroke(color(0,0,0)); 
      counter_overshoot=counter_overshoot+1;
      overshoottxt.setText("Over#   " + str(counter_overshoot/100)); 
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
    lastval_x=start_x;
    lastval_y=start_y;
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
  for (int i=0; i < 3 ; i=i+1){//milling condition
  if ((milcond_x[i]<3 && milcond_y[i]<3) ){
    s.h_avatar.attachImage(crosshairOFF1);   
 if (force_condition==true){   
     //Parinaz's codes start   
     if( counter_tool3<4 && counter_stage==1)
     {
     Tool3_depth= Tool3_depth+new_depth3;
     }
     if (Tool3_depth>55 && counter_tool3<4 ){   
       counter_tool3=counter_tool3+1;
       Tool3_depth=0;
       target_mil[i].setStroke(color(0,255,0));
       mil_x[i]=0;
       mil_y[i]=0;
       milcomplete[i]=1;
              
     }
     if (counter_tool3==4 )
     {
        Tool3_depth=55;
        if (counter_stage==3){
        counter_tool3=0;
        }
     }
     
          if (counter_tool1==1 && counter_stage==1 )
     {
        Tool1_depth=0;
        counter_tool1=0;
        
        
     }
      //Parinaz's codes finish             
      miltimes_temp[i] = timeS-miltimes[i];      
    }
    else{
      miltimes[i]=timeS-miltimes_temp[i];
    }    
  }
  }
  if (runcond_x<5 && runcond_y<5 && force_condition==true){//for the actual cutting
  //Parinaz's codes start here
   //new_depth1=(float)(30*0.02); //change the number later   
     Tool1_depth= Tool1_depth+new_depth1;   
     
     if (Tool1.isOn()==true){
      if (Tool1_depth>35  ){   
       Tool1_depth=35;
     
     } 
     }
     
     if (Tool2.isOn()==true){
      Tool2_depth= Tool2_depth+new_depth2;   
     
      if (Tool2_depth>35  ){   
       Tool2_depth=35;
     } 
     }
     
     //Parinaz's codes finish here
     
    s.h_avatar.attachImage(crosshairON1);
    drawMode=true;
  }
  else{
    drawMode=false;
  }
  //stagecompletion
  if ((check_condition==true && startcond_x<3 && startcond_y<3 && force_condition ==true)||(milcomplete[0]+milcomplete[1]+milcomplete[2])==3){
    println("going next stage");
    drawMode=false;
    lastval_x=0;
    lastval_y=0;
    runcond_x=20;runcond_y=20;
    check_condition=false;
    stagecomplete[counter_stage]=1;
    println(stagecomplete);
    if (stagecomplete[5]!=1){
    tasktxt.setText(disptxt[counter_stage+2]);
    gameflow();
    }
    lastval_x=0;
    lastval_y=0;
    milcomplete[0]=0;
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
  timertxt.setText("Time     " + nf(timeS_real-oldtimes,3,0));
      if (stagecomplete[4]==1 && (timeS_real-oldtimes)==2){      
    powerbtn.setState(false); 
    gamemode=false;
    tasktxt.setText(disptxt[0]);
    }
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
    
   //cut_target0.vertex((cos(i-0.04)*(radius-factor)+460)/pixelsPerCentimeter, (sin(i-0.04)*(radius-15)+210)/pixelsPerCentimeter); //you just had to convert back to cm!!!!
   cut_target0.vertex((cos(i-0.02)*(radius-factor+10)+460)/pixelsPerCentimeter, (sin(i-0.02)*(radius-15-factor+10)+210)/pixelsPerCentimeter);
 }
 cut_target0.setFill(100,45,45, 90);
 if (cut_target0 != null) {
 world.add(cut_target0);
  
 }

}


//End Devyani's code*************************************************************************************

//Game main code--moving keypressed 'r' to here
public void gameflow() {    
  for (int i=0; i < counter_points ; i=i+1){
    xArr[i]=0;
    yArr[i]=0;    
  }
  counter_points=0;
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
     // }
  

 /* work in progress ash ******************************************************************************************* */
 //target  
 targetA = createShape();
 targetB = createShape(GROUP);
 start_x=(radius-factor[counter_stage])+460;
 start_y=210;
 checkpoint_x=-(radius-factor[counter_stage])+460;
 checkpoint_y=210;
if (counter_stage==4){
   circ = 0.02;
   start_x=460;  
   checkpoint_x=460;
} 
if (counter_stage==1){
  start_x=0;
  circ = 0.02;
}
 if (counter_stage==5){
  tasktxt.setText("Task Complete");
  start_x=460; 
  checkpoint_x=0;
} 
/* work in progress ash ******************************************************************************************* */
 for (float i=0.02; i < circ ; i=i+0.12){
 targetB1 = createShape(LINE,cos(i-0.04)*(radius-factor[counter_stage])+460, sin(i-0.04)*(radius-15-factor[counter_stage])+210, cos(i-0.025)*(radius-factor[counter_stage])+460, sin(i-0.025)*(radius-15-factor[counter_stage])+210);
 //cut_target0.vertex((cos(i-0.04)*(radius-factor)+460)/25, (sin(i-0.04)*(radius-15)+210)/25);
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
 targetA.vertex(cos(i-0.02)*(radius-factor[counter_stage])+460, sin(i-0.02)*(radius-15-factor[counter_stage])+210);
 }
 targetA.endShape();
 //startpoint
 start_point= createShape(ELLIPSE, start_x,start_y, 15, 15);
 start_point.setStroke(color(100,0,0));
 check_point= createShape(ELLIPSE, checkpoint_x,checkpoint_y, 15, 15);
 check_point.setStroke(color(100,0,0));
}
