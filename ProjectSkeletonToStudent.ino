// This program is for ENGG1100 (14-15) Sem 2 Omni-wheel car.
#include <SPI.h>
#include <Wire.h>
#include <Motor.h>
#include <HMC5883L.h>

//OUTPUT//
#define SELECT    10  //SS
#define DATAOUT   11  //MOSI
#define DATAIN    12  //MISO
#define SPICLOCK  13  //SCK

//bluetooth pin/passcode: 1937

//RobotCar Setting (which need not to be change frequently)
int stepSPD = 250;  // The maximum PWM speed for motor.
int M1SPD=250;
int M2SPD=250;
int M3SPD=250;
int turnSPD=250;
int turnMode=5;
int speedMode=5;
int minLight=75;
int maxLight=310;
int minLightT2=75;
int maxLightT2=300;
int edgeLight=280;
int t3TurnSPD=250;
int t3TurnTime1=470;
int t3TurnTime2=420;
int t3ForwardSPD=250;
int t3ForwardTime=1000;
int t3ForwardTime2=1500;

float relative_angle=0;


float M3SPDScale=0.8;
int stepS = 250;    // The PWM  (+/-) speed step setting.
unsigned long sTickTimeout = 3000; // Bluetooth connection timeout (3000)(ms);

boolean sMotorStop = false;  // force MOTOR to STOP via soft IO
boolean sLightSen = false;   // light sensor function. (enable = true, disable = false)
boolean sCompassLightSen = false;   // compass and light sensor function. (enable = true, disable = false)
boolean sPCcontrol = false;  // pc control (enable = true = control through serial comm on PC)

//RobotCar Parameters (need to be update while system working)
unsigned long pLastTick;    // Timestamp of the last reveive message via bluetooth(ms).
int RobotRot = 0;          // rotate robot setting.
int joystrickX = 128;          // value receive by joystick.
int joystrickY = 128;
byte pLEDpattern = 0x00;
boolean sStateTick = false;
boolean sLightTick = false;
boolean sCompassLightTick = false;
boolean sDeviceFlag = false;

int smState = 0;

//PIN & IO for motor control
const int M1DIR = 4; //cpu-pin-6
const int M2DIR = 7; //cpu-pin-13
const int M3DIR = 2; //cpu-pin-4
const int M1PWM = 5; //cpu-pin-11
const int M2PWM = 6; //cpu-pin-12
const int M3PWM = 9; //cpu-pin-15
Motor M1(M1DIR, M1PWM);  // every Motor treat as an object.
Motor M2(M2DIR, M2PWM);
Motor M3(M3DIR, M3PWM);

//Digitial Compass
HMC5883L dc;
float initial_angle = 0.0; float angle=0.0;
int lcState = 0;
//Bluetooth id:
//30:14:08:26:35:12
//Bluetooth Communication
String inputString = "";         // a string to hold incoming data
String outputString = "";
boolean stringComplete = false;  // whether the string is complete
String btString = "";        // String Buffer received from Bluetooth
String btDev = "";           // Device name
String btVal = "";           // Device value
char btChar = ' ';
int btStatus = 0;     // Parameter name (0), Parameter value (1)
int btState = 0;      // Detemine the current state.  (Receive X (0), receive Y (1)
String btSendStr = "";

//For debug testing
int ledFrame=0;

void setup() {
  //Motor Control
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M3DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M3PWM, OUTPUT);

  //Setup SPI
  SPI.begin();
  pinMode(SELECT, OUTPUT); // D10 // SS
  pinMode(DATAOUT, OUTPUT);// D11 // MOSI
  pinMode(DATAIN, INPUT);  // D12 // MISO
  pinMode(SPICLOCK,OUTPUT);// D13 // SCK
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //
  SPI.setBitOrder(MSBFIRST);  // MSB at the Right
  SPI.setClockDivider(SPI_CLOCK_DIV128);  //Rate of SPI equal to system clock divide by 128
  digitalWrite(SELECT, HIGH);

  delay(1);
  Serial.begin(115200);
  //Bluetooth String Setting
  inputString.reserve(200);
  outputString.reserve(200);
  btString.reserve(10);
  btDev.reserve(2);
  btVal.reserve(4);
  btSendStr.reserve(200);

  pLEDpattern = 0b00110000;    // board LED
  pLastTick = 0;

  //Set motor
  /*M1.setProperties(stepSPD, (-1*stepSPD), stepS);
  M2.setProperties(stepSPD, (-1*stepSPD), stepS);
  M3.setProperties(stepSPD, (-1*stepSPD), stepS);*/
  M1.setProperties(M1SPD, (M1SPD*-1), stepS);
  M2.setProperties(M2SPD, (M2SPD*-1), stepS);
  M3.setProperties(M3SPD, (M3SPD*-1), stepS);

  //Set Digitial Compass
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  Wire.begin();
  dc = HMC5883L(); //new instance of HMC5883L library
  dc.SetScale(1.3); //Set the scale of the compass.
  dc.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous


  //setup init heading
  initial_angle = getHeading();
}

void debugTest(){
  //play with led
  if(ledFrame%8==0){
    setLED(0b11000000);
  }
  if(ledFrame%8==1){
    setLED(0b00110000);
  }
  if(ledFrame%8==2){
    setLED(0b00001100);
  }
  if(ledFrame%8==3){
    setLED(0b00000011);
  }
  if(ledFrame%8==4){
    setLED(0b10101010);
  }
  if(ledFrame%8==5){
    setLED(0b01010101);
  }
  if(ledFrame%8==6){
    setLED(0b10101010);
  }
  if(ledFrame%8==7){
    setLED(0b01010101);
  }

//TODO: avoid delay if possible
  delay(399);


  if(ledFrame<7){
    ledFrame++;
  }else{
    ledFrame=0;
  }
}


// this is arduino program loop.
// for details, please take a look of lecture notes.
// student DO NOT modify the code.
void loop()
{
  //Serial.println("Testing");


  BluetoothCom();


  if (sDeviceFlag) {
      SetDevice();
      sDeviceFlag = false;
  }
  if ( pLastTick < (millis()-sTickTimeout) || pLastTick == 0) {
      // Bluetooth receive TIMEOUT
      M1.set(0); M2.set(0); M3.set(0);
      //For debug
      //debugTest();
      printDebugInfo();
      delay(500);
  }else{
    //Decide to do StateMachine Function or LightSensing Function
    if (sStateTick) {
      setLED(0b00110000);
      StateMachine(joystrickX, joystrickY);
      sStateTick = false;
    }else if (sLightTick) {
      LightControl();
      sLightTick = false;
    }else if (sCompassLightTick) {
      angle=initial_angle;
      LightandCompassControl();
      sCompassLightTick = false;
    }
  }

  // update motor value
  updateMotor();
  updateLED();
  slowDown();
}

  // Bluetooth Communication
  // to receive and read every byte as a commad.
  // Student DO NOT modify this function, otherwise it will not work.
void BluetoothCom() {
  while(Serial.available()) {
    btChar = (char)Serial.read();
    //Debug:
    //Serial.print(btChar);
    switch (btChar) {
      //parsing command
      case '=':       //When a '=' is received, prepare to receive the value
      case ':':
        btStatus =1;
        break;
      case ';':
        btStatus =0;
        // do something else
        sDeviceFlag = true;
        return;
        // reset Strings
        //btDev = "";

        //btVal = "";
        break;
      default:
        switch (btStatus) {
          case 0:
            btDev +=btChar;
            break;
          case 1:
            btVal +=btChar;
            break;
        }
     }
  }

}

void SetDevice() {
  // When player press a key or move the joystick, robot need to do something.
  // Student can modify part of this function.
    // when a key pressed (e.g. io0), bluetooth will receive a command  "io0=0;".
    // when a key released (e.g. io3), bluetooth will receive a command "io3=1;".
    // to specify what need to do when io4 is pressed,
    // simply go to the section "io4 pressed", or look forward until you see "btDev == "io4" and "btVal" == "0"
    // and modify the block.
  pLastTick = millis(); // this is to record the last command received. to prevent timeout or lost communication.

  // this is the joystick part. (X and Y)
  // btState 0 = set X value
  if( btDev == "x" && btState ==0) {
    btState = 1;
    joystrickY = btVal.toInt();
  }
  // btState 1 = set Y value and push value to motor
  else if (btDev == "y" && btState == 1) {
    btState = 0;
    joystrickX = btVal.toInt();
    if (sMotorStop == false) {
      if (sLightSen == true && sCompassLightSen == false) {
        sLightTick = true;
        sCompassLightTick = false;
      }else if (sCompassLightSen == true && sLightSen == false) {
        sCompassLightTick = true;
        sLightTick = false;
      }else {
        sStateTick = true;
      }
    }
  }
  // this is the keypad part. (io0 to io9)
  // io1 = control motor can move or not.
     // IO PAD:
       // io4 | io3 | io2
       // ---------------
       // io0 | io9 | io8
       // ---------------
       // io7 | io6 | io5
     // Usage:
       //io0: Set motor speed (+)
       //io2: not use || change speed mode
       //io3: start light follow function (for Task 2)
       //io4: Set motor speed (-) || change turn mode
       //io5: rotate robot clockwise
       //io6: rotate robot anti-clockwise
       //io7: stop robot
       //io8: start compass light follow function (for Task 3)
       //io9: return to manual control mode
  else if (btDev == "pc") {
    sPCcontrol = !sPCcontrol;
  }
  else if (btDev == "io0") {
    if(btVal == "0" && stepSPD<241) {  // this is what need to do then io0 is pressed, the stepSPD is an extra requiremnt.
      stepSPD = stepSPD + 10;
      M1SPD+=10;
      M2SPD+=10;
      M3SPD+=10;
/*
      M1.setProperties(stepSPD, (stepSPD*-1), stepS);
      M2.setProperties(stepSPD, (stepSPD*-1), stepS);
      M3.setProperties(stepSPD, (stepSPD*-1), stepS);
      */

      M1.setProperties(M1SPD, (M1SPD*-1), stepS);
      M2.setProperties(M2SPD, (M2SPD*-1), stepS);
      M3.setProperties(M3SPD, (M3SPD*-1), stepS);
    }
  }
  else if (btDev == "io2") {
    //TODO: add some function, eg. speed mode(high, low)
    if(speedMode>1){
      speedMode-=1;
    }else{
      speedMode=5;
    }
    M1SPD=speedMode*50;
    M2SPD=speedMode*50;
    M3SPD=speedMode*50;


    M1.setProperties(M1SPD, (M1SPD*-1), stepS);
    M2.setProperties(M2SPD, (M2SPD*-1), stepS);
    M3.setProperties(M3SPD, (M3SPD*-1), stepS);


    //printDebugInfo();
  }
  else if (btDev == "io3") {
    if(btVal == "0" ) {
      sLightSen = true;
      sCompassLightSen = false;
      smState = 0; lcState = 0;
    }
  }
  else if (btDev == "io4") {
    /*if(btVal == "0" && stepSPD>20) {
      stepSPD = stepSPD - 10;
      M1SPD-=10;
      M2SPD-=10;
      M3SPD-=10;/*
    /*  M1.setProperties(stepSPD, (stepSPD*-1), stepS);
      M2.setProperties(stepSPD, (stepSPD*-1), stepS);
      M3.setProperties(stepSPD, (stepSPD*-1), stepS);*/

      if(turnMode>1){
        turnMode-=1;
      }else{
        turnMode=5;
      }
      turnSPD=turnMode*50;



  }
  else if (btDev == "io5" && RobotRot !=2 ) {
    if (btVal == "1") {
      RobotRot = 0;
    }
    if (btVal == "0") {
      RobotRot = 1;
    }
  }
  else if (btDev == "io6" && RobotRot !=1 ) {
    if (btVal == "1") {
      RobotRot = 0;
    }
    if (btVal == "0") {
      RobotRot = 2;
    }
  }
  else if (btDev == "io7") {
    if (btVal == "1") {
      sMotorStop = false;
    }
    if (btVal == "0") {
      sMotorStop = true;
    }
  }
  else if (btDev == "io8") {
    if(btVal == "0" ) {
      sCompassLightSen = true;
      sLightSen = false;
      smState = 0; lcState = 0;
      initial_angle = getHeading();
      relative_angle=getRelativeHeading();
    }
  }
  else if (btDev == "io9") {
    if(btVal == "0" ) {
      sCompassLightSen = false;
      sLightSen = false;
      smState = 0; lcState = 0;
      joystrickX = 128;  joystrickY = 128;
    }
  }
  btDev = "";
  btVal = "";
}

// this is a statemachine of the RobotCar motion.
// it is how 6 direction works with remote control.
// for details please take a look of lecture notes.
// student need not to modify the code.
void StateMachine(int X, int Y)
{
  X = X-128;  if(X==0)X=1;  // X cannot be 0.
  Y = Y-128;  if(Y==0)Y=1;
  //TODO: handle case where multiple button are pressed together, may need to modify bluetooth protocol
  switch (smState) {
    case 0:
      M1.set(0); M2.set(0); M3.set(0);
      if( M1.getSPEED()==0 && M2.getSPEED()==0 && M3.getSPEED()==0 ) {
        if (RobotRot == 1)
          smState = 7;
        else if  (RobotRot == 2)
          smState = 8;
        else if( Y>20 && ((X>0 && X<Y) || (X<0 && -X<Y) ) )
          smState = 1;
        else if(  Y>0 && X>20 && X>=Y)
          smState = 2;
        else if( Y<0 && X>20 && X>-Y)
          smState = 3;
        else if( Y<-20 && ((X>0 && X<-Y) || (X<0 && -X<-Y) ) )
          smState = 4;
        else if( Y<0 && X<-20 && -X>-Y)
          smState = 5;
        else if( Y>0 && X<-20 && -X>=Y)
          smState = 6;
      }
      break;
    case 1:
      M1.set(-255); M2.set(0); M3.set(255);
      if( Y>20 && ((X>0 && X<Y) || (X<0 && -X<Y) ) )
        smState = 1;
      else if( Y>0 && X>20 && X>=Y)
        smState = 2;
      else if( Y>0 && X<-20 && -X>=Y)
        smState = 6;
      else if( (abs(X)<20 && abs(Y)<20) || Y<0 )
        smState = 0;
      break;
    case 2:
      M1.set(0); M2.set(-255); M3.set(255);
      if( Y>20 && ((X>0 && X<Y) || (X<0 && -X<Y) ) )
        smState = 1;
      else if( Y>0 && X>20 && X>=Y)
        smState = 2;
      else if( Y<0 && X>20 && X>-Y)
        smState = 3;
      else
        smState = 0;
      break;
    case 3:
      M1.set(255); M2.set(-255); M3.set(0);
      if( Y>0 && X>20 && X>=Y)
        smState = 2;
      else if( Y<0 && X>20 && X>-Y)
        smState = 3;
      else if( Y<-20 && ((X>0 && X<-Y) || (X<0 && -X<-Y) ) )
        smState = 4;
      else
        smState = 0;
      break;
    case 4:
      M1.set(255); M2.set(0); M3.set(-255);
      if( Y<0 && X>20 && X/Y>-1)
        smState = 3;
      else if( Y<-20 && ((X>0 && X<-Y) || (X<0 && -X<-Y) ) )
        smState = 4;
      else if( Y<0 && X<-20 && -X>-Y)
        smState = 5;
      else
        smState = 0;
      break;
    case 5:
      M1.set(0); M2.set(255); M3.set(-255);
      if( Y<-20 && ((X>0 && X<-Y) || (X<0 && -X<-Y) ) )
        smState = 4;
      else if( Y<0 && X<-20 && -X>-Y)
        smState = 5;
      else if( Y>0 && X<-20 && -X>=Y)
        smState = 6;
      else
        smState = 0;
      break;
    case 6:
      M1.set(-255); M2.set(255); M3.set(0);
      if( Y<0 && X<-20 && -X>-Y)
        smState = 5;
      else if( Y>0 && X<-20 && -X>=Y)
        smState = 6;
      else if( Y>20 && ((X>0 && X/Y<1) || (X<0 && X/Y<-1) ) )
        smState = 1;
      else
        smState = 0;
      break;
    case 7:
      // Rotate clockwisely
      M1.set(turnSPD); M2.set(turnSPD); M3.set(turnSPD);
      if (RobotRot == 0)
        smState = 0;
      break;
    case 8:
      // Rotate anti-clockwise
      M1.set(-turnSPD); M2.set(-turnSPD); M3.set(-turnSPD);
      if (RobotRot == 0)
        smState = 0;
      break;
    default:
      smState=0;
      break;

  }
}

// this is a light control function.
// student should fill in some codes.
void LightControl() {
  // A[4] represent 4 sensors used.
  int A[4] = {0, 0, 0, 0};
  int maxA = 0, maxV = 0, Asum = 0;
  // first update sensors value.
  A[0] = analogRead(A0);
  A[1] = analogRead(A1);
  A[2] = analogRead(A2);
  A[3] = analogRead(A3);
  // and find the highest value.
  maxV = A[0]; Asum =0;
  for (int i=0; i<4; i++) {
    Asum += A[i];
    if (A[i] > maxV)  {
      maxV = A[i];
      maxA = i;
    }
  }


  if(maxV<minLightT2){
    //rotate if no light
    rotateRight(200, 100);
  }else if (maxV<maxLightT2){
    if (maxA==0) {  // the light sensor at A0 is maximum
      moveForward(200,100);
    }else if (maxA==1) {  // the light sensor at A1 is maximum
      rotateRight(150, 100);
    }else if (maxA==2) {  // the light sensor at A2 is maximum
      rotateLeft(150, 100);
    }
  }else {
    // stop
    stopMove(50);
  }
}

// this is a light and compass control function.
// student should fill in some codes.
void LightandCompassControl() {
  //heading
  float heading=getHeading();
  // A[3] represent 3 sensors used.
  int A[3] = {0, 0, 0};
  int maxA = 0, maxV = 0, Asum = 0;
  // first update sensors value.
  A[0] = analogRead(A0);
  A[1] = analogRead(A1);
  A[2] = analogRead(A2);
//  A[3] = analogRead(A3);
  // and find the highest value.
  maxV = A[0]; Asum =0;
  for (int i=0; i<3; i++) {
    Asum += A[i];
    if (A[i] > maxV)  {
      maxV = A[i];
      maxA = i;
    }
  }

  //printDebugInfo();


  float tang=0.0;
    switch (lcState) {
    case 0:

    //Serial.print("lcState:");Serial.println(lcState);
    //Serial.print("angle:");Serial.println(angle);
      //pointTo(angle);
      relativePointTo(0.0);
      for (int i=0; i<2; ++i) {
        //pointTo(angle);

        moveForward(250,500 );
        relativePointTo(0.0);
      }
      //relativePointTo(0.0);

      //TODO: Remove debug code
    //  moveForward(t3ForwardSPD,t3ForwardTime);
      //blinkLED();
      lcState = 1;
      break;

    case 1:

    //Serial.print("lcState:");Serial.println(lcState);
    //Serial.print("angle:");Serial.println(angle);
    if(maxV<minLight){
      //rotate if no light
      //rotateRight(200, 100);
      //moveForward(250,100);
      if(getRelativeHeading()<30.0&&getRelativeHeading()>330.0){
        moveForward(200,100);
      }else{
        relativePointTo(0.0);
        //pointTo(angle);
        moveForward(200,100);
      }

    }else if (maxV<maxLight&&maxV>edgeLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(200,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else if (maxV<maxLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(250,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else {
      // stop
      stopMove(50);
      blinkLED();
      lcState = 2;

    }

      break;

    case 2:

    angle+=150.0;
    if(angle>360){
      angle-=360;
    }

    //Serial.print("lcState:");Serial.println(lcState);
    //Serial.print("angle:");Serial.println(angle);

    //pointTo(angle);
    relativePointTo(150.0);
    for (int i=0; i<10; ++i) {
      //pointTo(angle);

      moveForward(200, 100);
      relativePointTo(150.0);
    }
    //relativePointTo(150.0);
    //TODO: Remove debug code
  /*  rotateRight(t3TurnSPD, t3TurnTime1);
    moveForward(t3ForwardSPD,t3ForwardTime2);
    */
    //blinkLED();
    lcState = 3;
    break;

    case 3:
    if(maxV<minLight){

      if(getRelativeHeading()<120.0+45.0&&getRelativeHeading()>120.0-45.0){
        moveForward(200,100);
      }else{
        //pointTo(angle);
        relativePointTo(150.0);
        moveForward(200,100);
      }
      moveForward(200,100);

    }else if (maxV<maxLight&&maxV>edgeLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(200,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else if (maxV<maxLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(250,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else {
      // stop
      stopMove(50);
      blinkLED();
      lcState = 4;

    }

      break;
    case 4:
    angle+=120.0;
    if(angle>360){
      angle-=360;
    }

    //Serial.print("lcState:");Serial.println(lcState);
    //Serial.print("angle:");Serial.println(angle);

    //pointTo(angle);
    relativePointTo(270.0);
    for (int i=0; i<10; ++i) {
      //pointTo(angle);

      moveForward(200, 100);
      relativePointTo(270.0);
    }
    //relativePointTo(270.0);
    //TODO: Remove debug code
  /*  rotateRight(t3TurnSPD, t3TurnTime2);
    moveForward(t3ForwardSPD,t3ForwardTime2);
    */
    //blinkLED();
    lcState = 5;
    break;

    case 5:
    if(maxV<minLight){
      //rotate if no light
      //rotateRight(200, 100);
      if(getRelativeHeading()<270.0+45.0&&getRelativeHeading()>270.0-45.0){
        moveForward(200,100);
      }else{
        //pointTo(angle);
        relativePointTo(270.0);
        moveForward(200,100);
      }
      moveForward(200,100);

    }else if (maxV<maxLight&&maxV>edgeLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(200,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else if (maxV<maxLight){
      if (maxA==0) {  // the light sensor at A0 is maximum
        moveForward(250,100);
      }else if (maxA==1) {  // the light sensor at A1 is maximum
        rotateRight(150, 100);
      }else if (maxA==2) {  // the light sensor at A2 is maximum
        rotateLeft(150, 100);
      }
    }else {
      // stop
      stopMove(50);
      blinkLED();
      lcState = -1;

    }

      break;

    default:

    stopMove(50);
    blinkLED();
    //blinkLED();
    //blinkLED();
    sCompassLightSen = false;
      break;
  }
}

// this function is to set/change LED value to the input argument
// Student can do anything with LED inside this function.
void setLED(byte val) {
  pLEDpattern = val;
}

// this function is to update LED
// Student should not modify this function
void updateLED() {
      digitalWrite(SELECT,LOW);
      SPI.transfer(pLEDpattern);
      digitalWrite(SELECT,HIGH);
}

void blinkLED() {
     int j;

     for(j=0;j<3;j++) {
       setLED(0b11111111);
       updateLED();
       delay(500);
       setLED(0b00000000);
       updateLED();
       delay(500);
    }
}


// this function is to update Motor
// Student should not modify this function
void updateMotor() {
  if(sMotorStop) {  // the Motor Stop function got the highest priority,
                    // it will override StatMachine and Light Control motor result.
    M1.set(0);
    M2.set(0);
    M3.set(0);
  }
  M1.update();
  M2.update();
  M3.update();
}

// Functions for Digital Compass
float getHeading(){
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = dc.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  return heading * RAD_TO_DEG; //radians to degrees
}

void stopMove(int t) {
  M1.set(0); M2.set(0); M3.set(0);
  M1.update(); M2.update(); M3.update();
  delay(t);
}
void rotateRight(int spd, int t) {
  M1.set(spd); M2.set(spd); M3.set(spd);
  M1.update(); M2.update(); M3.update();
  delay(t);
  //stopMove(50);
}
void rotateLeft(int spd, int t) {
  M1.set(-1*spd); M2.set(-1*spd); M3.set(-1*spd);
  M1.update(); M2.update(); M3.update();
  delay(t);
  //stopMove(50);
}
void moveForward(int spd, int t) {
  M1.set(-1*spd); M2.set(0); M3.set(spd);
  M1.update(); M2.update(); M3.update();
  delay(t);
  //stopMove(50);
}



float delta = 10.0;
void pointTo(float dir) {
  float heading = 0.0;
  float angle_diff = 0.0;

  if (dir<0.0 ){
    dir+=360.0;
  } else if(dir>360.0) {
    dir-=360.0;
  }

  do {
    heading = getHeading();
    angle_diff = heading-dir;
    if (angle_diff<0.0) angle_diff+=360.0;

    /*if (angle_diff>=delta && angle_diff<=180.0) {
      rotateLeft(150, 50);
    }
    else if (angle_diff>180.0 && angle_diff<=(360.0-delta)) {
      rotateRight(150, 50);
    }*/
    if (angle_diff>=delta && angle_diff<=(360.0-delta)) {
      rotateRight(150, 50);
    }
    else if (angle_diff<delta || angle_diff>(360.0-delta)) {
      stopMove(50);
      break;
    }
  }while(1);
}

void relativePointTo(float relative_dir){
  float rHeading = 0.0;
  float angle_diff = 0.0;

  if (relative_dir<0.0 ){
    relative_dir+=360.0;
  } else if(relative_dir>360.0) {
    relative_dir-=360.0;
  }

  do {
    rHeading = getRelativeHeading();
    angle_diff = rHeading-relative_dir;
    if (angle_diff<0.0) angle_diff+=360.0;

    if (angle_diff>=delta && angle_diff<=180.0) {
      rotateLeft(150, 50);
    }
    else if (angle_diff>180.0 && angle_diff<=(360.0-delta)) {
      rotateRight(150, 50);
    }
    /*if (angle_diff>=delta && angle_diff<=(360.0-delta)) {
      rotateRight(150, 50);
    }*/
    else if (angle_diff<delta || angle_diff>(360.0-delta)) {
      stopMove(50);
      break;
    }
  }while(1);
}

// this function is to slowdown the main loop by delay.
// Student should not modify this function
void slowDown() {
  delay(20);
}

float getRelativeHeading(){
  float heading=getHeading();
  float relativeHeading=heading-initial_angle;
  if(relativeHeading<0){
    relativeHeading+=360.0;
  }
  return relativeHeading;
}

float toAbsHeading(float angle){
  float absAngle=angle+initial_angle;
  if(absAngle>360){
    absAngle-=360;
  }
  return absAngle;
}

float toRelativeHeading(float angle){
  float relativeAngle=angle-initial_angle;
  if(relativeAngle<0){
    relativeAngle+=360;
  }
  return relativeAngle;
}

void printDebugInfo(){
 float heading= getHeading();
 float rHeading=getRelativeHeading();
  //// A[4] represent 4 sensors used.
  int A[4] = {0, 0, 0, 0};
  int maxA = 0, maxV = 0, Asum = 0;
  // first update sensors value.
  A[0] = analogRead(A0);
  A[1] = analogRead(A1);
  A[2] = analogRead(A2);
  //A[3] = analogRead(A3);
  // and find the highest value.
  maxV = A[0]; Asum =0;
  for (int i=0; i<4; i++) {
    Asum += A[i];
    if (A[i] > maxV)  {
      maxV = A[i];
      maxA = i;
    }
  }
  Serial.println("//Light sensor debug value:");
  Serial.print("A0: ");Serial.println(A[0]);
  Serial.print("A1: ");Serial.println(A[1]);
  Serial.print("A2: ");Serial.println(A[2]);
  Serial.print("maxV: ");Serial.println(maxV);
  Serial.print("maxA: ");Serial.println(maxA);
  Serial.print("Heading:");Serial.println(heading);
  Serial.print("Relative Heading:");Serial.println(rHeading);


}
