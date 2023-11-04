#include "ListLib.h"

//Pins For SR-04 Ultrasonic
#define T1 3
#define E1 4
#define T2 5
#define E2 6
#define T3 7
#define E3 8
#define T4 9
#define E4 10

//Pins For Motors
#define V1 11
#define V2 12
#define V3 A0
#define V4 A1
#define V5 A2

//Pin For LED
#define L1 13

//Operation Ranges-  RED_RANGE is the critical Inner Range, BLUE_RANGE is Outer Range
//Objects start to get detected if they are within BLUE_RANGE
//Operation Ranges is different for Indoors and Outdoors
#define RED_RANGE 100 // 100 1 meters

#define BLUE_RANGE 300 //300 3 meters


//Vibration Rates for Vibrator Motor. MIN_RATE & MAX_RATE Define minimul operating and Maximum Vibrating Capacity
#define MIN_RATE  100

#define MAX_RATE 300

// defines variables
long t1, t2, t3, t4; // variable for the duration of sound wave travel
int d1, d2, d3, d4; // variable for the distance measurement. Direction Reference: from Right to Left(1-->4)

//buffers[4] for distances; purpose: smoothening the values
List<int> d1Av;
List<int> d2Av;
List<int> d3Av;
List<int> d4Av;

//buffers[4] for combinational detections Algorithm
List<int> Cmb1;
List<int> Cmb2;
List<int> Cmb3;
List<int> Cmb4;

int DAvg[4];//Stores Average Distances from 4 buffers 

void setup() {
  //PinMode Initialisation for 4 X SR04 Sensors
  pinMode(T1, OUTPUT);
  pinMode(E1, INPUT);
  pinMode(T2, OUTPUT);
  pinMode(E2, INPUT);
  pinMode(T3, OUTPUT);
  pinMode(E3, INPUT);
  pinMode(T4, OUTPUT);
  pinMode(E4, INPUT);

  pinMode(L1, OUTPUT);//LED

  //PinMode Initialisation for V_Motors
  pinMode(V1, OUTPUT);
  pinMode(V2, OUTPUT);
  pinMode(V3, OUTPUT);
  pinMode(V4, OUTPUT);
  pinMode(V5, OUTPUT);
  
  //Serial For Logs
  Serial.begin(38400);
  Serial.print("\nHeadNavigation Output: "); // print some text in Serial Monitor

  //Trig Clear For SR04s
  digitalWrite(T1, LOW);
  digitalWrite(T2, LOW);
  digitalWrite(T3, LOW);
  digitalWrite(T4, LOW);

  //Initialising distance_Average(dAv)buffer Lists with four elements each for d1,d2,d3,d4
  d1Av.Add(0); d1Av.Add(0); d1Av.Add(0); d1Av.Add(0);
  d2Av.Add(0); d2Av.Add(0); d2Av.Add(0); d2Av.Add(0);
  d3Av.Add(0); d3Av.Add(0); d3Av.Add(0); d3Av.Add(0);
  d4Av.Add(0); d4Av.Add(0); d4Av.Add(0); d4Av.Add(0);

  //Initialising Combination_Buffer for consequent Sensor Same Object Detection patterns for each pairs with 4 elements
  Cmb1.Add(0); Cmb1.Add(0); Cmb1.Add(0); Cmb1.Add(0);
  Cmb2.Add(0); Cmb2.Add(0); Cmb2.Add(0); Cmb2.Add(0);
  Cmb3.Add(0); Cmb3.Add(0); Cmb3.Add(0); Cmb3.Add(0);
  Cmb4.Add(0); Cmb4.Add(0); Cmb4.Add(0); Cmb4.Add(0);


  //start sequence
  motorStartSeq();
}
void loop() {
measureDistance();// Call for all 4 distance measurement routine
//motorStartSeq();
displaySensors();// Log all 4 distances.

 if (!(d1 > BLUE_RANGE && d2 > BLUE_RANGE && d3 > BLUE_RANGE && d4 > BLUE_RANGE)) {
 processFront();//processing for sensor 2 & 3
 processRight();//Processing for Sensor 1 & 2
processLeft();//Processing for Sensor 3 & 4
   }
    
    
delay(200);
}


void displaySensors() {
  Serial.print("\nMeasurements:");
  Serial.print(d1);
  Serial.print(", ");
  Serial.print(d2);
  Serial.print(", ");
  Serial.print(d3);
  Serial.print(", ");
  Serial.print(d4);
  Serial.print("\n");
}

void measureDistance() {
  //Routine Uses SR04 Measuring mechanism
  //Routine Uses Smart Measurement Sequence. In order to avoid adjacent sensors measuring consequently, causing major errors.
  //Sequence Goes: 2-->4-->1-->3 

  digitalWrite(T2, HIGH);
  delayMicroseconds(10);
  digitalWrite(T2, LOW);
  t2 = pulseIn(E2, HIGH);
  d2 = t2 * 0.034 / 2;
  if (d2 > 400 || d2 < 2) {
    d2 = 500;
  }
  d2Av.RemoveFirst();
  d2Av.Add(d2);

  delayMicroseconds(10);

  digitalWrite(T4, HIGH);
  delayMicroseconds(10);
  digitalWrite(T4, LOW);
  t4 = pulseIn(E4, HIGH);
  d4 = t4 * 0.034 / 2;
  if (d4 > 400 || d4 < 2) {
    d4 = 500;
  }
  d4Av.RemoveFirst();
  d4Av.Add(d4);

  delayMicroseconds(10);
  
  digitalWrite(T1, HIGH);
  delayMicroseconds(10);
  digitalWrite(T1, LOW);
  t1 = pulseIn(E1, HIGH);
  d1 = t1 * 0.034 / 2;
  if (d1 > 400 || d1 < 2) {
    d1 = 500;
  }
  d1Av.RemoveFirst();
  d1Av.Add(d1);

  delayMicroseconds(10);

  digitalWrite(T3, HIGH);
  delayMicroseconds(10);
  digitalWrite(T3, LOW);
  t3 = pulseIn(E3, HIGH);
  d3 = t3 * 0.034 / 2;
  if (d3 > 400 || d3 < 2) {
    d3 = 500;
  }
  d3Av.RemoveFirst();
  d3Av.Add(d3);

  delayMicroseconds(10);
  
}

void processRight(){
  //RightProcess Right Most Sensors: Sensor1 and Sensor2. First->Checks for Same Object Blocking Both, using Combinational Algorithm. If Yes, Initiates Major Response RIGHT().
  //If Not-->Second-->Checks for Individual Object Detections, and Routines Minor Routines, i.e, right() for Sensor1 & rightFront() for Sensor2.
  
  if (d1 < BLUE_RANGE && d2 < BLUE_RANGE) {//Condition for Both Sensor detecting Objects within Bluerange. Purpose of this Block: Detect if the objects detected is same or different?

      //Averages The Buffer and Stores in Avergae Array DAvg[sensor_number]
      DAvg[0] = (d1Av[0] + d1Av[1] + d1Av[2] + d1Av[3]) / 4; //sensor1
      DAvg[1] = (d2Av[0] + d2Av[1] + d2Av[2] + d2Av[3]) / 4;//sensor2

      if (abs(d1 - DAvg[0]) < 4 && abs(d2 - DAvg[1]) < 4) {//Checks if Current distance is Near to the Buffer Average by maximum 4 cms. Purpose: Eliminates Instaneous Flyby Objects and Detects Persistent true Objects.

        //adds the DAvg Values to Combinational Buffer for Sameobject detection. Only Added if the both sensors detect obstacles, for sameObject detection.
        Cmb1.RemoveFirst();
        Cmb1.Add(DAvg[0]);
        Cmb2.RemoveFirst();
        Cmb2.Add(DAvg[1]);

        //Log
        Serial.print("\nCmb1: ");
        Serial.print(DAvg[0]);
        Serial.print(" Cmb2: ");
        Serial.print(DAvg[1]);


            if (((Cmb1[3] != Cmb1[4] && Cmb2[3] != Cmb2[4]) || (Cmb1[3] == Cmb1[4] && Cmb2[3] == Cmb2[4])) && (abs(DAvg[0] - DAvg[1])) < 5) {
                  /*Princple SameObject Detection Algorithm: By Referring to Previous Values in Combinational Buffers for sensor1& sensor2
                    check if, 1) if sensor 1 is changing then sensor2 should also change
                    Or 2) If sensor1 is not changing then Sensor2 shouldnt change
                      3) Maximum Difference between both Sensor Averages DAvg[] should be less than 5
                    if either 1 Or 2 is true along with 3,{(1||2)&&3} conditon is true, Then SAME OBJECT IS DETECTED
                   */

                 Serial.print("\nSame Object Detected at 1 &2");//Log
                 
                 RIGHT(calculateRate(DAvg[1]));//Call for Major RIGHT Function with passing Vibration Rate calculation for DAvg[] Value
              }

             else {//If Combinational Algorithm is False, then execute individual Routine
                   if (d1 < BLUE_RANGE) {//sensor1
                      right(calculateRate(DAvg[0]));//Call Minor Routine
                       //Warning for right
                       }

                     if (d2 < BLUE_RANGE) {//sensor2
                     rightFront(calculateRate(DAvg[1]));//Call Minor Routine
                     //Warning for right front
                     }
                  }
      }
      }
     else {//If Detection for Both Sensors 1&2 fails for within BLUE_RANGE, Executes individual Detection
      if (d1 < BLUE_RANGE) {
        right(calculateRate(d1));//Call Minor Routine
        //Warning for right
  }

  if (d2 < BLUE_RANGE) {
    rightFront(calculateRate(d2));//Call Minor Routine
    //Warning for right front
     }
  }
  }


void processFront(){//For Sensor 2 & 3
  if (d2 < BLUE_RANGE && d3 < BLUE_RANGE) {
    DAvg[1] = (d2Av[0] + d2Av[1] + d2Av[2] + d2Av[3]) / 4;
    DAvg[2] = (d3Av[0] + d3Av[1] + d3Av[2] + d3Av[3]) / 4;

    if (abs(d2 - DAvg[1]) < 4 && abs(d3 - DAvg[2]) < 4) {
      Cmb2.RemoveFirst();
      Cmb2.Add(DAvg[1]);
      Cmb3.RemoveFirst();
      Cmb3.Add(DAvg[2]);

      //Logs
      Serial.print("\nCmb2: ");
      Serial.print(DAvg[1]);
      Serial.print(" Cmb3: ");
      Serial.print(DAvg[2]);

      if (((Cmb2[3] != Cmb2[4] && Cmb3[3] != Cmb3[4]) || (Cmb2[3] == Cmb2[4] && Cmb3[3] == Cmb3[4])) && (abs(DAvg[1] - DAvg[2])) < 5) {
        
        Serial.print("\nSame Object Detected at 2 &3");//Log
        
        FRONT(calculateRate(DAvg[2]));//Major Routine
      }


      else {
        if (d2 < BLUE_RANGE) {
          rightFront(calculateRate(DAvg[1]));//Minor Routine
          //Warning for right front
        }

        if (d3 < BLUE_RANGE) {
          leftFront(calculateRate(DAvg[2]));//Minor Routine
          //Warning for left front
        }
      }
    }
  }
  else{
  if (d2 < BLUE_RANGE) {
    rightFront(calculateRate(d2));//Minor Routine
    //Warning for Right front
       }

  if (d3 < BLUE_RANGE) {
    leftFront(calculateRate(d3));//Minor Routine
    //Warning for left front
       }
    }
  }

void processLeft(){
  if (d3 < BLUE_RANGE && d4 < BLUE_RANGE) {//For Sensor 3&4
    
    DAvg[2] = (d3Av[0] + d3Av[1] + d3Av[2] + d3Av[3]) / 4;
    DAvg[3] = (d4Av[0] + d4Av[1] + d4Av[2] + d4Av[3]) / 4;
    
    if (abs(d3 - DAvg[2]) < 4 && abs(d4 - DAvg[3]) < 4) {
      Cmb3.RemoveFirst();
      Cmb3.Add(DAvg[2]);
      Cmb4.RemoveFirst();
      Cmb4.Add(DAvg[3]);

      //Logs
      Serial.print("\nCmb3: ");
      Serial.print(DAvg[2]);
      Serial.print(" Cmb4: ");
      Serial.print(DAvg[3]);

      if (((Cmb3[3] != Cmb3[4] && Cmb4[3] != Cmb4[4]) || (Cmb3[3] == Cmb3[4] && Cmb4[3] == Cmb4[4])) && (abs(DAvg[2] - DAvg[3])) < 5) {

        Serial.print("\nSame Object Detected at 3 &4");
        
        LEFT(calculateRate(DAvg[2]));//Major Routine 
      }


      else {
        if (d3 < BLUE_RANGE) {
          leftFront(calculateRate(DAvg[2]));
        }

        if (d4 < BLUE_RANGE) {
          left(calculateRate(DAvg[3]));
        }
      }
    }
  }

    else{
      if (d3 < BLUE_RANGE) {
         leftFront(calculateRate(d3));
         }

      if (d4 < BLUE_RANGE) {
        left(calculateRate(d4));
        }
      }
  } 

int calculateRate(int d){
  int rate;
  rate = (int)((d/(float)BLUE_RANGE) * (BLUE_RANGE-RED_RANGE)+ MIN_RATE);
  //ledBlink(rate); Routine Discontinued for extra Delays
  return rate;
  }

void  motorStartSeq() {//Motor_Start_Sequence for Startup Routine : 1-->2-->3-->4-->5
  int x = 300;//Vibration Rate
  digitalWrite(V1, LOW);
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  digitalWrite(V5, LOW);
  delay(100);
  digitalWrite(V1, HIGH);
  delay(x);
  digitalWrite(V1, LOW);
  digitalWrite(V2, HIGH);
  delay(x);
  digitalWrite(V2, LOW);
  digitalWrite(V3, HIGH);
  delay(x);
  digitalWrite(V3, LOW);
  digitalWrite(V4, HIGH);
  delay(x);
  digitalWrite(V4, LOW);
  digitalWrite(V5, HIGH);
  delay(x);
  digitalWrite(V5, LOW);
}

void RIGHT(int r){
  digitalWrite(V1, LOW);
  digitalWrite(V2, LOW);
  delay(1);
  digitalWrite(V1, HIGH);
  digitalWrite(V2, HIGH);
  delay(r);
  digitalWrite(V1, LOW);
  digitalWrite(V2, LOW);
  delay(r);
  digitalWrite(V1, HIGH);
  digitalWrite(V2, HIGH);
  delay(r);
  digitalWrite(V1, LOW);
  digitalWrite(V2, LOW);
  delay(r);
  }
  
  void FRONT(int r){
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  delay(1);
  digitalWrite(V2, HIGH);
  digitalWrite(V3, HIGH);
  digitalWrite(V4, HIGH);
  delay(r);
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  delay(r);
 digitalWrite(V2, HIGH);
  digitalWrite(V3, HIGH);
  digitalWrite(V4, HIGH);
  delay(r);
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  delay(r);
  }

  void LEFT(int r){
  digitalWrite(V4, LOW);
  digitalWrite(V5, LOW);
  delay(1);
  digitalWrite(V4, HIGH);
  digitalWrite(V5, HIGH);
  delay(r);
  digitalWrite(V4, LOW);
  digitalWrite(V5, LOW);
  delay(r);
  digitalWrite(V4, HIGH);
  digitalWrite(V5, HIGH);
  delay(r);
  digitalWrite(V4, LOW);
  digitalWrite(V5, LOW);
  delay(r);
  }

  void right(int r){
  digitalWrite(V1, LOW);
  delay(1);
  digitalWrite(V1, HIGH);
  delay(r);
  digitalWrite(V1, LOW);
  delay(r);
  }

  void rightFront(int r){
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  delay(1);
  digitalWrite(V2, HIGH);
  digitalWrite(V3, HIGH);
  delay(r);
  digitalWrite(V2, LOW);
  digitalWrite(V3, LOW);
  delay(r);
  }

   void front(int r){
  digitalWrite(V3, LOW);
  delay(1);
  digitalWrite(V3, HIGH);
  delay(r);
  digitalWrite(V3, LOW);
  delay(r);
  }

  void left(int r){
  digitalWrite(V5, LOW);
  delay(1);
  digitalWrite(V5, HIGH);
  delay(r);
  digitalWrite(V5, LOW);
  delay(r);
  }

  void leftFront(int r){
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  delay(1);
  digitalWrite(V3, HIGH);
  digitalWrite(V4, HIGH);
  delay(r);
  digitalWrite(V3, LOW);
  digitalWrite(V4, LOW);
  delay(r);
  }

 void ledBlink(int r){
    digitalWrite(13, LOW);
    delay(1);
    digitalWrite(13,HIGH);
    delay(r);
    digitalWrite(13,LOW);
    delay(r);
    digitalWrite(13,HIGH);
    delay(r);
    digitalWrite(13,LOW);
    delay(r);
    digitalWrite(13,HIGH);
    delay(r);
    digitalWrite(13,LOW);
    delay(r);
    }
