//Copyright (C) 2017  Jordi Guerrero Zapata

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

//radians = (degrees * 71) / 4068
//degrees = (radians * 4068) / 71

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "DRV8835_teensy3.h"
#include <Encoder.h>

#define test_mode_button   21 //black
#define next_test_button   22 //white

#define sonar_servo_pin     29

#define RED_LED             13
#define GREEN_LED           23
#define buzzer_pin          22


#define s_init                  0
#define s_listening             1
#define s_init_go               2
#define s_rotating              3
#define s_forwarding            4
#define s_waiting_at_obstacle   5
#define s_sonarstep             6
#define s_waiting_sonarstep     7

#define c_none    -1
#define c_go       1
#define c_sonar    2
#define c_resume   3
#define c_halt     4

// Variable declaration

const float Pi = 3.14159;
int TicksxRevolution = 1440;
float WheelRadius = 40.20/2;
float WheelDistance = 100.0;

float Kp = 150; //150.0;
float Ki = 1; //0.1;
float Kd = 0.001; //0.001;

long startTicksLeft;
long startTicksRight;

long endTicksLeft;
long endTicksRight;

long incTicksLeft;
long incTicksRight;

int t_angle;
float rad_angle;
float heading;
float f_heading; //angulo iniciado a cero para ir recto.
float distanceLeft;
float distanceRight;
float newdistance;
float r_distance; // distancia recorrida por el robot.
float x;
float y;

// v8
float total_heading;
float total_x;
float total_y;
float delta_heading;
float delta_x;
float delta_y;

double deltax;
double deltay;
double referenceHeading;
double headingError;
double u;
double headingErrorDerivative;
double lastHeadingError;
float dt;
long start_dt;
long end_dt;

String inMessage;
int command;
int angle;
unsigned int distance;
int state;
unsigned long obstacle; //cm
unsigned long safe_distance, safe_distance_plus; // cm
unsigned int timeout = 10000; // ms


int pos_sonar = 90;
int dipPins[] = {A1, A2, A3, A4}; //DIP Switch Pins

// int angleArray[] = {105, 120, 135, 150, 165, 180, 130, 90, 75, 60, 45, 30, 15, 0, 50, -1}; //end of array = -1
int angleArray[] = {15, 30, 45, 60, 75, 90, 40, 0, -15, -30, -45, -60, -75, -90, -40, -1}; // end of array = -1
int index_angleArray, coef;

unsigned int goR;
unsigned int goL;
unsigned int rightTurnR;
unsigned int rightTurnL;
unsigned int leftTurnR;
unsigned int leftTurnL;
unsigned int stopR;
unsigned int stopL;
float leftCal_angle;
float rightCal_angle;
float cal_distance;
float cal_distance10;
float cal_distance20;
float cal_distance30;
float cal_distance40;


bool calibration_mode;
unsigned long iter_angle;
char OKsonar[] = "0006";

void whoamI(bool showInfo);
void splitString (String inStr);
void readSerial();
void readSonar();
float computeCal_distance(unsigned int cm);
unsigned long compute_partial_distance(unsigned long elapsed_time);

Servo sonar_servo;
VL53L0X lidar;
//DRV8835 driveMotors(32, 30, 31, 29);
DRV8835 driveMotors(33, 35, 34, 36);
Encoder lwEnc(15, 14);
//Encoder rwEnc(36, 35);
Encoder rwEnc(38, 37);

void setup() {
  sonar_servo.attach(sonar_servo_pin);
  
  Serial1.begin(9600);
  Wire.begin();

  lidar.init();
  lidar.setTimeout(500);

  heading = 0.0;
  t_angle = 0;
  lastHeadingError = 0.0;
  
  lidar.startContinuous();

  driveMotors.init(23437);

  pinMode(test_mode_button, INPUT);
  pinMode(next_test_button, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(buzzer_pin, OUTPUT);

//  for(i = 0; i<=3; i++){
//    pinMode(dipPins[i], INPUT_PULLUP);      // sets the digital pin 2-5 as input
//  }
  

  whoamI(false);

  safe_distance = 12; //cm
  safe_distance_plus = safe_distance + 1;
  state = s_init;

  //v8
  total_heading = 0.0;
  total_x = 0.0;
  total_y = 0.0;  

} //end void setup

void loop() {

  // static unsigned long looptimer;
  static unsigned long timer_wao; //, timer_f, t_distance, t_distance_orig;//timer_r, , t_angle;
  static unsigned int partial_distance;


  readSerial();
  if (command == c_halt) {
    state = s_init;
    command = c_none;
  }

  switch (state) {

    case s_init:
        

        driveMotors.setSpeeds(0,0);
        Serial1.println("0000 : Initializing program :");
        if (command != c_none) {
          command = c_none;
          Serial1.println("1100 : Still initializing program");
        }

        
        index_angleArray = 0;
        pos_sonar = 90;
        sonar_servo.write(pos_sonar);
        delay(15);
        state = s_listening;
        Serial1.print("0001 : Listening");
        delay(250);
        break;

    case s_listening: 
        if (command == c_go) {
          Serial1.println("0002 : Initializing go");
          command = c_none;
          state = s_init_go;
        } else if (command == c_sonar) {
          driveMotors.setSpeeds(0,0);
          //right_servo.writeMicroseconds(stopR);
          //left_servo.writeMicroseconds(stopL);

          Serial1.println("0005 : Initializing sonar");
          
          command = c_none;
          // n_rotations = 0; //antiguo codigo
          pos_sonar = 90;
          sonar_servo.write(pos_sonar);
          delay(15);
          state = s_sonarstep;
        }

        break; //end of case listening

    case s_init_go: 

        if (angle == 0) {
          if ( distance == 0 ) {
            state = s_listening;
            Serial1.println("1103: destination reached");
          }
          else {
            state = s_forwarding;
            sonar_servo.write(90);
            delay(15);
            startTicksLeft = lwEnc.read();
            startTicksRight = rwEnc.read();
            r_distance = 0.0;
            f_heading = 0.0;
            lastHeadingError = 0.0;
            u = 0;
            x = 0.0;
            y = 0.0;


            
            start_dt = millis();
            
            // Recordar que en este punto los servos estan a stopR y stopL respectivamente           
            driveMotors.setSpeeds(90,90);                      
            
          }//end if else distance == 0
        }
        else {
          state = s_rotating;
          t_angle = t_angle + angle;
          rad_angle = ((float)t_angle*71.0) / 4068.0;
          startTicksLeft = lwEnc.read();
          startTicksRight = rwEnc.read();
          
          if (angle > 0) {  
            driveMotors.setSpeeds(90,-90);          
          }
          else {
            driveMotors.setSpeeds(-90,90);           
          }
        }//end if else angle == 0
        break;  //end of case init_go

    case s_rotating: 
    
        endTicksLeft = lwEnc.read();
        endTicksRight = rwEnc.read();
        
        incTicksLeft = endTicksLeft - startTicksLeft;
        incTicksRight = endTicksRight - startTicksRight;

        distanceLeft = 2.0 * Pi * WheelRadius * ((float)incTicksLeft / (float)TicksxRevolution);
        distanceRight = 2.0 * Pi * WheelRadius * ((float)incTicksRight / (float)TicksxRevolution);

        newdistance = (distanceLeft + distanceRight) / 2.0;

        delta_heading = ((2.0 * Pi * WheelRadius / (float)TicksxRevolution)*((float)incTicksLeft - (float)incTicksRight)) / WheelDistance;    

        
        heading += delta_heading;
        total_heading += delta_heading;

        total_x += newdistance * cos(total_heading);
        total_y += newdistance * sin(total_heading); 

        Serial1.print("heading: ");
        Serial1.println(total_heading);
        Serial1.print("x: ");
        Serial1.println(total_x);
        Serial1.print("y: ");
        Serial1.println(total_y);

    
        startTicksLeft = endTicksLeft;
        startTicksRight = endTicksRight;

        if (angle > 0 && (rad_angle - heading) / 0.0174 < 30 ){
          driveMotors.setSpeeds(40,-40); 
        }
        if (angle < 0 && (heading - rad_angle) / 0.0174 < 30 ){
          driveMotors.setSpeeds(-40,40);
        }
        
        if ( (angle > 0 && heading > rad_angle) || (angle < 0 && heading < rad_angle) ){
          driveMotors.setSpeeds(0,0);
          Serial1.println("1003: rotation complete");
          delay(1000);
          if (distance == 0) {
            state = s_listening;
            Serial1.println("1103: destination reached");
            
          }
          else {
            state = s_forwarding;
            sonar_servo.write(90);
            delay(15);
            startTicksLeft = lwEnc.read();
            startTicksRight = rwEnc.read();
            lastHeadingError = 0.0;
            r_distance = 0.0;
            f_heading = 0.0;
            x = 0.0;
            y = 0.0;
            u = 0;
            start_dt = millis();
            
            // Recordar que en este punto los servos estan a stopR y stopL respectivamente           
            //driveMotors.setSpeeds(90,90);
          }
        } 
        break;    //end of case rotating

    case s_forwarding:
        obstacle = lidar.readRangeContinuousMillimeters()/10; 
          if (obstacle <= safe_distance){
            driveMotors.setSpeeds(0,0);
            state = s_waiting_at_obstacle;
            timer_wao = millis(); //timer waiting at obstacle
            
          } else {
            
          endTicksLeft = lwEnc.read();
          endTicksRight = rwEnc.read();
          
          incTicksLeft = endTicksLeft - startTicksLeft;
          incTicksRight = endTicksRight - startTicksRight;
        
          distanceLeft = 2.0 * Pi * WheelRadius * ((float)incTicksLeft / (float)TicksxRevolution);
          distanceRight = 2.0 * Pi * WheelRadius * ((float)incTicksRight / (float)TicksxRevolution);
  
          startTicksLeft = endTicksLeft;
          startTicksRight = endTicksRight;
  
          newdistance = (distanceLeft + distanceRight) / 2.0;
          r_distance += newdistance;
  
          delta_heading = (distanceLeft - distanceRight) / WheelDistance;
          f_heading += delta_heading;

          total_heading += delta_heading;

          total_x += newdistance * cos(total_heading);
          total_y += newdistance * sin(total_heading);           

          Serial1.print("heading: ");
          Serial1.println(total_heading);
          Serial1.print("x: ");
          Serial1.println(total_x);
          Serial1.print("y: ");
          Serial1.println(total_y);
       
          if (r_distance > distance - 5.0) {
            driveMotors.setSpeeds(0,0);
            delay(1000);
            t_angle -= (f_heading * 4068) / 71;
            Serial1.println("1103: destination reached");
            state = s_listening;
          } else{
  
            x += newdistance * cos(f_heading);
            y += newdistance * sin(f_heading); 
            deltax = distance - x;
            deltay = 0 - y;
            //referenceHeading;
          
            if ((deltay >= 0) and (deltax >= 0))
              referenceHeading = acos(deltax / sqrt((deltay * deltay) + (deltax * deltax)));
            if ((deltay < 0) and (deltax < 0))
              referenceHeading = (acos(deltax / sqrt((deltay * deltay) + (deltax * deltax))) * -1);
            if ((deltay >= 0) and (deltax < 0))
              referenceHeading = acos(deltax / sqrt((deltay * deltay) + (deltax * deltax)));
            if ((deltay < 0) and (deltax >= 0))
              referenceHeading = asin(deltay / sqrt((deltay * deltay) + (deltax * deltax)));
            
            headingError = referenceHeading - (double)f_heading;
            
            end_dt = millis();
           
            dt = end_dt - start_dt;
            //Serial1.print("dt");
            //Serial1.println(dt);
            start_dt = end_dt;
            
            headingErrorDerivative = (headingError - lastHeadingError) / (double)dt;
          
            lastHeadingError = headingError;
            //Serial1.print(x);
            //Serial1.print(" ");
            //Serial1.println(y);
            double u = (Kp * headingError) + (Kd * headingErrorDerivative);
            //u = 0;      
            driveMotors.setSpeeds(80 + (float)u, 80 - (float)u);      
       
          }
        
        }
     
        break; //end of case forwarding

    case s_waiting_at_obstacle:
          obstacle = lidar.readRangeContinuousMillimeters()/10;
          if(obstacle <= safe_distance_plus){
             if(millis() - timer_wao >= timeout){
               state = s_init;
               Serial1.print("1105 ");
               Serial1.print(partial_distance);
               Serial1.println(" Timeout exceeded... Reinitialising.");
               t_angle -= (f_heading * 4068) / 71;
             }
          }else{
             state = s_forwarding;
              // Recordar que en este punto los servos estan a stopR y stopL respectivamente           
          }
 
        break;  //end of case waiting_at_obstacle

    case s_sonarstep:

        pos_sonar = angleArray[index_angleArray];
        sonar_servo.write(pos_sonar);
        delay(1000);
          
        obstacle = lidar.readRangeContinuousMillimeters()/10;
        
        Serial1.print(OKsonar);
        Serial1.print(" ");          
        Serial1.print(coef == 1 ? (pos_sonar-90) : (90 - pos_sonar));
        Serial1.print(" ");
        Serial1.println(obstacle);
          
        index_angleArray++;
        state = s_waiting_sonarstep;
          
        break;  //end of case sonarstep

    case s_waiting_sonarstep: 

        if (command == c_resume) {
          command = c_none;
          if(angleArray[index_angleArray] != -1){
            state = s_sonarstep;
            OKsonar[3] = '7';
          } else {
            Serial1.println("0008");
            sonar_servo.write(90);
            delay(100);
            index_angleArray = 0;
            OKsonar[3] = '6';
            state = s_listening;
          }
        }
        break;

  }// end of switch

  delay(20);

}// end of loop


void readSerial() {
  if (Serial1.available() > 0) {
    inMessage = Serial1.readStringUntil('\n');

    command = inMessage.toInt();
    //Serial1.println(command);

    if (command == c_go) {
      splitString(inMessage);
    }

    if ( command == c_resume ) {
      if  (state != s_waiting_sonarstep ) {
        command = c_none;
        Serial1.print("1100");
        Serial1.print("nothing to resume");
      }
    }

  } //end of if Serial available

}//end of readSerial


void splitString (String inStr) {
  char inChar[50];
  inStr.toCharArray(inChar, 50);
  

  char separator[] = " ";
  char *token;

  token = strtok(inChar, separator);

  int i = 0;

  while (token != NULL) {

    token = strtok(NULL, separator);
    if (i == 0) {
      angle = atoi(token);//-90;
    }

    if (i == 1) {
      distance = atoi(token);
    }

    i++;
  }//end of while

  if (i <= 2) {
    command = c_none;
    if (i == 1) {
      Serial1.println("1015 GO without arguments");
    }
    else {
      Serial1.println("1014 GO with no distance");
    }
  }

} //end of splitString

void whoamI(bool showInfo) {

   int i, robot_id;
   
   //Get the switches state
   for(i=0, robot_id = 0; i<=3; i++){
     robot_id = (robot_id << 1) | digitalRead(dipPins[i]);   // read the input pin
   }


   /* ajuste de la lista de angulos */
   robot_id += 10;
   coef = ((robot_id == 11) || (robot_id == 12) || (robot_id == 18) || (robot_id == 20)) ? -1 : 1;  
   i=0;
   while(angleArray[i] != -1){
     angleArray[i] = coef * angleArray[i] + 90;
     ++i;
   }      
       
   if(showInfo){
     Serial1.print("Hello, I am SERVOBOT_");
     Serial1.println(robot_id);
   }
      
} //end of whoamI
