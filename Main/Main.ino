#include "MotorManager.h"
#include "control_system.h"
#include "position_manager.h"
#include "trajectory_manager.h"
#include "cli.h"

#define NB_LEDS 5
const int leds[] = { 11, 12, 13, 20, 21 };

#define NB_BUTTONS 4
const int buttons[] = { 5, 6, 7 , 8 };

const int startPull = 2;

#define NB_GP2S 4
const int gp2s[] = { 0, 1, 2, 3 };
#define GP2_BLOCK_AT   1000 /*432*/ /*288*/
#define GP2_UNBLOCK_AT 1000 /*324*/ /*216*/

#define SPEED 125

void display(int n) {
  for(int i = 0; i <= NB_LEDS; ++i)
    digitalWrite(leds[i], ((1<<i) & n) ? HIGH : LOW);
}



void setup() {
  // put your setup code here, to run once:
  delay(500);

  for(int i = 0; i < NB_LEDS; ++i)
    pinMode(leds[i], OUTPUT);

  display(0x1F);

  for(int i = 0; i < NB_BUTTONS; ++i)
    pinMode(buttons[i], INPUT);

  pinMode(startPull, INPUT);

  for(int i = 0; i < NB_GP2S; ++i)
    pinMode(gp2s[i], INPUT);

  InitMotors();

  position_init(41826, 150);
  control_system_start();
  trajectory_init();

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int time = 0;
  //static int step = 0;

    int speed = 0;
    if (digitalRead(buttons[0]) == LOW)
      speed = SPEED;

    display((speed>0) | ((time % 1024 > 512) ? 2 : 0));
    
	cli_task();

    //updateMotors(speed);
    //updateAngleSpeed(0, speed);
    control_system_task();
	trajectory_task();

  //Serial.println(position_get_x_mm());
  //Serial.println(position_get_y_mm());
   //Serial.println("\n");

   // if(dist >= 1024*34)
     // step = 3;

/*  display(step);

  if(time >= 85000) {
    display(0x1F);
    while(1) {
      analogWrite(motorPWMs[0], 0);
      analogWrite(motorPWMs[1], 0);
    }
  }

  if(step == 0) {
    while(digitalRead(buttons[0]) == HIGH);
    step = 1;
  }
  else if(step == 1) {
    //while(digitalRead(buttons[1]) == HIGH);
    while(digitalRead(startPull) == LOW);
    time = 0;
    step = 2;
    encoder1.start();
    encoder2.start();
  }
  else if(step == 2) {
    int speed = SPEED;
    long long dist;
    static bool lastBlocked = false;
    int gp2Sum = 0;
    
    updateEnc();
    dist = curEnc[0] + curEnc[1];

    for(int i = 0; i < 2 && speed != 0; ++i)
      if(lastBlocked) {
        if(analogRead(gp2s[i]) <= GP2_UNBLOCK_AT)
          ++gp2Sum;
      }
      else {
        if(analogRead(gp2s[i]) >= GP2_BLOCK_AT)
          ++gp2Sum;
      }

    if(lastBlocked && gp2Sum < 2)
      speed = 0;
    else if(lastBlocked && gp2Sum == 2)
      lastBlocked = false;
    else if(!lastBlocked && gp2Sum > 0) {
      lastBlocked = true;
      speed = 0;
    }

    display(step | ((lastBlocked) ? 0b11000 : 0));
    
    //updateMotors(speed);
    updateAngleSpeed(0, speed);

    if(dist >= 1024*34)
      step = 3;
  }
  else if(step == 3) {
    updateEnc();
    updateMotors(0);
    if(time >= 80000)
      step = 4;
  }
  else {
    updateEnc();
    updateMotors(0);
  }
  */
  delay(10);
  time += 10;
}

