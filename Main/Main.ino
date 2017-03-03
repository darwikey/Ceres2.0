#include "MotorManager.h"
#include "ControlSystem.h"
#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "CommandLineInterface.h"
#include "Platform.h"


#define SPEED 125



void setup() {
	// put your setup code here, to run once:
	delay(500);

	Platform::Init();
	Platform::DisplayNumber(0x1F);

	InitMotors();

	PositionManager::Instance.Init(41826, 150);
	ControlSystem::Instance.Start();
	TrajectoryManager::Instance.Init();

	delay(500);
}

void loop() {
	// put your main code here, to run repeatedly:
	static int time = 0;
	//static int step = 0;

	int speed = 0;
	if (Platform::IsButtonPressed(0))
		speed = SPEED;

	Platform::DisplayNumber((speed > 0) | ((time % 1024 > 512) ? 2 : 0));

	CommandLineInterface::Instance.Task();

	//updateMotors(speed);
	//updateAngleSpeed(0, speed);
	ControlSystem::Instance.Task();
	TrajectoryManager::Instance.Task();

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

	  updateEnc();
	  dist = curEnc[0] + curEnc[1];

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

