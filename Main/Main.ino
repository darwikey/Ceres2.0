#include <SoftwareSerial.h>
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
	static int clock = 0;
	//static int step = 0;

	int speed = 0;
	if (Platform::IsButtonPressed(0))
		speed = SPEED;

	// Debug
	if (clock >= 1000)
	{
		static int led = 0;
		clock = 0;
		if (led == 0)
			Platform::SetServoLED(ServoID::SERVO1, ServoLED::RED);
		else if (led == 1)
			Platform::SetServoLED(ServoID::SERVO2, ServoLED::GREEN);
		else if (led == 2)
			Platform::SetServoLED(ServoID::SERVO3, ServoLED::BLUE);
		else
		{
			Platform::SetServoLED(ServoID::ALL, ServoLED::NONE);
			led = -1;
		}
		led++;
	}
	Platform::DisplayNumber((speed > 0) | ((time & 0x7FF) > 512 ? 2 : 0));

	CommandLineInterface::Instance.Task();

	//updateMotors(speed);
	//updateAngleSpeed(0, speed);
	ControlSystem::Instance.Task();
	TrajectoryManager::Instance.Task();

	
	delay(10);
	time += 10;
	clock += 10;
}

