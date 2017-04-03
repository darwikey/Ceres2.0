#include <SoftwareSerial.h>
#include "MotorManager.h"
#include "ControlSystem.h"
#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "CommandLineInterface.h"
#include "Platform.h"
#include "Strategy.h"

#define SPEED 125

void setup() {
	// put your setup code here, to run once:
	delay(500);

	Platform::Init();
	Platform::DisplayNumber(0x1F);

	InitMotors();

	PositionManager::Instance.Init(21638, 127.2);// 125.5);
	ControlSystem::Instance.Start();
	TrajectoryManager::Instance.Init();
	Strategy::Instance.Init();

	delay(500);
	Platform::DisplayNumber(0);
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
	Side side = Strategy::Instance.GetSide();
	Platform::SetLed(0, (time & 0x7FF) > 512);
	Platform::SetLed(3, side == Side::BLUE);
	Platform::SetLed(4, side == Side::YELLOW);

	CommandLineInterface::Instance.Task();

	//updateMotors(speed);
	//updateAngleSpeed(0, speed);
	ControlSystem::Instance.Task();
	TrajectoryManager::Instance.Task();

	
	delay(10);
	time += 10;
	clock += 10;
}

