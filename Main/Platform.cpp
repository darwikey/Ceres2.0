#include "Platform.h"
#include "WProgram.h"
#include <SoftwareSerial.h>
#include "XL320.h"

namespace Platform
{
	// Set the SoftwareSerial RX & TX pins
	SoftwareSerial SerialUart2(9, 10); // (RX, TX)

	XL320 Servo;

	void Init()
	{
		for (unsigned i = 0; i < _countof(leds); ++i)
			pinMode(leds[i], OUTPUT);

		for (unsigned i = 0; i < _countof(buttons); ++i)
			pinMode(buttons[i], INPUT);

		pinMode(startPull, INPUT);

		for (unsigned i = 0; i < _countof(gp2s); ++i)
			pinMode(gp2s[i], INPUT);

		//init servo
		SerialUart2.begin(115200);
		Servo.begin(SerialUart2);
		// fast moving servos, so set the joint speed to max!
		Servo.setJointSpeed(254, 1023);

	}

	void DisplayNumber(int n)
	{
		for (unsigned i = 0; i <= _countof(leds); ++i)
			digitalWrite(leds[i], ((1 << i) & n) ? HIGH : LOW);
	}

	bool IsButtonPressed(int id)
	{
		return digitalRead(buttons[id]) == LOW;
	}

	bool IsGp2Occluded()
	{
		static bool lastBlocked = false;
		int gp2Sum = 0;

		for (unsigned i = 0; i < _countof(gp2s); ++i)
		{
			if (lastBlocked) {
				if (analogRead(gp2s[i]) <= GP2_UNBLOCK_AT)
					++gp2Sum;
			}
			else {
				if (analogRead(gp2s[i]) >= GP2_BLOCK_AT)
					++gp2Sum;
			}
		}

		if (lastBlocked && gp2Sum < (int)_countof(gp2s))
			return true;
		else if (lastBlocked && gp2Sum == _countof(gp2s))
			lastBlocked = false;
		else if (!lastBlocked && gp2Sum > 0) {
			lastBlocked = true;
			return true;
		}
		return false;
	}

	void SetServoLED(ServoID id, int color)
	{
		Servo.LED((int)id, color);
	}

	void SetServoPos(ServoID id, int pos)
	{
		Servo.moveJoint((int)id, pos);
	}

	void SetServoSpeed(ServoID id, int speed)
	{
		Servo.setJointSpeed((int)id, speed);
	}
}
