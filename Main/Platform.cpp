#include "Platform.h"
#include "WProgram.h"
#include <SoftwareSerial.h>

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
		//SerialUart2.begin(9600);
		SerialUart2.begin(115200);
		//SerialUart2.begin(1000000);
		Servo.Begin(SerialUart2);

		//configure Serial2 for servo
		UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
		CORE_PIN10_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
		
	}

	void DisplayNumber(int n)
	{
		for (unsigned i = 0; i < _countof(leds); ++i)
			digitalWrite(leds[i], ((1 << i) & n) ? HIGH : LOW);
	}

	void SetLed(int ledId, bool state)
	{
		if (ledId < (int)_countof(leds))
		{
			digitalWrite(leds[ledId], state ? HIGH : LOW);
		}
	}

	bool IsStartPulled()
	{
		return digitalRead(startPull) == HIGH;
	}
	
	bool IsButtonPressed(int id)
	{
		return digitalRead(buttons[id]) == LOW;
	}

	bool IsGP2Occluded()
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

	void DebugGP2()
	{
		do
		{
			for (unsigned i = 0; i < _countof(gp2s); ++i)
			{
				Serial.printf("Gp2 %d : %d\r\n", i, analogRead(gp2s[i]));
			}
			Serial.printf("occluded %d\r\n\r\n", (int)IsGP2Occluded());
			delay(500);
		} while (Serial.available() == 0);
	}

	void SetServoLED(ServoID id, ServoLED color)
	{
		Servo.Write((int)id, XL320::Address::LED, (int)color);
	}

	void SetServoPos(ServoID id, int pos)
	{
		Servo.Write((int)id, XL320::Address::GOAL_POSITION, pos);
	}

	void SetServoSpeed(ServoID id, int speed)
	{
		Servo.Write((int)id, XL320::Address::GOAL_SPEED, speed);
	}

	void ForceServoBaudRate()
	{
		//SerialUart2.begin(1000000);
		// 0: 9600, 1:57600, 2:115200, 3:1Mbps
		Servo.Write((int)ServoID::ALL, XL320::Address::BAUD_RATE, 2);
		//SerialUart2.begin(115200);
	}

	void ForceServoId(int id)
	{
		Servo.Write((int)ServoID::ALL, XL320::Address::ID, id);
	}

	void SendServoPacket(int id, XL320::Address address, int value)
	{
		Servo.Write(id, address, value);
	}

	void ReadServoPacket(int id, int address)
	{
		Serial.printf("value: %d\r\n", Servo.GetValue(id, (XL320::Address)address));
	}

	void DebugServoRam(int id)
	{
		Servo.DebugAllValue(id, Serial);
	}

}
