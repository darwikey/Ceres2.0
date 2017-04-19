#ifndef PLATFORM_H
#define PLATFORM_H

#define _countof(a) (sizeof(a)/sizeof(*(a)))

#define GP2_BLOCK_AT   432 /*288*/
#define GP2_UNBLOCK_AT 324 /*216*/

enum class ServoID
{
	SERVO1 = 1,
	SERVO2 = 2,
	SERVO3 = 3,
	ALL = 254,
};

enum class ServoLED
{
	NONE = 0,
	RED,
	GREEN,
	YELLOW,
	BLUE,
	PURPLE,
	CYAN,
	WHITE
};

namespace Platform
{
	const int leds[] = { 11, 12, 13, 20, 21 };
	const int buttons[] = { 5, 6, 7 , 8 };
	const int startPull = 2;
	const int gp2s[] = { 0, 1 };// , 2, 3};

	void Init();

	void DisplayNumber(int n);
	void SetLed(int ledId, bool state);
	bool IsButtonPressed(int id);
	bool IsGP2Occluded();
	void DebugGP2();

	void SetServoLED(ServoID id, ServoLED color);
	//pos: 0 to 1023
	void SetServoPos(ServoID id, int pos);
	//speed 0 to 1023
	void SetServoSpeed(ServoID id, int speed);
	// set baudrate to 115200 for all servo connected
	void ForceServoBaudRate();
	// set id of one servo connected, id between 1 and 253 (but not 200)
	void ForceServoId(int id);
}

#endif
