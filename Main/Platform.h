#ifndef PLATFORM_H
#define PLATFORM_H

#define _countof(a) (sizeof(a)/sizeof(*(a)))

#define GP2_BLOCK_AT   1000 /*432*/ /*288*/
#define GP2_UNBLOCK_AT 1000 /*324*/ /*216*/

enum class ServoID
{
	SERVO1 = 1,
	ALL = 254,
};

namespace Platform
{
	const int leds[] = { 11, 12, 13, 20, 21 };
	const int buttons[] = { 5, 6, 7 , 8 };
	const int startPull = 2;
	const int gp2s[] = { 0, 1, 2, 3 };

	void Init();

	void DisplayNumber(int n);
	bool IsButtonPressed(int id);
	bool IsGp2Occluded();
	void SetServoLED(ServoID id, int color);
	//pos: 0 to 1023
	void SetServoPos(ServoID id, int pos);
	//speed 0 to 1023
	void SetServoSpeed(ServoID id, int speed);
}

#endif
