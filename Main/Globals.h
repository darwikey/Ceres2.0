#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <WProgram.h>

#define Assert(c) if (!(c)) {Serial.printf("Assert!: %s, %d \r\n",  __FILE__, __LINE__); }
#define DEG2RAD(a) ((a) * 0.01745329252f)//PI / 180.0)
#define RAD2DEG(a) ((a) * 57.2957795130f)

namespace Math
{
	template <typename T>
	T Clamp(T _x, T _min, T _max)
	{
		if (_x < _min)
			return _min;
		if (_x > _max)
			return _max;
		return _x;
	}
};

#include "Float2.h"
#include "CircularBuffer.h"

#endif
