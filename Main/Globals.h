#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <WProgram.h>
#include "Float2.h"

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

	inline float GetVectorAngle(const Float2 &_v)
	{
		return atan2f(-_v.x, _v.y);
	}

	template <typename T>
	inline T Lerp(const T &_x, const T &_y, const T &_s)
	{
		return _x + _s * (_y - _x);
	}
};

#include "CircularBuffer.h"

#endif
