#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <WProgram.h>

#define Assert(c) if (!(c)) {Serial.printf("Assert!: %s, %d \r\n",  __FILE__, __LINE__); }
#define DEG2RAD(a) ((a) * 0.01745329252f)//PI / 180.0)

#include "Float2.h"
#include "CircularBuffer.h"

#endif
