#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <WProgram.h>

#define Assert(c) if (!(c)) {Serial.printf("Assert!: %s, %d \r\n",  __FILE__, __LINE__); }

#include "Float2.h"
#include "CircularBuffer.h"

#endif
