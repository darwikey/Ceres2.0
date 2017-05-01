/*
  FrequencyTimer.h - A frequency generator and interrupt generator library
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.

  http://www.arduino.cc/playground/Code/FrequencyTimer

  Version 2.1 - updated by Paul Stoffregen, paul@pjrc.com
  for compatibility with Teensy 3.1

  Version 2 - updated by Paul Stoffregen, paul@pjrc.com
  for compatibility with newer hardware and Arduino 1.0

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "Scheduler.h"
#include "WProgram.h"


void (*Scheduler::onOverflow)() = 0;
bool Scheduler::enabled = 0;


#if defined(__arm__) && defined(TEENSYDUINO)

void Scheduler::setPeriod(unsigned long usPeriod)
{
	uint8_t bdiv, cdiv=0;

	if (usPeriod == 0) usPeriod = 1;
	usPeriod *= (F_BUS / 1000000);
	if (usPeriod < 65535*16) {
		bdiv = 0;
	} else if (usPeriod < 65535*2*16) {
		bdiv = 1;
	} else if (usPeriod < 65535*3*16) {
		bdiv = 2;
	} else if (usPeriod < 65535*4*16) {
		bdiv = 3;
	} else if (usPeriod < 65535*5*16) {
		bdiv = 4;
	} else if (usPeriod < 65535*6*16) {
		bdiv = 5;
	} else if (usPeriod < 65535*7*16) {
		bdiv = 6;
	} else if (usPeriod < 65535*8*16) {
		bdiv = 7;
	} else if (usPeriod < 65535*9*16) {
		bdiv = 8;
	} else if (usPeriod < 65535*10*16) {
		bdiv = 9;
	} else if (usPeriod < 65535*11*16) {
		bdiv = 10;
	} else if (usPeriod < 65535*12*16) {
		bdiv = 11;
	} else if (usPeriod < 65535*13*16) {
		bdiv = 12;
	} else if (usPeriod < 65535*14*16) {
		bdiv = 13;
	} else if (usPeriod < 65535*15*16) {
		bdiv = 14;
	} else if (usPeriod < 65535*16*16) {
		bdiv = 15;
	} else if (usPeriod < 65535*18*16) {
		bdiv = 8;
		cdiv = 1;
	} else if (usPeriod < 65535*20*16) {
		bdiv = 9;
		cdiv = 1;
	} else if (usPeriod < 65535*22*16) {
		bdiv = 10;
		cdiv = 1;
	} else if (usPeriod < 65535*24*16) {
		bdiv = 11;
		cdiv = 1;
	} else if (usPeriod < 65535*26*16) {
		bdiv = 12;
		cdiv = 1;
	} else if (usPeriod < 65535*28*16) {
		bdiv = 13;
		cdiv = 1;
	} else if (usPeriod < 65535*30*16) {
		bdiv = 14;
		cdiv = 1;
	} else if (usPeriod < 65535*32*16) {
		bdiv = 15;
		cdiv = 1;
	} else if (usPeriod < 65535*36*16) {
		bdiv = 8;
		cdiv = 2;
	} else if (usPeriod < 65535*40*16) {
		bdiv = 9;
		cdiv = 2;
	} else if (usPeriod < 65535*44*16) {
		bdiv = 10;
		cdiv = 2;
	} else if (usPeriod < 65535*48*16) {
		bdiv = 11;
		cdiv = 2;
	} else if (usPeriod < 65535*52*16) {
		bdiv = 12;
		cdiv = 2;
	} else if (usPeriod < 65535*56*16) {
		bdiv = 13;
		cdiv = 2;
	} else if (usPeriod < 65535*60*16) {
		bdiv = 14;
		cdiv = 2;
	} else if (usPeriod < 65535*64*16) {
		bdiv = 15;
		cdiv = 2;
	} else if (usPeriod < 65535*72*16) {
		bdiv = 8;
		cdiv = 3;
	} else if (usPeriod < 65535*80*16) {
		bdiv = 9;
		cdiv = 3;
	} else if (usPeriod < 65535*88*16) {
		bdiv = 10;
		cdiv = 3;
	} else if (usPeriod < 65535*96*16) {
		bdiv = 11;
		cdiv = 3;
	} else if (usPeriod < 65535*104*16) {
		bdiv = 12;
		cdiv = 3;
	} else if (usPeriod < 65535*112*16) {
		bdiv = 13;
		cdiv = 3;
	} else if (usPeriod < 65535*120*16) {
		bdiv = 14;
		cdiv = 3;
	} else {
		bdiv = 15;
		cdiv = 3;
	}
	usPeriod /= (bdiv + 1);
	usPeriod >>= (cdiv + 4);
	if (usPeriod > 65535) usPeriod = 65535;
	// high time = (CMD1:CMD2 + 1) ÷ (fCMTCLK ÷ 8)
	// low time  = CMD3:CMD4 ÷ (fCMTCLK ÷ 8)
	SIM_SCGC4 |= SIM_SCGC4_CMT;
	CMT_MSC = 0;
	CMT_PPS = bdiv;
	CMT_CMD1 = ((usPeriod - 1) >> 8) & 255;
	CMT_CMD2 = (usPeriod - 1) & 255;
	CMT_CMD3 = (usPeriod >> 8) & 255;
	CMT_CMD4 = usPeriod & 255;
	CMT_OC = 0x60;
	CMT_MSC = (cdiv << 5) | 0x0B; // baseband mode
}

unsigned long Scheduler::getPeriod()
{
	uint32_t period;

	period = (CMT_CMD3 << 8) | CMT_CMD4;
	period *= (CMT_PPS + 1);
	period <<= ((CMT_MSC >> 5) & 3) + 4;
	period /= (F_BUS / 1000000);
	return period;
}

void Scheduler::enable()
{
	NVIC_ENABLE_IRQ(IRQ_CMT);
	//Scheduler::enabled = 1;
	//CORE_PIN5_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_DSE|PORT_PCR_SRE;
}

void Scheduler::disable()
{
	NVIC_DISABLE_IRQ(IRQ_CMT);
	//Scheduler::enabled = 0;
	//CORE_PIN5_CONFIG = PORT_PCR_MUX(1)|PORT_PCR_DSE|PORT_PCR_SRE;
	//digitalWriteFast(5, LOW);
}

void Scheduler::setOnOverflow( void (*func)() )
{
	if (func) {
		Scheduler::onOverflow = func;
		NVIC_ENABLE_IRQ(IRQ_CMT);
	} else {
		NVIC_DISABLE_IRQ(IRQ_CMT);
		Scheduler::onOverflow = func;
	}
}

void cmt_isr(void)
{
	static volatile uint8_t inHandler = 0;

	uint8_t __attribute__((unused)) tmp = CMT_MSC;
	tmp = CMT_CMD2;
	if ( !inHandler && Scheduler::onOverflow) {
		inHandler = 1;
		(*Scheduler::onOverflow)();
		inHandler = 0;
	}
}

#endif
