#include <WProgram.h>
#include <EEPROM.h>
#include "Strategy.h"
#include "Astar.h"
#include "TrajectoryManager.h"
#include "PositionManager.h"

Strategy Strategy::Instance;

Strategy::Strategy()
{
}

void Strategy::Init()
{
	eeprom_initialize();
	eeprom_busy_wait();
	if (eeprom_read_byte(0) == (uint8_t)Side::BLUE)
		m_Side = Side::BLUE;
	else
		m_Side = Side::YELLOW;
}

void Strategy::PrintSide()
{
	Serial.printf("Side: %s\r\n", m_Side == Side::BLUE ? "Blue" : "Yellow");
}

void Strategy::SetSide(Side _side)
{
	m_Side = _side;
	eeprom_busy_wait();
	eeprom_write_byte(0, (uint8_t)m_Side);
}


