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

void Strategy::SetInitialPosition()
{
	if (m_Side == Side::BLUE)
	{
		PositionManager::Instance.SetPosMm(Float2(1070.f - 0.5f * ROBOT_WIDTH, ROBOT_CENTER_BACK));
	}
	else
	{
		PositionManager::Instance.SetPosMm(Float2(1930.f + 0.5f * ROBOT_WIDTH, ROBOT_CENTER_BACK));
	}
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
	Serial.println("set initial pos");
	SetInitialPosition();
}

Float2 Strategy::GetGameElementPosition(GameElement _module)
{
	if (m_Side == Side::BLUE)
	{
		if (_module == GameElement::MODULE1)
			return Float2(1000.f, 600.f);
		if (_module == GameElement::MODULE2)
			return Float2(200.f, 600.f);
		if (_module == GameElement::MODULE3)
			return Float2(500.f, 1100.f);
		if (_module == GameElement::MODULE4)
			return Float2(900.f, 1400.f);
		if (_module == GameElement::MODULE5)
			return Float2(800.f, 1850.f);
	}
	else
	{
		if (_module == GameElement::MODULE1)
			return Float2(2000.f, 600.f);
		if (_module == GameElement::MODULE2)
			return Float2(2800.f, 600.f);
		if (_module == GameElement::MODULE3)
			return Float2(2500.f, 1100.f);
		if (_module == GameElement::MODULE4)
			return Float2(2100.f, 1400.f);
		if (_module == GameElement::MODULE5)
			return Float2(2200.f, 1850.f);
	}
	Serial.println("incorrect game element");
	return Float2();
}


