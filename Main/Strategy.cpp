#include <WProgram.h>
#include <EEPROM.h>
#include "Strategy.h"
#include "Platform.h"
#include "TrajectoryManager.h"
#include "PositionManager.h"
#include "MotorManager.h"

Strategy Strategy::Instance;

Strategy::Strategy()
{
}

void Strategy::Init()
{
#if ENABLE_POSITIONNING
	m_State = State::POSITIONING;
#else
	m_State = State::WAITING_START;
#endif
	eeprom_initialize();
	eeprom_busy_wait();
	//if (eeprom_read_byte(0) == (uint8_t)Side::BLUE)
	if (Platform::IsButtonPressed(2))
		m_Side = Side::GREEN;
	else
		m_Side = Side::ORANGE;
}

void Strategy::Task()
{
#if ENABLE_POSITIONNING
	if (m_State == State::POSITIONING)
	{
		if (Platform::IsStartPulled())
		{
			TrajectoryManager::Instance.GotoXY(Float2(PositionManager::Instance.GetXMm(), 360.f - ROBOT_CENTER_FRONT - 30.f));
			m_State++;
		}
	}
	if (m_State == State::POSITIONING2 && !Platform::IsStartPulled())
	{
		delay(500);
		m_State++;
	}
#endif

	if (m_State == State::WAITING_START)
	{
#if ENABLE_TIMER
		if (Platform::IsStartPulled())
			Start();
#endif
		return;
	}

#if ENABLE_TIMER
	if (millis() > m_StartTime + 90000)
	{
		m_State = State::END;
		TrajectoryManager::Instance.Pause();
		MotorManager::Instance.Enabled = false;
		return;
	}
#endif

#if ENABLE_AVOIDANCE
	if (Platform::IsGP2Occluded(TrajectoryManager::Instance.IsForwardMovement()))
	{
		TrajectoryManager::Instance.Pause();
		return;
	}
	else
	{
		TrajectoryManager::Instance.Resume();
	}
#endif

	if (!TrajectoryManager::Instance.IsEnded())
		return;

	switch (m_State)
	{
	case State::WATER_TOWER0:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(2260.f, 1540.f));
		TrajectoryManager::Instance.GotoDegreeAngle(0.f);
		break;

	case State::WATER_TOWER01:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(2260.f, 1800.f));
		break;

	case State::WATER_TOWER1:
		PushRobotAgainstWall();
		RePosAgainstBackWall();
		TrajectoryManager::Instance.GotoDistance(-50.f);
		TrajectoryManager::Instance.GotoDegreeAngle(-90.f);
		break;

	case State::WATER_TOWER2:
		PushRobotAgainstWall(1500, m_Side == Side::ORANGE);// go backward when side=green
		RePosAgainstWaterPlantSide();
		if (m_Side == Side::GREEN)
			TrajectoryManager::Instance.GotoDistance(2390.f - PositionManager::Instance.GetTheoreticalPosMm().x);//forward
		else
			TrajectoryManager::Instance.GotoDistance(610.f - PositionManager::Instance.GetTheoreticalPosMm().x);//backward
		//TrajectoryManager::Instance.GotoXY(GetCorrectPos(2390.f, PositionManager::Instance.GetPosMm().y));
		break;

	case State::WATER_TOWER3:
		SetArmState(ArmState::OPEN);
		break;

	case State::WATER_PLANT0:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(2390.f, 1500.f));
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(1870.f, 1500.f));
		TrajectoryManager::Instance.GotoDegreeAngle(0.f);
		break;

	case State::WATER_PLANT1:
		PushRobotAgainstWall();
		RePosAgainstWaterPlantFront();
		TrajectoryManager::Instance.GotoDistance(-50.f);
		TrajectoryManager::Instance.GotoDegreeAngle(-90.f);
		break;

	case State::WATER_PLANT2:
		SetDoorState(DoorState::OPEN);
		delay(2000);
		for (int i = 0; i < 2; i++) // shake the door
		{
			SetDoorState(DoorState::CLOSE);
			SetDoorState(DoorState::OPEN);
			delay(2000);
		}
		break;

	default:
		return;
	}

	if (m_State < State::WAITING_END)
	{
		m_State++;
		Serial.printf("new state: %d\r\n", (int)m_State);
	}
}

void Strategy::Start()
{
	if (m_State == State::WAITING_START)
	{
		m_State++;
		m_StartTime = millis();
		Platform::InitServo();
		SetArmState(ArmState::NORMAL);
		SetDoorState(DoorState::CLOSE);
	}
}

void Strategy::SetInitialPosition()
{
	float y = 650.f - 0.5f * ROBOT_WIDTH;
	if (m_Side == Side::GREEN)
	{
		PositionManager::Instance.SetAngleDeg(-90.f);
		PositionManager::Instance.SetPosMm(Float2(400.f - ROBOT_CENTER_FRONT, y));
	}
	else
	{
		PositionManager::Instance.SetAngleDeg(90.f);
		PositionManager::Instance.SetPosMm(Float2(2600.f + ROBOT_CENTER_FRONT, y));
	}
}

void Strategy::PushRobotAgainstWall(uint32_t durationMs, bool goForward)
{
	ControlSystem::Instance.m_Enable = false;
	int cmd = -60;
	if (!goForward)
		cmd = -cmd;
	for (uint32_t i = 0; i < durationMs / 10; i++)
	{
		MotorManager::Instance.SetSpeed(MotorManager::RIGHT, cmd);
		MotorManager::Instance.SetSpeed(MotorManager::LEFT, cmd);
		delay(10);
	}
	MotorManager::Instance.SetSpeed(MotorManager::RIGHT, 0);
	MotorManager::Instance.SetSpeed(MotorManager::LEFT, 0);
	ControlSystem::Instance.Reset();
	ControlSystem::Instance.m_Enable = true;
}

void Strategy::RePosAgainstBackWall()
{
	PositionManager::Instance.SetAngleDeg(GetCorrectAngle(0.f));
	PositionManager::Instance.SetPosMm(Float2(PositionManager::Instance.GetPosMm().x, 2000.f - ROBOT_CENTER_FRONT));
}

void Strategy::RePosAgainstWaterPlantSide()
{
	PositionManager::Instance.SetAngleDeg(-90.f);
	float x;
	if (m_Side == Side::GREEN)
		x = 2100.f + ROBOT_CENTER_BACK;
	else
		x = 900.f - ROBOT_CENTER_FRONT;
	PositionManager::Instance.SetPosMm(Float2(x, PositionManager::Instance.GetPosMm().y));
}

void Strategy::RePosAgainstWaterPlantFront()
{
	PositionManager::Instance.SetAngleDeg(GetCorrectAngle(0.f));
	PositionManager::Instance.SetPosMm(Float2(PositionManager::Instance.GetPosMm().x, 1750.f - ROBOT_CENTER_FRONT));
}

void Strategy::Print()
{
	Serial.print("State: ");
	switch (m_State)
	{
	case State::WAITING_START:
		Serial.print("waiting start\r\n");
		break;
	case State::WAITING_END:
		Serial.print("waiting end\r\n");
		break;
	case State::END:
		Serial.print("end\r\n");
		break;
	default:
		Serial.printf("actions (%d)\r\n", (int)m_State);
		break;
	};
	Serial.printf("Side: %s\r\n", m_Side == Side::GREEN ? "Green" : "Orange");
	Serial.printf("Time since start: %ds\r\n", (millis() - m_StartTime) / 1000);
	Serial.printf("Is trajectory paused: %d\r\n", (int)TrajectoryManager::Instance.IsPaused());
}

void Strategy::SetSide(Side _side)
{
	m_Side = _side;
	eeprom_busy_wait();
	eeprom_write_byte(0, (uint8_t)m_Side);
	Serial.println("set initial pos");
	SetInitialPosition();
}

Float2 Strategy::GetCorrectPos(float x, float y)
{
	if (m_Side == Side::GREEN)
		return Float2(x, y);
	else
		return Float2(3000.f - x, y);
}

float Strategy::GetCorrectAngle(float a)
{
	if (m_Side == Side::GREEN)
		return a;
	else
		return -a;
}

void Strategy::SetArmState(ArmState _state)
{
	switch (_state)
	{
	case ArmState::NORMAL:
		Platform::SetServoPos(ServoID::SERVO2, 800);
		break;
	case ArmState::OPEN:
		Platform::SetServoPos(ServoID::SERVO2, 100);
		break;
	}
	delay(1000);
}

void Strategy::SetDoorState(DoorState _state)
{
	switch (_state)
	{
	case DoorState::CLOSE:
		Platform::SetServoPos(ServoID::SERVO3, 210);
		break;
	case DoorState::OPEN:
		Platform::SetServoPos(ServoID::SERVO3, 400);
		break;
	}
	delay(500);
}

Strategy::State operator++(Strategy::State &s, int)
{
	if (s != Strategy::State::END)
		s = (Strategy::State)((int)s + 1);
	return s;
}
