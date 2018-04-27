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
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(2390.f, 1700.f));
		break;

	case State::WATER_PLANT0:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(1870.f, 1500.f));
		break;

	/*case State::MODULE_A0:
		TrajectoryManager::Instance.GotoDistance(50.f);
		break;

	case State::MODULE_A1:
		if (m_Side == Side::ORANGE)
			TrajectoryManager::Instance.GotoXY(GetCorrectPos(1200.f, 600.f));
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(1000.f, 600.f));
		break;

	case State::MODULE_A2:
		SetGripState(GripState::CLOSE);
		SetArmState(ArmState::EMPTYING);
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(775, 790.f));
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(250, 775.f));
		break;

	case State::MODULE_A3:
		PushRobotAgainstWall();
		RePosAgainstSideBase();
		SetGripState(GripState::FULLY_OPEN);
		TrajectoryManager::Instance.GotoDistance(-100.f);
		break;

	case State::MODULE_B1:
		SetGripState(GripState::NORMAL);
		SetArmState(ArmState::NORMAL);
		SetGripState(GripState::FULLY_OPEN);
		break;
	
	case State::MODULE_B2:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(200, 600.f));
		break;

	case State::MODULE_B3:
		SetGripState(GripState::CLOSE);
		SetArmState(ArmState::EMPTYING);
		TrajectoryManager::Instance.GotoDistance(-200.f);
		break;

	case State::MODULE_B4:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(400.f, 920.f));
		break;

	case State::MODULE_B5:
		TrajectoryManager::Instance.GotoXY(GetCorrectPos(250.f, 920.f));
		break;

	case State::MODULE_B6:
		PushRobotAgainstWall();
		RePosAgainstSideBase();
		SetGripState(GripState::FULLY_OPEN);
		if (m_Side == Side::GREEN)
		{
			TrajectoryManager::Instance.GotoDistance(-200.f);
		}
		else
		{
			TrajectoryManager::Instance.GotoDistance(-700.f);
		}
		break;
*/

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
	int cmd = -30;
	if (!goForward)
		cmd = -cmd;
	MotorManager::Instance.SetSpeed(MotorManager::RIGHT, cmd);
	MotorManager::Instance.SetSpeed(MotorManager::LEFT, cmd);
	delay(durationMs);
	MotorManager::Instance.SetSpeed(MotorManager::RIGHT, 0);
	MotorManager::Instance.SetSpeed(MotorManager::LEFT, 0);
	ControlSystem::Instance.Reset();
	ControlSystem::Instance.m_Enable = true;
}

void Strategy::RePosAgainstSideBase()
{
	PositionManager::Instance.SetAngleDeg(GetCorrectAngle(90.f));
	PositionManager::Instance.SetPosMm(GetCorrectPos(202.f, PositionManager::Instance.GetPosMm().y));
}

void Strategy::RePosAgainstFrontBase()
{
	PositionManager::Instance.SetAngleDeg(GetCorrectAngle(0.f));
	PositionManager::Instance.SetPosMm(Float2(PositionManager::Instance.GetPosMm().x, 382.f + ROBOT_CENTER_BACK));
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
//
//Float2 Strategy::GetGameElementPosition(GameElement _module)
//{
//	if (m_Side == Side::BLUE)
//	{
//		if (_module == GameElement::MODULE_A)
//			return Float2(1000.f, 600.f);
//		if (_module == GameElement::MODULE_B)
//			return Float2(200.f, 600.f);
//		if (_module == GameElement::MODULE_C)
//			return Float2(500.f, 1100.f);
//		if (_module == GameElement::MODULE_D)
//			return Float2(900.f, 1400.f);
//		if (_module == GameElement::MODULE_E)
//			return Float2(800.f, 1850.f);
//	}
//	else
//	{
//		if (_module == GameElement::MODULE_A)
//			return Float2(2000.f, 600.f);
//		if (_module == GameElement::MODULE_B)
//			return Float2(2800.f, 600.f);
//		if (_module == GameElement::MODULE_C)
//			return Float2(2500.f, 1100.f);
//		if (_module == GameElement::MODULE_D)
//			return Float2(2100.f, 1400.f);
//		if (_module == GameElement::MODULE_E)
//			return Float2(2200.f, 1850.f);
//	}
//	Serial.println("incorrect game element");
//	return Float2();
//}

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
		Platform::SetServoPos(ServoID::SERVO3, 237);
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
