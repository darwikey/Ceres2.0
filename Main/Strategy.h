#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "Float2.h"

#define ROBOT_WIDTH 235.f
#define ROBOT_CENTER_BACK 55.f // distance between axle and back
#define ROBOT_CENTER_FRONT (150.f-ROBOT_CENTER_BACK) // distance between axle and front

enum class Side
{
	BLUE,
	YELLOW
};

enum class GameElement
{
	MODULE_A = 1 << 0,
	MODULE_B = 1 << 1,
	MODULE_C = 1 << 2,
	MODULE_D = 1 << 3,
	MODULE_E = 1 << 4,
	MODULE_F = 1 << 5,
};

enum class ArmState
{
	NORMAL,
	EMPTYING
};

enum class GripState
{
	CLOSE,
	NORMAL,
	FULLY_OPEN
};

class Strategy
{
public:
	enum class State
	{
		WAITING_START,
		MODULE_A1,
		MODULE_A2,
		MODULE_A3,
		MODULE_B1,
		MODULE_B2,
		MODULE_B3,
		MODULE_B4,
		WAITING_END,
		END
	};
	
	static Strategy Instance;
	Strategy();
	void Init();
	void Task();
	void Start();
	
	void SetInitialPosition();
	void PushRobotAgainstWall();

	Side GetSide() { return m_Side; }
	void SetSide(Side _side);
	void Print();

	Float2 GetGameElementPosition(GameElement _module);
	Float2 GetCorrectPos(float x, float y);
	float GetCorrectAngle(float a);

	void SetArmState(ArmState _state);
	void SetGripState(GripState _state);

private:
	Side		m_Side = Side::BLUE;
	State		m_State = State::WAITING_START;
	uint32_t	m_StartTime = 0;
};

Strategy::State operator++(Strategy::State &s, int);

#endif
