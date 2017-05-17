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
	MODULE1 = 1 << 0,
	MODULE2 = 1 << 1,
	MODULE3 = 1 << 2,
	MODULE4 = 1 << 3,
	MODULE5 = 1 << 4,
	MODULE6 = 1 << 5,
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
		ACTIONS,
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

	void SetArmState(ArmState _state);
	void SetGripState(GripState _state);

private:
	Side		m_Side = Side::BLUE;
	State		m_State = State::WAITING_START;
	uint32_t	m_StartTime = 0;
};

Strategy::State operator++(Strategy::State &s, int);

#endif
