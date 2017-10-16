#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "Globals.h"

#define ENABLE_POSITIONNING 0
#define ENABLE_TIMER 0
#define ENABLE_AVOIDANCE 1
#define ENABLE_EXTRA 1

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

enum class FunnyState
{
	NORMAL,
	EJECT
};

class Strategy
{
public:
	enum class State
	{
		POSITIONING,
		POSITIONING2,
		WAITING_START,
		MODULE_A0,
		MODULE_A1,
		MODULE_A2,
		MODULE_A3,
		MODULE_B1,
		MODULE_B2,
		MODULE_B3,
		MODULE_B4,
		MODULE_B5,
		MODULE_B6,
		MODULE_C1,
		MODULE_C2,
		MODULE_C3,
		MODULE_C4,
#if ENABLE_EXTRA
		MODULE_RECAL0,
		MODULE_RECAL1,
#endif
		MODULE_E1,
		MODULE_E2,
		MODULE_E3,
		MODULE_E4,
		WAITING_END,
		END
	};
	
	static Strategy Instance;
	Strategy();
	void Init();
	void Task();
	void Start();
	
	void SetInitialPosition();
	void PushRobotAgainstWall(uint32_t durationMs = 1500, bool goForward = true);
	void RePosAgainstSideBase();
	void RePosAgainstFrontBase();

	Side GetSide() { return m_Side; }
	void SetSide(Side _side);
	void Print();

	Float2 GetGameElementPosition(GameElement _module);
	Float2 GetCorrectPos(float x, float y);
	float GetCorrectAngle(float a);

	void SetArmState(ArmState _state);
	void SetGripState(GripState _state);
	void SetFunnyState(FunnyState _state);

private:
	Side		m_Side = Side::BLUE;
	State		m_State = (State)0;
	uint32_t	m_StartTime = 0;
};

Strategy::State operator++(Strategy::State &s, int);

#endif
