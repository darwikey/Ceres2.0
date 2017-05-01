#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "Float2.h"

#define ROBOT_WIDTH 235.f
#define ROBOT_CENTER_BACK 55.f // distance between axle and back

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
	static Strategy Instance;
	Strategy();
	void Init();
	void SetInitialPosition();

	Side GetSide() { return m_Side; }
	void PrintSide();
	void SetSide(Side _side);

	Float2 GetGameElementPosition(GameElement _module);

	void SetArmState(ArmState _state);
	void SetGripState(GripState _state);

private:
	Side m_Side = Side::BLUE;
};


#endif
