#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "Vector2.h"

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

private:
	Side m_Side = Side::BLUE;
};


#endif
