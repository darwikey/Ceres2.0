#ifndef _STRATEGY_H_
#define _STRATEGY_H_

enum class Side
{
	BLUE,
	YELLOW
};

class Strategy
{
public:
	static Strategy Instance;
	Strategy();
	void Init();

	Side GetSide() { return m_Side; }
	void PrintSide();
	void SetSide(Side _side);

private:
	Side m_Side = Side::BLUE;
};


#endif
