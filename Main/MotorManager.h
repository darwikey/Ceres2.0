#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

class MotorManager
{
public:
	enum MotorId
	{
		LEFT,
		RIGHT
	};

	static MotorManager Instance;
	void Init();
	void SendCommand(MotorId m, int32_t cmd);

	bool Enabled = true;
};

#endif

