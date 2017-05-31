#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "PIDController.h"

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
	void SetSpeed(MotorId m, int32_t cmd);
	void SendCommand(MotorId m, int32_t cmd);

	bool Enabled = true;

	PIDController& GetRightMotorPID() { return m_RightMotorPID; }
	PIDController& GetLeftMotorPID() { return m_LeftMotorPID; }

private:
	void updateMotor(MotorId m, int speed);

	PIDController m_RightMotorPID;
	PIDController m_LeftMotorPID;


	int32_t m_LastEnc[2] = {0};
	//int32_t m_LastErr[2] = {0};
};

#endif

