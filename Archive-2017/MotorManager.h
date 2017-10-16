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
	void SetMotorPidP(float k) { m_RightMotorPID.SetKP(k); m_LeftMotorPID.SetKP(k); }
	void SetMotorPidI(float k) { m_RightMotorPID.SetKI(k); m_LeftMotorPID.SetKI(k); }
	void SetMotorPidD(float k) { m_RightMotorPID.SetKD(k); m_LeftMotorPID.SetKD(k); }

private:
	void updateMotor(MotorId m, int speed);

	PIDController m_RightMotorPID;
	PIDController m_LeftMotorPID;

	int32_t m_LastEnc[2] = {0};
};

#endif

