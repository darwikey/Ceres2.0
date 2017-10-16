#include "MotorManager.h"
#include "Platform.h"
#include "PositionManager.h"

const int motorPWMs[] = { 22, 23 };
const int motorDirs[] = { 26, 31 };

MotorManager MotorManager::Instance;

void MotorManager::Init()
{
	for (unsigned i = 0; i < _countof(motorPWMs); ++i)
	{
		pinMode(motorPWMs[i], OUTPUT);
		pinMode(motorDirs[i], OUTPUT);
	}
	m_RightMotorPID.Init(0.f, 0.f, 0.f);
	m_LeftMotorPID.Init(0.f, 0.f, 0.f);
	SetMotorPidP(1.f);
	SetMotorPidD(0.5f);
}

void MotorManager::SetSpeed(MotorId m, int32_t speed)
{
	updateMotor(m, speed);
}

void MotorManager::SendCommand(MotorId m, int32_t cmd)
{
	if (!Enabled)
		cmd = 0;

	if (m == MotorManager::RIGHT)
		cmd = -cmd;
	int32_t AbsCmd = abs(cmd);
	//Serial.printf("%d\r\n", (int)AbsCmd);
	if (AbsCmd < 15)
		AbsCmd = 0;
	const int32_t maxi = 255;
	analogWrite(motorPWMs[m], (AbsCmd > maxi) ? maxi : AbsCmd);
	digitalWrite(motorDirs[m], (cmd >= 0) ? HIGH : LOW);
}

void MotorManager::updateMotor(MotorId m, int speed)
{
	int32_t curEnc = (m == RIGHT) ? PositionManager::Instance.GetLeftEncoder() : PositionManager::Instance.GetRightEncoder(); // Yep it's not logic...
	int32_t actualSpeed = curEnc - m_LastEnc[m];
	m_LastEnc[m] = curEnc;
	int32_t err = speed - actualSpeed;

	int cmd;
	if (m == RIGHT)
	{
		cmd = m_RightMotorPID.EvaluatePID((float)err);
	}
	else
	{
		cmd = m_LeftMotorPID.EvaluatePID((float)err);
	}

	SendCommand(m, cmd);
}

