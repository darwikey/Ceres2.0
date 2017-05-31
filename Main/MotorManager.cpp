#include "WProgram.h"
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
	m_RightMotorPID.Init(1.f, 0.f, 0.5f);
	m_LeftMotorPID.Init(1.f, 0.f, 0.5f);
}


//void updateEnc() {
 // lastEnc[0] = curEnc[0];
  //lastEnc[1] = curEnc[1];

  //curEnc[0] = -encoder2.calcPosn();
  //curEnc[1] = encoder1.calcPosn();
//}

//static float pidSpeeds[3] = {
//	//    P     I     D
//	1.f,   0.,  0.5 
//};

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

	/*int32_t P = pidSpeeds[0] * err;
	int32_t I = 0;
	int32_t D = pidSpeeds[2] * (err - m_LastErr[m]);

	int32_t cmd = P + I + D;*/

	int cmd;
	if (m == RIGHT)
	{
		cmd = PIDController::EvaluatePID(&m_RightMotorPID, (float)err);
	}
	else
	{
		cmd = PIDController::EvaluatePID(&m_LeftMotorPID, (float)err);
	}

	SendCommand(m, cmd);

	//m_LastErr[m] = err;
}

//int32_t lastErrAngle = 0;
//static float pidAngle[3] = {
//	//    P     I     D
//		0.25,   0.,  0.25,
//};

//int32_t lastDistance = 0;
//int32_t lastErrSpeed = 0;
//static float pidSpeed[3] = {
//	//    P     I     D
//		0.5,   0.,  0.5,
//};

/*void MotorManager::updateAngleSpeed(int angle, int speed) {
	int32_t actualAngle = (curEnc[1] - curEnc[0]) / 2;
	int32_t actualDistance = (curEnc[1] + curEnc[0]) / 2;

	int32_t errAngle = angle - actualAngle;

	int32_t PAngle = pidAngle[0] * errAngle;
	int32_t IAngle = 0;
	int32_t DAngle = pidAngle[2] * (errAngle - lastErrAngle);

	int32_t actualSpeed = lastDistance - actualDistance;

	int32_t errSpeed = speed - actualSpeed;

	int32_t PSpeed = pidSpeed[0] * errSpeed;
	int32_t ISpeed = 0;
	int32_t DSpeed = pidSpeed[2] * (errSpeed - lastErrSpeed);

	updateMotor(1, PSpeed + ISpeed + DSpeed + PAngle + IAngle + DAngle);
	updateMotor(0, PSpeed + ISpeed + DSpeed - PAngle - IAngle - DAngle);

	lastErrAngle = errAngle;
	lastDistance = actualDistance;
	lastErrSpeed = errSpeed;
}*/


