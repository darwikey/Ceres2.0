/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.2
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Implementation file.
 */

#include "Scheduler.h"
#include "PIDController.h"
#include "MotorManager.h"
#include "ControlSystem.h"
#include "PositionManager.h"


ControlSystem ControlSystem::Instance;


void ControlSystem::Start()
{
	m_DistancePID.Init(0.075f, 0.037f, 0.037f);
	m_AnglePID.Init(0.1f, 0.05f, 0.025f);

	m_DistancePID.SetOutputRange(10.f);
	m_AnglePID.SetOutputRange(0.25f);

	// Quadramp setup
	m_DistanceQuadramp.Init();
	m_AngleQuadramp.Init();

	// Setting QuadrampFilter eval period to the control system period
	m_DistanceQuadramp.SetEvalPeriod(CONTROL_SYSTEM_PERIOD_S);
	m_AngleQuadramp.SetEvalPeriod(CONTROL_SYSTEM_PERIOD_S);

	SetSpeedHigh();// init quandramp

	m_DistanceTarget = 0.f;
	m_AngleTarget = 0.f;
}

void ControlSystem::Task()
{
	PositionManager::Instance.Update();

	if (m_Enable)
	{
		//platform_led_toggle(PLATFORM_LED1);
		float DistanceCmd, AngleCmd;
		{
			float Target = m_DistanceQuadramp.Evaluate(m_DistanceTarget);
			Debug("Dist target", Target);
			float Measure = PositionManager::Instance.GetDistanceMm();
			Debug("Dist measure", Measure);
			float Error = Target - Measure;
			Debug("Dist error", Error);
			DistanceCmd = m_DistancePID.EvaluatePID(Error);
			Debug("Dist cmd", DistanceCmd);
		}
		{
			float Target = m_AngleQuadramp.Evaluate(m_AngleTarget);
			Debug("Angle target", Target);
			float Measure = PositionManager::Instance.GetAngleRad();
			Debug("Angle measure", Measure);
			float Error = Target - Measure;
			Debug("Angle error", Error);
			AngleCmd = m_AnglePID.EvaluatePID(Error);
			Debug("Angle cmd", AngleCmd);
		}
#if 0
		static float DistCmdMax = 0.f, AngleCmdMax = 0.f;
		if (abs(DistanceCmd) > DistCmdMax)
		{
			DistCmdMax = abs(DistanceCmd);
			Serial.printf("cmd dist %f\r\n", DistCmdMax);
		}
		if (abs(AngleCmd) > AngleCmdMax)
		{
			AngleCmdMax = abs(AngleCmd);
			Serial.printf("cmd angle %f\r\n", AngleCmdMax);
		}
#endif

		SetMotorCmd(DistanceCmd, AngleCmd);
	}

	m_DebugCounter++;
	if (m_DebugCounter >= m_DebugInterval)
		m_DebugCounter = 0;
}

void ControlSystem::SetMotorCmd(float d_mm, float theta)
{
	uint32_t axle_track_mm = PositionManager::Instance.GetAxleTrackMm();

	d_mm = -d_mm;

	int32_t right_motor_ref = PositionManager::Instance.MmToTicks(d_mm + (1.0 * axle_track_mm * theta) / 2);
	int32_t left_motor_ref = PositionManager::Instance.MmToTicks(d_mm - (1.0 * axle_track_mm * theta) / 2);

	if (abs(right_motor_ref) > 50 || abs(left_motor_ref) > 50)
		m_MotorCounter++;
	else
		m_MotorCounter = 0;

	// if the robot has changed its position
	if ((m_LastPosition - PositionManager::Instance.GetPosMm()).LengthSquared() > 20.f * 20.f
		|| abs(m_LastAngle - PositionManager::Instance.GetAngleRad()) > DEG2RAD(20.f))
	{
		m_LastPosition = PositionManager::Instance.GetPosMm();
		m_LastAngle = PositionManager::Instance.GetAngleRad();
		m_MotorCounter = 0;
	}

	// Ouch, the robot want to move and he cant !
	if (m_MotorCounter > 200)
	{
		MotorManager::Instance.Enabled = false;
		Serial.print("Alert: the robot want to move and he can't, shutdown motors\r\n");
		m_MotorCounter = 0;
	}

	//printf("cmd right %d   left %d\r\n", (int)right_motor_ref, (int)left_motor_ref);
	MotorManager::Instance.SetSpeed(MotorManager::RIGHT, right_motor_ref);
	MotorManager::Instance.SetSpeed(MotorManager::LEFT, left_motor_ref);
}

void ControlSystem::Debug(const char * msg, float value)
{
	if (m_DebugInterval >= 0 && m_DebugCounter == 0)
	{
		int MsgLen = Serial.print(msg);
		for (int i = 0; i < 15 - MsgLen; i++)
			Serial.print(' ');
		Serial.print(value);
		Serial.print("\r\n");
	}
}

// User functions
void ControlSystem::SetDistanceTarget(float ref)
{
	Scheduler::disable();
	m_DistanceTarget = ref;
	Scheduler::enable();
}

void ControlSystem::SetRadAngleTarget(float ref_rad, bool _useQuadramp)
{
	Scheduler::disable();
	m_AngleTarget = ref_rad;
	m_AngleQuadramp.SetEnable(_useQuadramp);
	Scheduler::enable();
}

void ControlSystem::SetDistanceMaxSpeed(float max_speed)
{
	m_DistanceQuadramp.Set1stOrderVars(max_speed, max_speed);
}

void ControlSystem::SetDistanceMaxAcc(float max_acc)
{
	m_DistanceQuadramp.Set2ndOrderVars(max_acc, max_acc);
}

void ControlSystem::SetAngleMaxSpeed(float max_speed)
{
	m_AngleQuadramp.Set1stOrderVars(DEG2RAD(max_speed), DEG2RAD(max_speed));
}

void ControlSystem::SetAngleMaxAcc(float max_acc)
{
	m_AngleQuadramp.Set2ndOrderVars(DEG2RAD(max_acc), DEG2RAD(max_acc));
}

void ControlSystem::SetSpeedAccelerationRatio(float ratio)
{
	ratio = Math::Clamp(ratio, 0.f, 1.f);

	SetDistanceMaxSpeed(ratio*DISTANCE_MAX_SPEED); // Translation speed (in mm/s)
	SetAngleMaxSpeed(ratio*ANGLE_MAX_SPEED_DEG); // Rotation speed (in rad/s)

	SetDistanceMaxAcc(ratio * DISTANCE_MAX_ACC); // Translation acceleration (in mm/s^2)
	SetAngleMaxAcc(ratio * ANGLE_MAX_ACC_DEG); // Rotation acceleration (in rad/s^2)
}

void ControlSystem::SetSpeedHigh()
{
	SetSpeedAccelerationRatio(1);
}

void ControlSystem::SetSpeedMedium()
{
	SetSpeedAccelerationRatio(0.7);
}

void ControlSystem::SetSpeedLow()
{
	SetSpeedAccelerationRatio(0.5);
}

void ControlSystem::Reset()
{
	SetDistanceTarget(PositionManager::Instance.GetDistanceMm());
	SetRadAngleTarget(PositionManager::Instance.GetAngleRad());
	m_DistanceQuadramp.Reset(PositionManager::Instance.GetDistanceMm());
	m_AngleQuadramp.Reset(PositionManager::Instance.GetAngleRad());
}

void ControlSystem::ResetAngle()
{
	m_AngleQuadramp.Reset(PositionManager::Instance.GetAngleRad());
}
