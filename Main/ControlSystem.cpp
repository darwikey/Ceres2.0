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
#include "WProgram.h"

//#define PI 3.1415926535
#define DEG2RAD(a) ((a) * PI / 180.0)

ControlSystem ControlSystem::Instance;


void ControlSystem::Start()
{
	m_pid_distance.Init(0.05f, 0/*0.005*/, 0.05f);
	m_pid_angle.Init(0.05f, 0, 0.05f);//0.3, 0/*0.005*/, 0.2);

	m_pid_distance.SetOutputRange(-100, 100);
	m_pid_angle.SetOutputRange(-100, 100);

	// Quadramp setup
	m_quadramp_distance.Init();
	m_quadramp_angle.Init();

	// Setting QuadrampFilter eval period to the control system period
	m_quadramp_distance.SetEvalPeriod(CONTROL_SYSTEM_PERIOD_S);
	m_quadramp_angle.SetEvalPeriod(CONTROL_SYSTEM_PERIOD_S);

	m_quadramp_distance.Set2ndOrderVars(DISTANCE_MAX_ACC, DISTANCE_MAX_ACC); // Translation acceleration (in mm/s^2)
	m_quadramp_angle.Set2ndOrderVars(DEG2RAD(ANGLE_MAX_ACC_DEG),	DEG2RAD(ANGLE_MAX_ACC_DEG)); // Rotation acceleration (in rad/s^2)

	SetSpeedHigh();

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
			float Target = m_quadramp_distance.Evaluate(m_DistanceTarget);
			float Measure = PositionManager::Instance.GetDistanceMm();
			float Error = Target - Measure;
			DistanceCmd = m_pid_distance.EvaluatePID(Error);
			
		}
		{
			float Target = m_quadramp_angle.Evaluate(m_AngleTarget);
			float Measure = PositionManager::Instance.GetAngleRad();
			float Error = Target - Measure;
			AngleCmd = m_pid_angle.EvaluatePID(Error);
		}

		SetMotorCmd(DistanceCmd, AngleCmd);
	}
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
	if ((m_LastPosition - PositionManager::Instance.GetPosMm()).LengthSquared() > 20.f * 20.f)
	{
		m_LastPosition = PositionManager::Instance.GetPosMm();
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

// User functions
void ControlSystem::SetDistanceTarget(float ref)
{
	Scheduler::disable();
	m_DistanceTarget = ref;
	Scheduler::enable();
}

void ControlSystem::SetDegAngleTarget(float ref_deg)
{
	Scheduler::disable();
	float ref_rad = DEG2RAD(ref_deg);
	m_AngleTarget = ref_rad;
	Scheduler::enable();
}

void ControlSystem::SetRadAngleTarget(float ref_rad)
{
	Scheduler::disable();
	m_AngleTarget = ref_rad;
	Scheduler::enable();
}

void ControlSystem::SetDistanceMaxSpeed(float max_speed)
{
	m_quadramp_distance.Set1stOrderVars(max_speed, max_speed);
}

void ControlSystem::SetDistanceMaxAcc(float max_acc)
{
	m_quadramp_distance.Set2ndOrderVars(max_acc, max_acc);
}

void ControlSystem::SetAngleMaxSpeed(float max_speed)
{
	m_quadramp_angle.Set1stOrderVars(DEG2RAD(max_speed), DEG2RAD(max_speed));
}

void ControlSystem::SetAngleMaxAcc(float max_acc)
{
	m_quadramp_angle.Set2ndOrderVars(DEG2RAD(max_acc), DEG2RAD(max_acc));
}

void ControlSystem::SetSpeedRatio(float ratio)
{
	if (ratio < 0) {
		ratio = 0;
	}
	else if (ratio > 1) {
		ratio = 1;
	}

	SetDistanceMaxSpeed(ratio*DISTANCE_MAX_SPEED); // Translation speed (in mm/s)

	SetAngleMaxSpeed(ratio*ANGLE_MAX_SPEED_DEG); // Rotation speed (in rad/s)
}

void ControlSystem::SetSpeedHigh()
{
	SetSpeedRatio(1);
}

void ControlSystem::SetSpeedMedium()
{
	SetSpeedRatio(0.7);
}

void ControlSystem::SetSpeedLow()
{
	SetSpeedRatio(0.5);
}

PIDController& ControlSystem::GetDistancePID()
{
	return m_pid_distance;
}

PIDController& ControlSystem::GetAnglePID()
{
	return m_pid_angle;
}

void ControlSystem::Reset()
{
	SetDistanceTarget(PositionManager::Instance.GetDistanceMm());
	SetRadAngleTarget(PositionManager::Instance.GetAngleRad());
	m_quadramp_distance.Reset(PositionManager::Instance.GetDistanceMm());
	m_quadramp_angle.Reset(PositionManager::Instance.GetAngleRad());
}

void ControlSystem::ResetAngle()
{
	m_quadramp_angle.Reset(PositionManager::Instance.GetAngleRad());
}
