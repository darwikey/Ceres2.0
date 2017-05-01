/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.2
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Implementation file.
 */

#include <stdint.h>
#include "Scheduler.h"

#include "PIDController.h"

#include "MotorManager.h"
#include "ControlSystem.h"
#include "PositionManager.h"

//#define PI 3.1415926535
#define DEG2RAD(a) ((a) * PI / 180.0)

ControlSystem ControlSystem::Instance;


static float measureDistanceMm()
{
	return PositionManager::Instance.GetDistanceMm();
}

static float measureAngleRad()
{
	return PositionManager::Instance.GetAngleRad();
}

void ControlSystem::Start()
{
	m_pid_distance.Init(0.15, 0/*0.005*/, 0.1);
	m_pid_angle.Init(0.3, 0/*0.005*/, 0.2);

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

	// Initialise each control system manager
	m_csm_distance.Init();
	m_csm_angle.Init();

	// Set reference filter
	m_csm_distance.SetReferenceFilter(QuadrampFilter::Evaluate, (void*)&(m_quadramp_distance));
	m_csm_angle.SetReferenceFilter(QuadrampFilter::Evaluate, (void*)&(m_quadramp_angle));

	// Set measure functions
	m_csm_distance.SetMeasureFetcher(measureDistanceMm);
	m_csm_angle.SetMeasureFetcher(measureAngleRad);

	// We use a pid controller because we like it here at Eirbot
	m_csm_distance.SetController(&PIDController::EvaluatePID, (void*)&(m_pid_distance));
	m_csm_angle.SetController(&PIDController::EvaluatePID, (void*)&(m_pid_angle));

	// Set processing command
	m_csm_distance.SetProcessCommand(SetDistanceMmDiff);
	m_csm_angle.SetProcessCommand(SetAngleRadDiff);

	SetDistanceMmDiff(0);
	SetAngleRadDiff(0);
}

void ControlSystem::Task()
{

	//for (;;)
	{

		PositionManager::Instance.Update();

		{
			//platform_led_toggle(PLATFORM_LED1);

			m_csm_distance.Update();
			m_csm_angle.Update();

			SetMotorsRef(m_distance_mm_diff, m_angle_rad_diff);
		}
		//ausbee_cs_manage(&(m_csm_right_motor));
		//ausbee_cs_manage(&(m_csm_left_motor));

		//vTaskDelay(CONTROL_SYSTEM_PERIOD_S * 1000 / portTICK_RATE_MS);
	}
}

void ControlSystem::SetMotorsRef(float d_mm, float theta)
{
	uint32_t axle_track_mm = PositionManager::Instance.GetAxleTrackMm();

	d_mm = -d_mm;

	int32_t right_motor_ref = PositionManager::Instance.MmToTicks(d_mm + (1.0 * axle_track_mm * theta) / 2);
	int32_t left_motor_ref = PositionManager::Instance.MmToTicks(d_mm - (1.0 * axle_track_mm * theta) / 2);

	//printf("cmd right %d   left %d\r\n", (int)right_motor_ref, (int)left_motor_ref);

	SendCommandToMotor(RIGHT_MOTOR, right_motor_ref);
	SendCommandToMotor(LEFT_MOTOR, left_motor_ref);

	//SetReference(&(m_csm_right_motor), right_motor_ref);
	//SetReference(&(m_csm_left_motor), left_motor_ref);
}

void ControlSystem::SetDistanceMmDiff(float ref)
{
	Instance.m_distance_mm_diff = ref;
}

void ControlSystem::SetAngleRadDiff(float ref)
{
	Instance.m_angle_rad_diff = ref;
}

// User functions
void ControlSystem::SetDistanceRef(float ref)
{
	Scheduler::disable();
	m_csm_distance.SetReference(ref);
	Scheduler::enable();
}

void ControlSystem::SetDegAngleRef(float ref_deg)
{
	Scheduler::disable();
	float ref_rad = DEG2RAD(ref_deg);
	m_csm_angle.SetReference(ref_rad);
	Scheduler::enable();
}

void ControlSystem::SetRadAngleRef(float ref_rad)
{
	Scheduler::disable();
	m_csm_angle.SetReference(ref_rad);
	Scheduler::enable();
}

void ControlSystem::SetRightMotorRef(int32_t ref)
{
	m_csm_right_motor.SetReference(ref);
}

void ControlSystem::SetLeftMotorRef(int32_t ref)
{
	m_csm_left_motor.SetReference(ref);
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

//TODO prendre en compte les angles != 0
void ControlSystem::ResetAngle()
{
	m_quadramp_angle.ResetPrevious();
}
