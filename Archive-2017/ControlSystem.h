/**
 * @file    control_system.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Definition file.
 */

#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "PIDController.h"
#include "DiffFilter.h"
#include "QuadrampFilter.h"
#include "Globals.h"

#define CONTROL_SYSTEM_PERIOD_S 0.01 // in s

#define DISTANCE_MAX_SPEED 180//240 // in mm/s
#define DISTANCE_MAX_ACC   300 // in mm/s^2

#define ANGLE_MAX_SPEED_DEG 90 // in deg/s
#define ANGLE_MAX_ACC_DEG   180 // in deg/s^2

class ControlSystem
{
public:
	static ControlSystem Instance;

	void Start();
	void Task();

	void SetDistanceTarget(float ref_mm);
	void SetDegAngleTarget(float ref);
	void SetRadAngleTarget(float ref_rad);
	void SetRightMotorTarget(int32_t ref);
	void SetMotorTarget(int32_t ref);

	void SetDistanceMaxSpeed(float max_speed);
	void SetDistanceMaxAcc(float max_acc);
	void SetAngleMaxSpeed(float max_speed);
	void SetAngleMaxAcc(float max_acc);

	void SetSpeedRatio(float ratio);
	void SetSpeedHigh();
	void SetSpeedMedium();
	void SetSpeedLow();

	PIDController& GetDistancePID();
	PIDController& GetAnglePID();

	void Reset();
	void ResetAngle();

	bool m_Enable = true;
	int m_DebugInterval = -1;

private:
	void SetMotorCmd(float d_mm, float theta);
	void Debug(const char *msg, float value);
	
	float m_DistanceTarget;
	float m_AngleTarget;

	PIDController m_pid_distance;
	PIDController m_pid_angle;

	QuadrampFilter m_quadramp_distance;
	QuadrampFilter m_quadramp_angle;

	uint32_t m_MotorCounter = 0;
	Float2 m_LastPosition;
	int m_DebugCounter = 0;
};


#endif /* CONTROL_SYSTEM_H */
