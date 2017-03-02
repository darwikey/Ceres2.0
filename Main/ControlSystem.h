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

#include "FilteredController.h"
#include "PIDController.h"
#include "diff.h"
#include "QuadrampFilter.h"

#define CONTROL_SYSTEM_PERIOD_S 0.01 // in s

#define DISTANCE_MAX_SPEED 240 // in mm/s
#define DISTANCE_MAX_ACC   300 // in mm/s^2

#define ANGLE_MAX_SPEED_DEG 90 // in deg/s
#define ANGLE_MAX_ACC_DEG   180 // in deg/s^2

class ControlSystem
{
public:
	static ControlSystem Instance;

	void Start();
	void Task();

	void SetDistanceRef(float ref_mm);
	void SetDegAngleRef(float ref);
	void SetRadAngleRef(float ref_rad);
	void SetRightMotorRef(int32_t ref);
	void SetLeftMotorRef(int32_t ref);

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

	void ResetAngle();

private:
	void SetMotorsRef(float d_mm, float theta);
	static void SetDistanceMmDiff(float ref);
	static void SetAngleRadDiff(float ref);

	FilteredController m_csm_right_motor;
	FilteredController m_csm_left_motor;
	FilteredController m_csm_distance;
	FilteredController m_csm_angle;

	/*ausbee_diff diff_right_motor;
	ausbee_diff diff_left_motor;*/

	PIDController m_pid_right_motor;
	PIDController m_pid_left_motor;
	PIDController m_pid_distance;
	PIDController m_pid_angle;

	QuadrampFilter m_quadramp_distance;
	QuadrampFilter m_quadramp_angle;

	float m_distance_mm_diff;
	float m_angle_rad_diff;
};


#endif /* CONTROL_SYSTEM_H */
