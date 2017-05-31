/**
 ********************************************************************
 * @file    pid.c
 * @author  Kevin JOLY
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    14-Mar-2014
 * @brief   PID controller implementation file. Contain controllers
 *          for control engineering.
 ********************************************************************
 * @attention
 *
 * This file is part of LIBAUSBEE.
 *
 * LIBAUSBEE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LIBAUSBEE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LIBAUSBEE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * <h2><centor>&copy;  Copyright 2013-2014 (C) EIRBOT </center></h2>
 ********************************************************************
 */
#include "PIDController.h"

#include <math.h>
/**
* @fn void Init(float Kp, float Ki, float Kd)
* @brief PIDController structure initialisation.
*
* @param pid Structure reference.
* @param Kp Proportional value.
* @param Ki Integral value.
* @param Kd Derivative value.
*
*/
void PIDController::Init(float Kp, float Ki, float Kd)
{
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;

	m_last_error = 0;
	m_error_sum = 0;
	m_error_diff = 0;

	m_min_output = -1e10f;
	m_max_output = 1e10f;

	m_error_deadband = 0;
}

/**
 * @fn void SetOutputRange(float min_output, float max_output)
 * @brief Define bounds for pid output
 *
 * @param pid Structure reference.
 * @param min_output Minimum saturation output value.
 * @param max_output Maximum saturation output value.
 *
 */
void PIDController::SetOutputRange(float min_output, float max_output)
{
	m_min_output = min_output;
	m_max_output = max_output;
}

/**
 * @fn void SetErrorDeadband(float error_deadband)
 * @brief Define a deadband for pid error
 *
 * @param pid Structure reference.
 * @param error_deadband Error within [-error_deadband, error_deadband] is considered equaling zero.
 *                       Useful to avoid some problems resulting from using floats.
 *                       Must be a positive number.
 *
 */
void PIDController::SetErrorDeadband(float error_deadband)
{
	m_error_deadband = error_deadband;
}

void PIDController::SetKP(float Kp)
{
	m_Kp = Kp;
}

void PIDController::SetKI(float Ki)
{
	m_error_sum = 0;
	m_Ki = Ki;
}

void PIDController::SetKD(float Kd)
{
	m_Kd = Kd;
}

float PIDController::GetKP()
{
	return m_Kp;
}

float PIDController::GetKI()
{
	return m_Ki;
}

float PIDController::GetKd()
{
	return m_Kd;
}

/**
  * @fn float EvaluatePID(void *pid, float error)
  * @brief Compute PID control from the last error.
  *
  * @param pid Generic structure reference.
  * @param error Current computed error value.
  *
  * @return Output value of the controller (i.e. the command).
  *
  */
float PIDController::EvaluatePID(float error)
{
	if ((error < m_error_deadband) && (error > -m_error_deadband))
		error = 0;

	m_error_sum += error;
	m_error_diff = error - m_last_error;

	float output = m_Kp * error + m_Ki * m_error_sum + m_Kd * m_error_diff;

	m_last_error = error;

	if (output > m_max_output) {
		output = m_max_output;
	}

	if (output < m_min_output) {
		output = m_min_output;
	}

	return output;
}

/**
  * @fn float GetError()
  * @brief Get last computed error
  *
  * @param pid Structure reference.
  *
  * @return Error value
  *
  */
float PIDController::GetError()
{
	return m_last_error;
}

/**
  * @fn float GetErrorSum()
  * @brief Get last computed error_sum
  *
  * @param pid Structure reference.
  *
  * @return Error sum value
  *
  */
float PIDController::GetErrorSum()
{
	return m_error_sum;
}

/**
  * @fn float GetErrorDiff()
  * @brief Get last computed error_diff
  *
  * @param pid Structure reference.
  *
  * @return Error diff value
  *
  */
float PIDController::GetErrorDiff()
{
	return m_error_diff;
}
