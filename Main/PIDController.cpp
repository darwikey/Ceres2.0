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

#include "Globals.h"
#include "ControlSystem.h"
/**
* @brief PIDController structure initialisation.
*
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

	m_max_output = 1e10f;
}

/**
 * @brief Define bounds for pid output
 *
 * @param max_output Maximum saturation output value.
 *
 */
void PIDController::SetOutputRange(float max_output)
{
	m_max_output = max_output;
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

float PIDController::GetKD()
{
	return m_Kd;
}

/**
  * @param error Current computed error value.
  *
  * @return Output value of the controller (i.e. the command).
  *
  */
float PIDController::EvaluatePID(float error)
{
	m_error_diff = error - m_last_error;

	float output = m_Kp * error + m_Kd * m_error_diff;
	float tempErrorI = m_Ki * CONTROL_SYSTEM_PERIOD_S *(m_error_sum + error);
	float tempOutput = output + tempErrorI;

	// anti windup
	if (!(abs(25.f * tempOutput) > m_max_output && tempOutput * tempErrorI > 0.f))
	{
		m_error_sum += error;
	}
	output += m_Ki * CONTROL_SYSTEM_PERIOD_S * m_error_sum;

	m_last_error = error;

	output = Math::Clamp(output, -m_max_output, m_max_output);
	return output;
}

/**
  * @fn float GetError()
  * @brief Get last computed error
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
  * @return Error diff value
  *
  */
float PIDController::GetErrorDiff()
{
	return m_error_diff;
}
