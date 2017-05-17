/**
 ********************************************************************
 * @file    control_system_manager.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.3
 * @date    23-May-2014
 * @brief   FilteredController system manager implementation file.
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
//#include <stdlib.h>
#include "FilteredController.h"
#include "WProgram.h"

#define AUSBEE_DEBUG_PRINTF 0

#if AUSBEE_DEBUG_PRINTF == 1
#include <stdio.h>
#define debug_printf(args...) do { static int count = 0; if (!((count++) & 0x7F)) Serial.printf(args); } while(0)
#else
#define debug_printf(args,...) ((void)0)
#endif

 /** @addtogroup Libausbee
   * @{
   */

   /** @addtogroup Control_System
	 * @brief Control engineering module
	 * @{
	 */

	 /** @addtogroup FilteredController
	   * @brief FilteredController for the control engineering module.
	   * @{
	   */

static float safe_filter(float(*f)(void *, float), void *params, float value)
{
	if (f) {
		return f(params, value);
	}
	return value;
}

/**
 * @fn void Init()
 * @brief FilteredController structure initialisation.
 *
 * @param cs The structure to initialise.
 *
 */
void FilteredController::Init()
{
	m_target_filter = nullptr;
	m_target_filter_params = nullptr;

	m_measure_fetcher = nullptr;

	m_measure_filter = nullptr;
	m_measure_filter_params = nullptr;

	m_controller = nullptr;
	m_controller_params = nullptr;

	m_process_command = nullptr;

	m_target = 0;
	m_filtered_target = 0;
	m_measure = 0;
	m_filtered_measure = 0;
	m_error = 0;
	m_command = 0;
}

/**
 * @fn void SetTargetFilter(,
 *         float (*reference_filter)(void *, float),
 *         void * reference_filter_params)
 * @brief Setting a function to filter the reference value used by the
 *        control system.
 *
 * @param cs                    Control system structure reference.
 * @param reference_filter        Function to filter the reference value.
 * @param reference_filter_params Parameters for the function.
 *
 */
void FilteredController::SetTargetFilter(float(*filter)(void *, float), void * filter_params)
{
	m_target_filter = filter;
	m_target_filter_params = filter_params;
}

/**
 * @fn void SetMeasureFetcher(,
 *         float (*measure_fetcher)(void *),
 *         void * measure_fetcher_params)
 * @brief Setting a function to get measure value used by the
 *        control system.
 *
 * @param cs                     Control system structure reference.
 * @param measure_fetcher        Function to get the measure value.
 *
 */
void FilteredController::SetMeasureFetcher(float(*measure_fetcher)(void))
{
	m_measure_fetcher = measure_fetcher;
}

/**
 * @fn void SetMeasureFilter(,
 *         float (*measure_filter)(void *, float),
 *         void * measure_filter_params)
 * @brief Setting a function to filter the measure value used by the
 *        control system.
 *
 * @param cs                    Control system structure reference.
 * @param measure_filter        Function to filter the measure value.
 * @param measure_filter_params Parameters for the function.
 *
 */
void FilteredController::SetMeasureFilter(float(*measure_filter)(void *, float), void * measure_filter_params)
{
	m_measure_filter = measure_filter;
	m_measure_filter_params = measure_filter_params;
}

/**
 * @fn void SetController(
 *         float (*controller)(void *, float),
 *         void * controller_params)
 * @brief Setting the controller to use in the control system.
 *
 * @param cs                Control system structure reference.
 * @param controller        FilteredController processing function.
 * @param controller_params Parameters for the processing function.
 *
 */
void FilteredController::SetController(float(*controller)(void *, float), void * controller_params)
{
	m_controller = controller;
	m_controller_params = controller_params;
}

/**
 * @fn void SetProcessCommand(
	void (*process_command)(void *, float),
	void * process_command_params)
 * @brief Setting the processing function for the command
 *        computed by the control system.
 *
 * @param cs                     Control system structure reference.
 * @param process_command        Command processing function.
 *
 */
void FilteredController::SetProcessCommand(void(*process_command)(float))
{
	m_process_command = process_command;
}

/**
  * @fn float Update()
  * @brief Process the control loop to compute the command.
  *
  * @return Command value
  */
float FilteredController::Update()
{
	debug_printf("[csm] Input target: %d\r\n", (int)m_target);

	m_filtered_target = safe_filter(m_target_filter, m_target_filter_params, m_target);
	debug_printf("[csm] Filtered target: %d\r\n", (int)m_filtered_target);

	m_measure = m_measure_fetcher();
	debug_printf("[csm] Measure: %d\r\n", (int)m_measure);

	m_filtered_measure = safe_filter(m_measure_filter, m_measure_filter_params, m_measure);
	debug_printf("[csm] Filtered Measure: %d\r\n", (int)m_filtered_measure);

	m_error = m_filtered_target - m_filtered_measure;
	debug_printf("[csm] Error: %d\r\n", (int)m_error);

	m_command = m_controller(m_controller_params, m_error);
	debug_printf("[csm] Controller output command: %d\r\n\r\n", (int)m_command);

	m_process_command(m_command);

	return m_command;
}

/**
 * @fn float GetTarget()
 * @brief Getting the reference we want to reach.
 *
 * @return Reference value.
 *
 */
float FilteredController::GetTarget()
{
	return m_target;
}

/**
 * @fn float GetFilteredTarget()
 * @brief Getting the filtered reference.
 *
 * @return Filtered reference value.
 *
 */
float FilteredController::GetFilteredTarget()
{
	return m_filtered_target;
}

/**
 * @fn float GetMeasure()
 * @brief Getting the measure.
 *
 * @return Measure value.
 *
 */
float FilteredController::GetMeasure()
{
	return m_measure;
}

/**
 * @fn float GetFilteredMeasure()
 * @brief Getting the filtered measure.
 *
 * @return Filtered measure value.
 *
 */
float FilteredController::GetFilteredMeasure()
{
	return m_filtered_measure;
}

/**
 * @fn float GetError()
 * @brief Getting the computed error.
 *
 * @return Error value.
 *
 */
float FilteredController::GetError()
{
	return m_error;
}

/**
 * @fn float GetCommand()
 * @brief Getting the computed command.
 *
 * @return Command value.
 *
 */
float FilteredController::GetCommand()
{
	return m_command;
}

/**
 * @fn void SetTarget(float ref)
 * @brief Setting the reference we want to reach.
 *
 * @param ref Reference value.
 *
 */
void FilteredController::SetTarget(float ref)
{
	m_target = ref;
}

/**
  * @}
  */

  /**
	* @}
	*/

	/**
	  * @}
	  */

	  /************** (C) COPYRIGHT 2013-2014 Eirbot **** END OF FILE ****/
