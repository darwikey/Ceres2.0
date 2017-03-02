/**
 ********************************************************************
 * @file    control_system_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.3
 * @date    23-May-2014
 * @brief   FilteredController system manager definition file.
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
#ifndef CONTROLLER_H
#define CONTROLLER_H

 /** @addtogroup Libausbee
   * @{
   */

   /** @defgroup Control_System
	 * @brief Control engineering module
	 * @{
	 */

	 /** @defgroup FilteredController
	   * @brief FilteredController for the control engineering module.
	   * @{
	   */

	   /**
		* @struct FilteredController
		* @brief Control system structure
		*
		* FilteredController contains all the parameters and status of the control system
		* manager.
		*
		*/
class FilteredController
{
public:
	void Init();

	void SetReferenceFilter(float(*reference_filter)(void *, float), void * reference_filter_params);

	void SetMeasureFetcher( float(*measure_fetcher)(void));

	void SetMeasureFilter(float(*measure_filter)(void *, float), void * measure_filter_params);

	void SetController(float(*controller)(void *, float), void * controller_params);

	void SetProcessCommand(void(*process_command)(float));

	float Update();

	float GetReference();
	float GetFilteredReference();
	float GetMeasure();
	float GetFilteredMeasure();
	float GetError();
	float GetCommand();

	void SetReference(float ref);

private:
	float(*m_reference_filter)(void *, float);
	void * m_reference_filter_params;

	float(*m_measure_fetcher)(void);

	float(*m_measure_filter)(void *, float);
	void * m_measure_filter_params;

	float(*m_controller)(void *, float);
	void * m_controller_params;

	void(*m_process_command)(float);

	float m_reference;          /*!< The value we want to reach. */
	float m_filtered_reference; /*!< The value we think we can reach now. */
	float m_measure;            /*!< Last measured value. */
	float m_filtered_measure;   /*!< Last measured value filtered. */
	float m_error;              /*!< Last computed error. */
	float m_command;            /*!< Last command computed by the controller. */
};



#endif

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
