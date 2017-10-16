/**
 ********************************************************************
 * @file    diff.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    23-May-2014
 * @brief   Diff filter definition file.
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

#ifndef DIFF_H
#define DIFF_H

#include <inttypes.h>

 /** @addtogroup Libausbee
   * @{
   */

   /** @addtogroup Control_System
	 * @brief Control engineering module
	 * @{
	 */

	 /** @addtogroup Filters
	   * @brief Filters for the control engineering module
	   * @{
	   */

	   /** @addtogroup Diff
		 * @brief Diff filter
		 * @{
		 */

		 /**
		  * @struct DiffFilter
		  * @brief Diff filter structure
		  *
		  * DiffFilter contains all the parameters and status of the diff
		  * filter.
		  *
		  */
class DiffFilter 
{
public:
	void Init();
	void SetDelta(float delta);
	static float Evaluate(void *diff, float in);

private:
	float   m_prev_in;    /*!< Previous input value. */
	float   m_delta;      /*!< Diff factor. */
	bool	m_first_call; /*!< Whether ausbee_diff_eval has already be called or not. */
};


#endif /* DIFF_H */

/**
  * @}
  */

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
