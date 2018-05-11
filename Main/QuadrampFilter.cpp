/**
 ********************************************************************
 * @file    QuadrampFilter.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.1
 * @date    24-May-2014
 * @brief   Quadramp filter implementation file.
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
#include <string.h>
#include <math.h>

#include "QuadrampFilter.h"

#define SQUARE(x) ((x) * (x))


void QuadrampFilter::Init()
{
	m_var_2nd_ord_pos = 0;
	m_var_2nd_ord_neg = 0;
	m_var_1st_ord_pos = 0;
	m_var_1st_ord_neg = 0;

	m_prev_var = 0;
	m_prev_out = 0;
	m_eval_period = 1;
	m_Enable = true;
}

void QuadrampFilter::SetEvalPeriod(float period)
{
	m_eval_period = period;
}

void QuadrampFilter::Set2ndOrderVars(float var_2nd_ord_pos, float var_2nd_ord_neg)
{
	m_var_2nd_ord_pos = var_2nd_ord_pos;
	m_var_2nd_ord_neg = var_2nd_ord_neg;
}

void QuadrampFilter::Set1stOrderVars(float var_1st_ord_pos, float var_1st_ord_neg)
{
	m_var_1st_ord_pos = var_1st_ord_pos;
	m_var_1st_ord_neg = var_1st_ord_neg;
}

void QuadrampFilter::Reset(float value)
{
	m_prev_out = value;
	m_prev_var = 0;
}

/* TODO: handle float equality */
//bool QuadrampFilter::ausbee_quadramp_is_finished()
//{
//	return (m_prev_out == m_prev_in && m_prev_var == 0);
//}

float QuadrampFilter::Evaluate(float in)
{
	if (!m_Enable)
	{
		Reset(in);
		return in;
	}

	float var_1st_ord_pos = m_var_1st_ord_pos * m_eval_period;

	float var_1st_ord_neg = -m_var_1st_ord_neg * m_eval_period;

	float var_2nd_ord_pos = m_var_2nd_ord_pos * SQUARE(m_eval_period);

	float var_2nd_ord_neg = -m_var_2nd_ord_neg * SQUARE(m_eval_period);

	float prev_var = m_prev_var;
	float prev_out = m_prev_out;

	float d = in - prev_out;

	/* Deceleration ramp */
	if (d > 0 && var_2nd_ord_neg) {
		/* var_2nd_ord_neg < 0 */
		/* real EQ : sqrtf( var_2nd_ord_neg^2/4 - 2.d.var_2nd_ord_neg ) + var_2nd_ord_neg/2 */
		float ramp_pos = sqrtf((var_2nd_ord_neg*var_2nd_ord_neg) / 4 - 2 * d*var_2nd_ord_neg) + var_2nd_ord_neg / 2;

		if (ramp_pos < var_1st_ord_pos)
			var_1st_ord_pos = ramp_pos;
	}

	else if (d < 0 && var_2nd_ord_pos) {

		/* var_2nd_ord_pos > 0 */
		/* real EQ : sqrtf( var_2nd_ord_pos^2/4 - 2.d.var_2nd_ord_pos ) - var_2nd_ord_pos/2 */
		float ramp_neg = -sqrtf((var_2nd_ord_pos*var_2nd_ord_pos) / 4 - 2 * d*var_2nd_ord_pos) - var_2nd_ord_pos / 2;

		/* ramp_neg < 0 */
		if (ramp_neg > var_1st_ord_neg)
			var_1st_ord_neg = ramp_neg;
	}

	/* try to set the speed : can we reach the speed with our acceleration ? */
	/* si on va moins vite que la Vmax */
	if (prev_var < var_1st_ord_pos) {
		/* acceleration would be to high, we reduce the speed */
		/* si rampe acceleration active ET qu'on ne peut pas atteindre Vmax,
		 * on sature Vmax a Vcourante + acceleration */
		if (var_2nd_ord_pos && (var_1st_ord_pos - prev_var > var_2nd_ord_pos))
			var_1st_ord_pos = prev_var + var_2nd_ord_pos;
	}
	/* si on va plus vite que Vmax */
	else if (prev_var > var_1st_ord_pos) {
		/* deceleration would be to high, we increase the speed */
		/* si rampe deceleration active ET qu'on ne peut pas atteindre Vmax,
		 * on sature Vmax a Vcourante + deceleration */
		if (var_2nd_ord_neg && (var_1st_ord_pos - prev_var < var_2nd_ord_neg))
			var_1st_ord_pos = prev_var + var_2nd_ord_neg;
	}

	/* same for the neg */
	/* si on va plus vite que la Vmin (en negatif : en vrai la vitesse absolue est inferieure) */
	if (prev_var > var_1st_ord_neg) {
		/* acceleration would be to high, we reduce the speed */
		/* si rampe deceleration active ET qu'on ne peut pas atteindre Vmin,
		 * on sature Vmax a Vcourante + deceleration */
		if (var_2nd_ord_neg && (var_1st_ord_neg - prev_var < var_2nd_ord_neg))
			var_1st_ord_neg = prev_var + var_2nd_ord_neg;
	}
	/* si on va moins vite que Vmin (mais vitesse absolue superieure) */
	else if (prev_var < var_1st_ord_neg) {
		/* deceleration would be to high, we increase the speed */
		/* si rampe acceleration active ET qu'on ne peut pas atteindre Vmin,
		 * on sature Vmax a Vcourante + deceleration */
		if (var_2nd_ord_pos && (var_1st_ord_neg - prev_var > var_2nd_ord_pos))
			var_1st_ord_neg = prev_var + var_2nd_ord_pos;
	}

	// Position consign : can we reach the position with our speed ?
	float pos_target;
	if (d > var_1st_ord_pos) {
		pos_target = prev_out + var_1st_ord_pos;
		prev_var = var_1st_ord_pos;
	}
	else if (d < var_1st_ord_neg) {
		pos_target = prev_out + var_1st_ord_neg;
		prev_var = var_1st_ord_neg;
	}
	else {
		pos_target = prev_out + d;
		prev_var = d;
	}

	// update prev_out and prev_var
	m_prev_var = prev_var;
	m_prev_out = pos_target;

	return pos_target;
}
