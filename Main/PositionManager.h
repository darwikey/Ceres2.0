/**
 * @file    position_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Definition file.
 */

#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <stdint.h>

class PositionManager
{
public:
	static PositionManager Instance;

	void Init(uint32_t ticks_per_m, double axle_track_mm);
	void Update();

	void SetAxleTrackMm(double axle_track_mm);
	double GetAxleTrackMm(void);

	float GetLeftEncoder(void);
	float GetRightEncoder(void);

	float GetDistanceMm(void);
	float GetAngleRad(void);
	float GetAngleDeg(void);
	void SetAngleDeg(float a);
	float GetXMm(void);
	float GetYMm(void);
	void SetXYMm(float x, float y);

	int32_t MmToTicks(float value_mm);

private:
	uint32_t m_TicksPerM;
	double m_AxleTrackMm; // ecart en mm entre les deux encodeurs

	int32_t m_LeftEncoder, m_RightEncoder;

	double m_DistanceMm;
	double m_AngleRad;
	double m_XMm, m_YMm;
};

#endif /* POSITION_MANAGER_H */
