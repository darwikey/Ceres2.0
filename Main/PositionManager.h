/**
 * @file    position_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Definition file.
 */

#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include "QuadDecode.h"
#include "Float2.h"

class PositionManager
{
public:
	static PositionManager Instance;

	void Init(uint32_t ticks_per_m, double axle_track_mm);
	void Update();

	void SetAxleTrackMm(double axle_track_mm);
	double GetAxleTrackMm(void);

	int32_t GetLeftEncoder(void);
	int32_t GetRightEncoder(void);

	float GetDistanceMm(void);
	float GetAngleRad(void);
	float GetAngleDeg(void);
	void SetAngleDeg(float a);
	float GetXMm(void);
	float GetYMm(void);
	Float2 GetPosMm();
	void SetPosMm(const Float2 &_pos);

	int32_t MmToTicks(float value_mm);

	QuadDecode<1> m_Encoder1;  // Template using FTM1
	QuadDecode<2> m_Encoder2;  // Template using FTM2

private:
	uint32_t m_TicksPerM;
	double m_AxleTrackMm; // ecart en mm entre les deux encodeurs

	int32_t m_LeftEncoder, m_RightEncoder;

	double m_DistanceMm;
	double m_AngleRad;
	double m_XMm, m_YMm;
};

#endif /* POSITION_MANAGER_H */
