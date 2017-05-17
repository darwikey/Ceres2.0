/**
 * @file    position_manager.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Implementation file.
 */

#include "PositionManager.h"
#include "ControlSystem.h"
#include "Scheduler.h"
#include <math.h>

PositionManager PositionManager::Instance;

// FTM Interrupt Service routines - on overflow and position compare
void ftm1_isr(void)
{
	PositionManager::Instance.m_Encoder1.ftm_isr();
}

void ftm2_isr(void)
{
	PositionManager::Instance.m_Encoder2.ftm_isr();
}

#define position_ticks_to_mm(value_ticks) ((value_ticks) * 1000.0 / m_TicksPerM)

void PositionManager::Init(uint32_t ticks_per_m, double axle_track_mm) {
	m_Encoder1.setup();
	m_Encoder2.setup();
	m_Encoder1.start();
	m_Encoder2.start();

	m_TicksPerM = ticks_per_m;
	m_AxleTrackMm = axle_track_mm;

	m_LeftEncoder = 0;
	m_RightEncoder = 0;

	m_DistanceMm = 0;
	m_AngleRad = 0;

	m_XMm = 0;
	m_YMm = 0;
}

void PositionManager::Update()
{
	// Reading encoder value
	int32_t new_left_enc = -m_Encoder1.calcPosn();
	int32_t new_right_enc = m_Encoder2.calcPosn();

	int32_t left_enc_diff = m_LeftEncoder - new_left_enc;
	int32_t right_enc_diff = m_RightEncoder - new_right_enc;

	m_LeftEncoder = new_left_enc;
	m_RightEncoder = new_right_enc;

#if 0
	if (left_enc_diff != 0) {
		platform_led_toggle(PLATFORM_LED2);
		//printf("G.");//encG : %d   (sumG : %d)\n\r", (int)left_enc_diff, (int)m_LeftEncoder);
	}

	if (right_enc_diff != 0) {
		platform_led_toggle(PLATFORM_LED3);
		//printf("D.");//encD : %d   (sumD : %d)\n\r", (int)right_enc_diff, (int)m_RightEncoder);
	}
#endif

	// Distance
	double distance_diff_ticks = (left_enc_diff + right_enc_diff) / 2.0;
	double distance_diff_mm = position_ticks_to_mm(distance_diff_ticks);
	m_DistanceMm += distance_diff_mm;

	/*if (right_enc_diff != 0){
		printf("dist %d\n", (int)m_DistanceMm);
	}*/

	// Special case: no rotation
	if ((right_enc_diff - left_enc_diff) == 0) {
		m_XMm += -distance_diff_mm * sin(m_AngleRad);
		m_YMm += distance_diff_mm * cos(m_AngleRad);
		return;
	}

	double angle_diff_rad = atan(position_ticks_to_mm(right_enc_diff - left_enc_diff) / m_AxleTrackMm);

	// Special case: only rotation -> no need to update x and y
	if ((right_enc_diff + left_enc_diff) == 0) {
		m_AngleRad += angle_diff_rad;
		return;
	}

	// Radius of curvature
	double r = m_AxleTrackMm / 2.0 * (right_enc_diff + left_enc_diff) / (right_enc_diff - left_enc_diff);

	// Trajectory circle center coordinates
	double x0_mm = m_XMm - r * cos(m_AngleRad);
	double y0_mm = m_YMm - r * sin(m_AngleRad);

	// Update position
	m_AngleRad += angle_diff_rad;
	m_XMm = x0_mm + r * cos(m_AngleRad);
	m_YMm = y0_mm + r * sin(m_AngleRad);
}

void PositionManager::SetAxleTrackMm(double axle_track_mm)
{
	m_AxleTrackMm = axle_track_mm;
}

double PositionManager::GetAxleTrackMm(void) {
	return m_AxleTrackMm;
}

float PositionManager::GetLeftEncoder(void) {
	return m_LeftEncoder;
}

float PositionManager::GetRightEncoder(void) {
	return m_RightEncoder;
}

float PositionManager::GetDistanceMm(void) {
	Scheduler::disable();
	float r = m_DistanceMm;
	Scheduler::enable();
	return r;
}

float PositionManager::GetAngleRad(void) {
	Scheduler::disable();
	float r = m_AngleRad;
	Scheduler::enable();
	return r;
}

float PositionManager::GetAngleDeg(void) {
	return GetAngleRad() * 180.f / M_PI;
}

void PositionManager::SetAngleDeg(float a) {
	m_AngleRad = a * M_PI / 180.f;
	ControlSystem::Instance.SetRadAngleTarget(m_AngleRad);
	ControlSystem::Instance.ResetAngle();
}

float PositionManager::GetXMm(void) {
	Scheduler::disable();
	float r = m_XMm;
	Scheduler::enable();
	return r;
}

float PositionManager::GetYMm(void) {
	Scheduler::disable();
	float r = m_YMm;
	Scheduler::enable();
	return r;
}

Float2 PositionManager::GetPosMm()
{
	Scheduler::disable();
	Float2 r(m_XMm, m_YMm);
	Scheduler::enable();
	return r;
}

void PositionManager::SetPosMm(const Float2 &_pos){
	Scheduler::disable();
	m_XMm = _pos.x;
	m_YMm = _pos.y;
	Scheduler::enable();
}

int32_t PositionManager::MmToTicks(float value_mm) {
	return value_mm * m_TicksPerM / 1000.f;
}
