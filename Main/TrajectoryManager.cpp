#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "WProgram.h"

#if 0
	#define TRAJ_DEBUG(msg) Serial.println(msg)
#else
	#define TRAJ_DEBUG(msg) ((void)0)
#endif

TrajectoryManager TrajectoryManager::Instance;

#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) ((x)*(x))
#define UNDEFINED_ANGLE (NAN)
#define IS_UNDEFINED_ANGLE(x) (isnan(x))


/******************** User functions ********************/

void TrajectoryManager::Init()
{
	m_CurId = 0;
	m_LastId = 0;
	m_Pause = false;
}

void TrajectoryManager::Reset()
{
	ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm());
	ControlSystem::Instance.SetRadAngleRef(PositionManager::Instance.GetAngleRad());
	m_CurId = m_LastId;
}

int TrajectoryManager::IsEnded()
{
	return (m_CurId == m_LastId);
}

void TrajectoryManager::NextPoint()
{
	TRAJ_DEBUG("NextPoint");
	/* Update list pointer if not empty */
	if (!IsEnded()) {
		m_CurId = (m_CurId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
	}
}

uint32_t TrajectoryManager::GetCurId()
{
	return m_CurId;
}

uint32_t TrajectoryManager::GetLastId()
{
	return m_LastId;
}

/******************** Movement functions ********************/

bool TrajectoryManager::IsPaused()
{
	return m_Pause;
}

void TrajectoryManager::Pause()
{
	m_Pause = true;
}

void TrajectoryManager::Resume()
{
	m_Pause = false;
}

void TrajectoryManager::GotoXY(const Float2 &_pos_mm)
{
	// if it's the first point, we turn the robot face to the next point
	if (TrajectoryManager::IsEnded()) {
		float start_angle = atan2f(-(_pos_mm.x - PositionManager::Instance.GetXMm()), _pos_mm.y - PositionManager::Instance.GetYMm());
		TrajectoryManager::GotoRadianAngle(start_angle);
	}

	TrajDest dest;

	dest.pos = _pos_mm;
	dest.a = UNDEFINED_ANGLE;
	dest.movement = COMMON;

	AddPoint(dest, END);
}

void TrajectoryManager::GotoDistance(float d) {
	float angle = PositionManager::Instance.GetAngleRad();
	Float2 v (d * cos(angle), -d * sin(angle));

	TrajDest dest;
	dest.pos = v + PositionManager::Instance.GetPosMm();
	dest.a = UNDEFINED_ANGLE;

	if (d >= 0.f)
		dest.movement = FORWARD;
	else
		dest.movement = BACKWARD;

	//TrajectoryManager::smooth_traj_goto_xy_mm(dest.x, dest.y);

	AddPoint(dest, END);
}

void TrajectoryManager::GotoDegreeAngle(float a) {
	// convert to radian
	a *= 0.01745329f;

	GotoRadianAngle(a);
}

void TrajectoryManager::GotoRadianAngle(float a)
{
	TrajDest dest;

	dest.pos = PositionManager::Instance.GetPosMm();
	dest.a = a;
	dest.movement = COMMON;

	AddPoint(dest, END);
}


/****************** Internal functions ******************/

void TrajectoryManager::Task()
{
	Update();
}

bool TrajectoryManager::TrajIsFull()
{
	return (((m_LastId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS) == m_CurId);
}

void TrajectoryManager::DecreaseId(uint32_t *id)
{
	*id = (*id + SMOOTH_TRAJ_MAX_NB_POINTS - 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
}

void TrajectoryManager::AddPoint(TrajDest point, TrajWhen when)
{

	if (when == END) {
		if (TrajIsFull()) {
			printf("[TrajectoryManager] Warning: List of points is full. Last point not added.\n");
			return;
		}

		/* New points are added at the end of the list */
		m_Points[m_LastId] = point;

		/* Update end of list pointer */
		m_LastId = (m_LastId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
	}
	else if (when == NOW) {
		if (TrajIsFull()) {
			DecreaseId(&(m_LastId));
			printf("[TrajectoryManager] Warning: List of points is full. Last point was removed.\n");
		}

		/* Insert a point before the current one */
		DecreaseId(&(m_CurId));
		m_Points[m_CurId] = point;
	}
}


// work in progress
void TrajectoryManager::Update()
{
	/* Nothing to do if there is no point in the list */
	if (m_CurId == m_LastId) {
		return;
	}

	// reference to the next waypoint
	TrajDest* next1 = m_Points + m_CurId;
	float next1_dist = (PositionManager::Instance.GetPosMm() - next1->pos).Length();
	// position the robot want to reach
	Float2 target;

	// it's a rotation
	if (!IS_UNDEFINED_ANGLE(next1->a))
	{
		TRAJ_DEBUG("Simple rot");
		if (ABS(next1->a - PositionManager::Instance.GetAngleRad()) < SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD) {
			NextPoint();
		}
		ControlSystem::Instance.SetRadAngleRef(next1->a);
	}
	else
	{
#ifdef TRAJ_STEERING
		uint32_t next_id = (m_CurId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;

		// at least two waypoints
		if (next_id != m_LastId)
		{
			TRAJ_DEBUG("At least two waypoints");
			// find the second waypoint
			TrajDest* next2 = m_Points + next_id;

			float d = SMOOTH_TRAJ_STEER_DISTANCE_MM;
			d -= next1_dist;

			if (d > 0.f)
			{
				float direction_x = next2->x - next1->x;
				float direction_y = next2->y - next1->y;
				float direction_length = sqrt(SQUARE(direction_x) + SQUARE(direction_y));
				direction_x = direction_x / direction_length * d;
				direction_y = direction_y / direction_length * d;

				target_x = next1->x + direction_x;
				target_y = next1->y + direction_y;

				if (d > 0.5f * SMOOTH_TRAJ_STEER_DISTANCE_MM)
				{
					NextPoint();
				}
			}
			else // the next waypoint is too far, so it's our target
			{
				target_x = next1->x;
				target_y = next1->y;
			}

			// if the robot go away the current waypoint, we switch to the next
			/*if (next1_dist > m_PreviousWaypointDist + 35.f)
			{
				Serial.printf("next1 dist %f, prev waypoint dist %f\r\n", next1_dist, m_PreviousWaypointDist);
				NextPoint();
			}*/

		}
		// only one more waypoint
		else
#endif
		{
			TRAJ_DEBUG("one more waypoint");
			target = next1->pos;

			// Waypoint reachs
			if (next1_dist < SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM)
			{
				NextPoint();
			}
		}

		//Serial.printf("pos: %f, %f  target: %f, %f\r\n", PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm(), target_x, target_y);
		GotoTarget(next1, target);
	}
}

float WrapAngle(float a)
{
	a += M_PI;
	return a - floor(a / M_TWOPI) * M_TWOPI - M_PI;
}

void TrajectoryManager::GotoTarget(const TrajDest* _nextPoint, const Float2 &_target)
{
	// Compute the angle and distance to send to the control system
	//float angle_ref, remaining_dist;
	// if the robot is close to the end, and he has to have a specified angle
	/*if ((next_point_distance < SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM && !IS_UNDEFINED_ANGLE(next_point->a))
	&& (next_id == m_LastId || (next_id != m_LastId && ABS(next_point->a - position_get_angle_rad()) > SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD)))
	{
	angle_ref = next_point->a;
	remaining_dist = 0.f;
	}
	else*/

	float AngleRef = atan2f(-(_target.x - PositionManager::Instance.GetXMm()), _target.y - PositionManager::Instance.GetYMm());
	float RemainingDist = (PositionManager::Instance.GetPosMm() - _target).Length();

	float DiffAngle = WrapAngle(AngleRef - PositionManager::Instance.GetAngleRad());
	AngleRef = DiffAngle + PositionManager::Instance.GetAngleRad();

	// only apply a rotation if the difference of angle is too important
	if (fabs(AngleRef - PositionManager::Instance.GetAngleRad()) > 0.5f)
	{
		RemainingDist = 0.f;
	}

	//printf("tar x:%d  y:%d  dist:%f  a:%f\r\n", (int)target_x, (int)target_y, (double)next1_dist, (double)angle_ref);

	// go backward
	if (_nextPoint->movement != COMMON)
	{
		// just asserv in distance
		if (_nextPoint->movement == BACKWARD) {
			ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() - RemainingDist);
		}
		else if (_nextPoint->movement == FORWARD) {
			ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() + RemainingDist);
		}
	}
	else
	{
		ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() + RemainingDist);
		ControlSystem::Instance.SetRadAngleRef(AngleRef);
	}
}