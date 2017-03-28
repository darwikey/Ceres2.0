#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "WProgram.h"

TrajectoryManager TrajectoryManager::Instance;


#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) ((x)*(x))
#define DISTANCE(x1, y1, x2, y2) (sqrtf(SQUARE((x1-x2)) + SQUARE((y1-y2))))
#define UNDEFINED_ANGLE (NAN)
#define IS_UNDEFINED_ANGLE(x) (isnan(x))



/******************** User functions ********************/

void TrajectoryManager::Init()
{
	m_CurId = 0;
	m_LastId = 0;
	m_PreviousWaypointDist = 1.E20f;
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
	/* Update list pointer if not empty */
	if (!IsEnded()) {
		m_CurId = (m_CurId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;
		m_PreviousWaypointDist = 1e20f;
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

void TrajectoryManager::GotoXY(float x, float y)
{
	// if it's the first point, we turn the robot face to the next point
	if (TrajectoryManager::IsEnded()) {
		float start_angle = atan2f(-(x - PositionManager::Instance.GetXMm()), y - PositionManager::Instance.GetYMm());
		TrajectoryManager::GotoRadianAngle(start_angle);
	}

	TrajDest dest;

	dest.x = x;
	dest.y = y;
	dest.a = UNDEFINED_ANGLE;
	dest.movement = COMMON;

	AddPoint(dest, END);
}

void TrajectoryManager::GotoDistance(float d) {
	float angle = PositionManager::Instance.GetAngleRad();
	float y = d * cos(angle);
	float x = -d * sin(angle);

	TrajDest dest;
	dest.x = x + PositionManager::Instance.GetXMm();
	dest.y = y + PositionManager::Instance.GetYMm();
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

	dest.x = PositionManager::Instance.GetXMm();
	dest.y = PositionManager::Instance.GetYMm();
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
	float next1_dist = DISTANCE(PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm(), next1->x, next1->y);
	// position the robot want to reach
	float target_x = 0, target_y = 0;


	uint32_t next_id = (m_CurId + 1) % SMOOTH_TRAJ_MAX_NB_POINTS;

	// it's a rotation
	if (!IS_UNDEFINED_ANGLE(next1->a))
	{
		if (ABS(next1->a - PositionManager::Instance.GetAngleRad()) < SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD) {
			NextPoint();
		}
		ControlSystem::Instance.SetRadAngleRef(next1->a);
	}

	else
	{
		// at least two waypoints
		if (next_id != m_LastId)
		{
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
			}
			else // the next waypoint is too far, so it's our target
			{
				target_x = next1->x;
				target_y = next1->y;
			}

			// if the robot go away the current waypoint, we switch to the next
			if (next1_dist > m_PreviousWaypointDist + 35.f)
			{
				NextPoint();
			}
			// compute the nearest distance to the next point
			if (next1_dist < m_PreviousWaypointDist) {
				m_PreviousWaypointDist = next1_dist;
			}

		}
		// only one more waypoint
		else
		{
			target_x = next1->x;
			target_y = next1->y;

			// Waypoint reachs
			if (next1_dist < SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM)
			{
				NextPoint();
			}
		}

		GotoTarget(next1, target_x, target_y);
	}
}


void TrajectoryManager::GotoTarget(TrajDest* next_point, float target_x, float target_y)
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

	float angle_ref = atan2f(-(target_x - PositionManager::Instance.GetXMm()), target_y - PositionManager::Instance.GetYMm());
	float remaining_dist = DISTANCE(PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm(), target_x, target_y);


	//printf("tar x:%d  y:%d  dist:%f  a:%f\r\n", (int)target_x, (int)target_y, (double)next1_dist, (double)angle_ref);

	// go backward
	if (next_point->movement != COMMON)
	{
		// just asserv in distance
		if (next_point->movement == BACKWARD) {
			ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() - remaining_dist);
		}
		else if (next_point->movement == FORWARD) {
			ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() + remaining_dist);
		}
	}
	else
	{
		ControlSystem::Instance.SetDistanceRef(PositionManager::Instance.GetDistanceMm() + remaining_dist);
		ControlSystem::Instance.SetRadAngleRef(angle_ref);
	}
}