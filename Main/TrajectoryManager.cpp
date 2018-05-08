#include "PositionManager.h"
#include "TrajectoryManager.h"

#if 0
	#define TRAJ_DEBUG(msg) Serial.println(msg)
#else
	#define TRAJ_DEBUG(msg) ((void)0)
#endif

TrajectoryManager TrajectoryManager::Instance;

#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) ((x)*(x))
#define UNDEFINED_ANGLE (1000.f)
#define IS_UNDEFINED_ANGLE(x) (x==UNDEFINED_ANGLE)

float WrapAngle(float a)
{
	a += M_PI;
	return a - floor(a / M_TWOPI) * M_TWOPI - M_PI;
}

float GetAngleRef(const Float2 &_target, float *_diff = nullptr)
{
	float start_angle0 = atan2f(-(_target.x - PositionManager::Instance.GetXMm()), _target.y - PositionManager::Instance.GetYMm());
	float start_angle1 = start_angle0 + (float)M_TWOPI;
	float start_angle2 = start_angle0 - (float)M_TWOPI;

	float diff0 = fabs(PositionManager::Instance.GetAngleRad() - start_angle0);
	float diff1 = fabs(PositionManager::Instance.GetAngleRad() - start_angle1);
	float diff2 = fabs(PositionManager::Instance.GetAngleRad() - start_angle2);
	/*Serial.printf("diff0: %f", diff0);
	Serial.printf("diff1: %f", diff1);
	Serial.printf("diff2: %f", diff2);*/

	if (_diff)
	{
		*_diff = min(diff0, min(diff1, diff2));
	}

	if (diff2 < diff1 && diff2 < diff0)
		return start_angle2;
	if (diff1 < diff0)
		return start_angle1;
	else
		return start_angle0;
}

/******************** User functions ********************/

void TrajectoryManager::Init()
{
	m_Pause = false;
	m_IsOnlyRotation = false;
}

void TrajectoryManager::Reset()
{
	ControlSystem::Instance.SetDistanceTarget(PositionManager::Instance.GetDistanceMm());
	ControlSystem::Instance.SetRadAngleTarget(PositionManager::Instance.GetAngleRad());
	m_Points.Clear();
}

void TrajectoryManager::NextPoint()
{
	TRAJ_DEBUG("NextPoint");
	if (!m_Points.IsEmpty()) {
		const TrajDest& current = m_Points.Front();

		// it's a rotation
		if (!IS_UNDEFINED_ANGLE(current.angle))
		{
			PositionManager::Instance.SetTheoreticalAngleRad(current.angle);
		}
		else
		{
			PositionManager::Instance.SetTheoreticalPosMm(current.pos);
		}
		m_Points.PopFront();
	}
}

void TrajectoryManager::Print()
{
	Serial.printf("Number of points: %d", m_Points.GetSize());
	m_Points.Apply([](const TrajDest &t){
		Serial.printf("Dest: mvt:%d, target:(%f, %f), angle:%f", (int)t.movement, t.pos.x, t.pos.y, t.angle);
	});
	Serial.printf("is paused: %d\r\n", (int)m_Pause);
}

bool TrajectoryManager::IsForwardMovement()
{
	if (!m_Points.IsEmpty())
	{
		return m_Points.Front().movement != BACKWARD;
	}
	return true;
}

bool TrajectoryManager::IsOnlyRotation()
{
	if (!m_Points.IsEmpty())
	{
		const TrajDest& next1 = m_Points.Front();
		if (next1.movement == BACKWARD)
			return false;
		// it's a rotation
		if (!IS_UNDEFINED_ANGLE(next1.angle))
		{
			return true;
		}
		else
		{
			//TODO duplicate with gototarget
			Float2 _target = next1.pos;
			float AngleRef = atan2f(-(_target.x - PositionManager::Instance.GetXMm()), _target.y - PositionManager::Instance.GetYMm());
			float DiffAngle = WrapAngle(AngleRef - PositionManager::Instance.GetAngleRad());
			AngleRef = DiffAngle + PositionManager::Instance.GetAngleRad();

			// only apply a rotation if the difference of angle is too important
			if (fabs(AngleRef - PositionManager::Instance.GetAngleRad()) > 0.5f)
			{
				return true;
			}
		}
	}
	return false;
}

/******************** Movement functions ********************/

bool TrajectoryManager::IsPaused()
{
	return m_Pause;
}

void TrajectoryManager::Pause()
{
	m_Pause = true;
	m_PauseDist = PositionManager::Instance.GetDistanceMm();
	m_pauseAngle = PositionManager::Instance.GetAngleRad();
}

void TrajectoryManager::Resume()
{
	m_Pause = false;
}

void TrajectoryManager::GotoXY(const Float2 &_pos_mm)
{
	// if it's the first point, we turn the robot face to the next point
	if (TrajectoryManager::IsEnded()) {
		
		TrajectoryManager::GotoRadianAngle(GetAngleRef(_pos_mm));
	}

	TrajDest dest;

	dest.pos = _pos_mm;
	dest.angle = UNDEFINED_ANGLE;
	dest.movement = COMMON;

	AddPoint(dest, END);
}

void TrajectoryManager::GotoDistance(float _dist) {

	float finalAngle = PositionManager::Instance.GetTheoreticalAngleRad();
	Float2 finalPos = PositionManager::Instance.GetTheoreticalPosMm();

	Serial.print("goto d\r\n");
	Serial.printf("Robot pos mm:    %f, %f\r\n", PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm());
	Serial.printf("theoretical pos: %f, %f\r\n", PositionManager::Instance.GetTheoreticalPosMm().x, PositionManager::Instance.GetTheoreticalPosMm().y);

	auto fct = [&](const TrajDest &t) {
		// it's a rotation
		if (!IS_UNDEFINED_ANGLE(t.angle))
		{
			finalAngle = t.angle;
		}
		else
		{
			finalPos = t.pos;
		}
	};
	// find the position & angle at the end
	m_Points.Apply(fct);

	//Float2 v (d * cos(angle), -d * sin(angle));
	Float2 v(-_dist * sin(finalAngle), _dist * cos(finalAngle));

	TrajDest dest;
	dest.pos = v + finalPos;
	dest.angle = UNDEFINED_ANGLE;
	//Serial.printf("x %f, y %f\r\n", dest.pos.x, dest.pos.y);

	if (_dist >= 0.f)
		dest.movement = FORWARD;
	else
		dest.movement = BACKWARD;

	AddPoint(dest, END);
}

void TrajectoryManager::GotoDegreeAngle(float a) {
	// convert to radian
	a = DEG2RAD(a);

	GotoRadianAngle(a);
}

void TrajectoryManager::GotoRadianAngle(float a)
{
	TrajDest dest;
	Serial.printf("GotoRadianAngle: %f", a);
	dest.pos = PositionManager::Instance.GetPosMm();
	dest.angle = a;
	dest.movement = COMMON;

	AddPoint(dest, END);
}


/****************** Internal functions ******************/

void TrajectoryManager::Task()
{
	Update();
}

void TrajectoryManager::AddPoint(TrajDest point, TrajWhen when)
{

	if (when == END) {
		if (TrajIsFull()) {
			Serial.printf("[TrajectoryManager] Warning: List of points is full. Last point not added.\n");
			return;
		}

		/* New points are added at the end of the list */
		m_Points.PushBack(point);
	}
	else if (when == NOW) {
		if (TrajIsFull()) {
			m_Points.PopBack();
			Serial.printf("[TrajectoryManager] Warning: List of points is full. Last point was removed.\n");
		}

		/* Insert a point before the current one */
		m_Points.PushFront(point);
	}
}


// work in progress
void TrajectoryManager::Update()
{
	//Serial.printf("only rot %d", (int)m_IsOnlyRotation);
	m_IsOnlyRotation = false;

	/* Nothing to do if there is no point in the list */
	if (m_Points.IsEmpty()) {
		return;
	}

	if (m_Pause)
	{
		ControlSystem::Instance.SetDistanceTarget(m_PauseDist);
		ControlSystem::Instance.SetRadAngleTarget(m_pauseAngle);
		return;
	}

	// reference to the next waypoint
	const TrajDest& next1 = m_Points.Front();
	float next1_dist = (PositionManager::Instance.GetPosMm() - next1.pos).Length();
	// position the robot want to reach
	Float2 target;

	// it's a rotation
	if (!IS_UNDEFINED_ANGLE(next1.angle))
	{
		TRAJ_DEBUG("Simple rot");
		
		if (ABS(next1.angle - PositionManager::Instance.GetAngleRad()) < SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD) {
			NextPoint();
		}
		m_IsOnlyRotation = true;
		ControlSystem::Instance.SetRadAngleTarget(next1.angle);
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
			target = next1.pos;

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

void TrajectoryManager::GotoTarget(const TrajDest &_nextPoint, const Float2 &_target)
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

	float AngleRefDiff;
	float AngleRef = GetAngleRef(_target, &AngleRefDiff);// atan2f(-(_target.x - PositionManager::Instance.GetXMm()), _target.y - PositionManager::Instance.GetYMm());
	float RemainingDist = (PositionManager::Instance.GetPosMm() - _target).Length();

	//printf("tar x:%d  y:%d  dist:%f  a:%f\r\n", (int)_target.x, (int)_target.y, (double)_nextPoint., (double)angle_ref);

	// go backward
	if (_nextPoint.movement != COMMON)
	{
		// just asserv in distance
		if (_nextPoint.movement == BACKWARD) {
			ControlSystem::Instance.SetDistanceTarget(PositionManager::Instance.GetDistanceMm() - RemainingDist);
		}
		else if (_nextPoint.movement == FORWARD) {
			ControlSystem::Instance.SetDistanceTarget(PositionManager::Instance.GetDistanceMm() + RemainingDist);
		}
	}
	else
	{
		//float DiffAngle = WrapAngle(AngleRef - PositionManager::Instance.GetAngleRad());
		//AngleRef = DiffAngle + PositionManager::Instance.GetAngleRad();

		// only apply a rotation if the difference of angle is too important
		//Serial.printf("angle: %f,  %f\r\n", AngleRef, PositionManager::Instance.GetAngleRad());
		if (AngleRefDiff > 0.5f)//fabs(AngleRef - PositionManager::Instance.GetAngleRad()) > 0.5f)
		{
			m_IsOnlyRotation = true;
			RemainingDist = 0.f;
		}

		ControlSystem::Instance.SetDistanceTarget(PositionManager::Instance.GetDistanceMm() + RemainingDist);
		ControlSystem::Instance.SetRadAngleTarget(AngleRef);
	}
}