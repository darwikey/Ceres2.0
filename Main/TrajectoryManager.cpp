#include <stdio.h>
#include "PositionManager.h"
#include "TrajectoryManager.h"

TrajectoryManager TrajectoryManager::Instance;


#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) (x*x)


/******************** User functions ********************/

void TrajectoryManager::Init()
{
	m_cur_id = 0;
	m_last_id = 0;
}

void TrajectoryManager::Reset()
{
	m_cur_id = m_last_id;
}

int TrajectoryManager::IsEnded()
{
	return (m_cur_id == m_last_id);
}

void TrajectoryManager::NextPoint()
{
	/* Update list pointer if not empty */
	if (!IsEnded()) {
		m_cur_id = (m_cur_id + 1) % TRAJECTORY_MAX_NB_POINTS;
	}
}

uint32_t TrajectoryManager::GetCurId()
{
	return m_cur_id;
}

uint32_t TrajectoryManager::GetLastId()
{
	return m_last_id;
}

/******************** Movement functions ********************/

bool TrajectoryManager::IsPaused()
{
	return (!IsEnded() &&
		(m_points[m_cur_id].type == PAUSE));
}

void TrajectoryManager::Pause()
{
	if (!IsPaused()) {
		struct trajectory_dest dest;
		dest.type = PAUSE;
		AddPoint(dest, NOW);
	}

	/* Force update now to stop more quickly */
	Update();
}

void TrajectoryManager::Resume()
{
	while (IsPaused()) {
		NextPoint();
	}
}

void TrajectoryManager::GotoDistance(float d_mm)
{
	struct trajectory_dest dest;

	dest.type = D;
	dest.d.mm = d_mm;
	dest.d.precision = TRAJECTORY_DEFAULT_PRECISION_D_MM;

	AddPoint(dest, END);
}

/* Absolute angle = current angle + relative angle. */
void TrajectoryManager::GotoAbsoluteAngle(float a_deg)
{
	struct trajectory_dest dest;

	dest.type = A_ABS;
	dest.a.deg = a_deg;
	dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

	AddPoint(dest, END);
}

void TrajectoryManager::GotoRelativeAngle(float a_deg)
{
	struct trajectory_dest dest;

	dest.type = A_REL;
	dest.a.deg = a_deg;
	dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

	AddPoint(dest, END);
}

/****************** Internal functions ******************/

void TrajectoryManager::Task()
{

#if SMOOTH_TRAJ
	trajectory_update_smoothly();
#else
	Update();
#endif


}

bool TrajectoryManager::IsFull()
{
	return (((m_last_id + 1) % TRAJECTORY_MAX_NB_POINTS) == m_cur_id);
}

void TrajectoryManager::DecreaseId(uint32_t *id)
{
	*id = (*id + TRAJECTORY_MAX_NB_POINTS - 1) % TRAJECTORY_MAX_NB_POINTS;
}

void TrajectoryManager::AddPoint(struct trajectory_dest point, enum trajectory_when when)
{
	point.is_init = 0;

	if (when == END) {
		if (IsFull()) {
			printf("[trajectory_manager] Warning: List of points is full. Last point not added.\n");
			return;
		}

		/* New points are added at the end of the list */
		m_points[m_last_id] = point;

		/* Update end of list pointer */
		m_last_id = (m_last_id + 1) % TRAJECTORY_MAX_NB_POINTS;
	}
	else if (when == NOW) {
		if (IsFull()) {
			DecreaseId(&(m_last_id));
			printf("[trajectory_manager] Warning: List of points is full. Last point was removed.\n");
		}

		/* Insert a point before the current one */
		DecreaseId(&(m_cur_id));
		m_points[m_cur_id] = point;
	}
}

void TrajectoryManager::ManageDistanceOrder(struct trajectory_dest *p)
{
	float d_mm_ref = p->starting_d_mm + p->d.mm;
	if (ABS(d_mm_ref - PositionManager::Instance.GetDistanceMm()) < p->d.precision) {
		NextPoint();
	}
	else {
		ControlSystem::Instance.SetDistanceRef(d_mm_ref);
	}
}

void TrajectoryManager::ManageAbsoluteAngleOrder(struct trajectory_dest *p)
{
	// TODO: Better handle
	if (ABS(p->a.deg - PositionManager::Instance.GetAngleDeg()) < p->a.precision) {
		NextPoint();
	}
	else {
		ControlSystem::Instance.SetDegAngleRef(p->a.deg);
	}
}

void TrajectoryManager::ManageRelativeAngleOrder(struct trajectory_dest *p)
{
	float a_deg_ref = p->starting_a_deg + p->a.deg;
	if (ABS(a_deg_ref - PositionManager::Instance.GetAngleDeg()) < p->a.precision) {
		NextPoint();
	}
	else {
		ControlSystem::Instance.SetDegAngleRef(a_deg_ref);
	}
}

void TrajectoryManager::ManagePauseOrder(struct trajectory_dest *p)
{
	ControlSystem::Instance.SetDistanceRef(p->starting_d_mm);
	ControlSystem::Instance.SetDegAngleRef(p->starting_a_deg);
}

void TrajectoryManager::Update()
{
	/* Nothing to do if there is no point in the list */
	if (m_cur_id == m_last_id) {
		return;
	}

	/* Get current point reference */
	struct trajectory_dest *p = m_points + m_cur_id;

	/* Get current starting position in distance and angle */
	if (!p->is_init) {
		p->starting_d_mm = PositionManager::Instance.GetDistanceMm();
		p->starting_a_deg = PositionManager::Instance.GetAngleDeg();
		p->is_init = 1;
	}

	/* Set new reference according to point type */
	switch (p->type) {
	case D:
		ManageDistanceOrder(p);
		break;
	case A_ABS:
		ManageAbsoluteAngleOrder(p);
		break;
	case A_REL:
		ManageRelativeAngleOrder(p);
		break;
	case PAUSE:
		ManagePauseOrder(p);
		break;
	default:
		break;
	}
}

// work in progress
//static inline void trajectory_update_smoothly()
//{
//	/* Nothing to do if there is no point in the list */
//	if (m_cur_id == m_last_id) {
//		return;
//	}
//
//	/* Get current point reference */
//	struct trajectory_dest *p = m_points + m_cur_id;
//
//	/* Get current starting position in distance and angle */
//	/*if (!p->is_init) {
//		p->starting_d_mm = position_get_distance_mm();
//		p->starting_a_deg = position_get_angle_deg();
//		p->is_init = 1;
//	}*/
//
//	//static float target_x = 0;
//	//static float target_y = 0;
//
//
//  /* Set new reference according to point type */
//	switch (p->type) {
//		case D:
//			//float d_mm_ref = p->starting_d_mm + p->d.mm;
//			break;
//
//		case A_ABS:
//			ManageAbsoluteAngleOrder(p);
//			break;
//
//		case A_REL:
//			ManageRelativeAngleOrder(p);
//			break;
//
//		case PAUSE:
//			ManagePauseOrder(p);
//			break;
//
//		default:
//			break;
//	}
//
//	float _WaypointDistance = sqrtf(SQUARE(position_get_x_mm() - _nextWaypoint_x) + SQUARE(position_get_y_mm() * _nextWaypoint_y));
//}
