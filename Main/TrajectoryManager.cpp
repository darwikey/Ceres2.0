#include <stdio.h>
#include "PositionManager.h"
#include "TrajectoryManager.h"

TrajectoryManager TrajectoryManager::Instance;


#define ABS(x) (((x) < 0)? -(x): (x))
#define SQUARE(x) (x*x)


/******************** User functions ********************/

void TrajectoryManager::trajectory_init()
{
	m_cur_id = 0;
	m_last_id = 0;
}

void TrajectoryManager::trajectory_end()
{
	m_cur_id = m_last_id;
}

int TrajectoryManager::trajectory_is_ended()
{
	return (m_cur_id == m_last_id);
}

void TrajectoryManager::trajectory_next_point()
{
	/* Update list pointer if not empty */
	if (!trajectory_is_ended()) {
		m_cur_id = (m_cur_id + 1) % TRAJECTORY_MAX_NB_POINTS;
	}
}

uint32_t TrajectoryManager::trajectory_get_cur_id()
{
	return m_cur_id;
}

uint32_t TrajectoryManager::trajectory_get_last_id()
{
	return m_last_id;
}

/******************** Movement functions ********************/

bool TrajectoryManager::trajectory_is_paused()
{
	return (!trajectory_is_ended() &&
		(m_points[m_cur_id].type == PAUSE));
}

void TrajectoryManager::trajectory_pause()
{
	if (!trajectory_is_paused()) {
		struct trajectory_dest dest;
		dest.type = PAUSE;
		trajectory_add_point(dest, NOW);
	}

	/* Force update now to stop more quickly */
	trajectory_update();
}

void TrajectoryManager::trajectory_resume()
{
	while (trajectory_is_paused()) {
		trajectory_next_point();
	}
}

void TrajectoryManager::trajectory_goto_d_mm(float d_mm)
{
	struct trajectory_dest dest;

	dest.type = D;
	dest.d.mm = d_mm;
	dest.d.precision = TRAJECTORY_DEFAULT_PRECISION_D_MM;

	trajectory_add_point(dest, END);
}

/* Absolute angle = current angle + relative angle. */
void TrajectoryManager::trajectory_goto_a_abs_deg(float a_deg)
{
	struct trajectory_dest dest;

	dest.type = A_ABS;
	dest.a.deg = a_deg;
	dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

	trajectory_add_point(dest, END);
}

void TrajectoryManager::trajectory_goto_a_rel_deg(float a_deg)
{
	struct trajectory_dest dest;

	dest.type = A_REL;
	dest.a.deg = a_deg;
	dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

	trajectory_add_point(dest, END);
}

/****************** Internal functions ******************/

void TrajectoryManager::trajectory_task()
{

#if SMOOTH_TRAJ
	trajectory_update_smoothly();
#else
	trajectory_update();
#endif


}

bool TrajectoryManager::trajectory_is_full()
{
	return (((m_last_id + 1) % TRAJECTORY_MAX_NB_POINTS) == m_cur_id);
}

void TrajectoryManager::trajectory_decrease_id(uint32_t *id)
{
	*id = (*id + TRAJECTORY_MAX_NB_POINTS - 1) % TRAJECTORY_MAX_NB_POINTS;
}

void TrajectoryManager::trajectory_add_point(struct trajectory_dest point, enum trajectory_when when)
{
	point.is_init = 0;

	if (when == END) {
		if (trajectory_is_full()) {
			printf("[trajectory_manager] Warning: List of points is full. Last point not added.\n");
			return;
		}

		/* New points are added at the end of the list */
		m_points[m_last_id] = point;

		/* Update end of list pointer */
		m_last_id = (m_last_id + 1) % TRAJECTORY_MAX_NB_POINTS;
	}
	else if (when == NOW) {
		if (trajectory_is_full()) {
			trajectory_decrease_id(&(m_last_id));
			printf("[trajectory_manager] Warning: List of points is full. Last point was removed.\n");
		}

		/* Insert a point before the current one */
		trajectory_decrease_id(&(m_cur_id));
		m_points[m_cur_id] = point;
	}
}

void TrajectoryManager::trajectory_manage_order_d(struct trajectory_dest *p)
{
	float d_mm_ref = p->starting_d_mm + p->d.mm;
	if (ABS(d_mm_ref - PositionManager::Instance.GetDistanceMm()) < p->d.precision) {
		trajectory_next_point();
	}
	else {
		control_system_set_distance_mm_ref(d_mm_ref);
	}
}

void TrajectoryManager::trajectory_manage_order_a_abs(struct trajectory_dest *p)
{
	// TODO: Better handle
	if (ABS(p->a.deg - PositionManager::Instance.GetAngleDeg()) < p->a.precision) {
		trajectory_next_point();
	}
	else {
		control_system_set_angle_deg_ref(p->a.deg);
	}
}

void TrajectoryManager::trajectory_manage_order_a_rel(struct trajectory_dest *p)
{
	float a_deg_ref = p->starting_a_deg + p->a.deg;
	if (ABS(a_deg_ref - PositionManager::Instance.GetAngleDeg()) < p->a.precision) {
		trajectory_next_point();
	}
	else {
		control_system_set_angle_deg_ref(a_deg_ref);
	}
}

void TrajectoryManager::trajectory_manage_order_pause(struct trajectory_dest *p)
{
	control_system_set_distance_mm_ref(p->starting_d_mm);
	control_system_set_angle_deg_ref(p->starting_a_deg);
}

void TrajectoryManager::trajectory_update()
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
		trajectory_manage_order_d(p);
		break;
	case A_ABS:
		trajectory_manage_order_a_abs(p);
		break;
	case A_REL:
		trajectory_manage_order_a_rel(p);
		break;
	case PAUSE:
		trajectory_manage_order_pause(p);
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
//			trajectory_manage_order_a_abs(p);
//			break;
//
//		case A_REL:
//			trajectory_manage_order_a_rel(p);
//			break;
//
//		case PAUSE:
//			trajectory_manage_order_pause(p);
//			break;
//
//		default:
//			break;
//	}
//
//	float _WaypointDistance = sqrtf(SQUARE(position_get_x_mm() - _nextWaypoint_x) + SQUARE(position_get_y_mm() * _nextWaypoint_y));
//}
