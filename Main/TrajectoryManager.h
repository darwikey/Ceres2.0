#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "control_system.h"

#define TRAJECTORY_UPDATE_PERIOD_S 0.1 // 100 ms

#define TRAJECTORY_MAX_NB_POINTS 50

#define TRAJECTORY_DEFAULT_PRECISION_D_MM  10.0
#define TRAJECTORY_DEFAULT_PRECISION_A_DEG 5.0f //1.0


#define SMOOTH_TRAJ 0

class TrajectoryManager {
public:
	static TrajectoryManager Instance;

	void trajectory_init();

	void trajectory_task();

	/* Remove every points from the trajectory */
	void trajectory_end();
	/* Check whether points remain in the trajectory*/
	int trajectory_is_ended();

	void trajectory_next_point();

	uint32_t trajectory_get_cur_id();
	uint32_t trajectory_get_last_id();

	bool trajectory_is_paused();
	void trajectory_pause();
	void trajectory_resume();

	void trajectory_goto_d_mm(float d_mm);

	/* Set absolute angle. Does not depend on current angle. */
	void trajectory_goto_a_abs_deg(float a_deg_ref);

	/* Set relative angle. Depends on current angle. */
	void trajectory_goto_a_rel_deg(float a_deg);

private:
	enum order_type {
		PAUSE, D, A_ABS, A_REL
	};

	enum trajectory_when {
		NOW, END
	};

	struct trajectory_dest {
		union {
			struct {
				float mm;
				float precision;
			} d;

			struct {
				float deg;
				float precision;
			} a;
		};

#if SMOOTH_TRAJ
		float x;
		float y;
#endif

		float starting_d_mm;
		float starting_a_deg;
		uint8_t is_init;
		enum order_type type;
	};



	bool trajectory_is_full();
	void trajectory_decrease_id(uint32_t *id);
	void trajectory_manage_order_d(struct trajectory_dest *p);
	void trajectory_manage_order_a_abs(struct trajectory_dest *p);
	void trajectory_manage_order_a_rel(struct trajectory_dest *p);
	void trajectory_manage_order_pause(struct trajectory_dest *p);
	void trajectory_update();
	//void trajectory_update_smoothly();
	void trajectory_add_point(struct trajectory_dest point, enum trajectory_when when);

	struct trajectory_dest m_points[TRAJECTORY_MAX_NB_POINTS];
	uint32_t m_cur_id;
	uint32_t m_last_id;
};

#endif /* TRAJECTORY_MANAGER_H */
