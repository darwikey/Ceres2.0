#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "ControlSystem.h"

#define TRAJECTORY_UPDATE_PERIOD_S 0.01 // 10 ms

#define TRAJECTORY_MAX_NB_POINTS 50

#define TRAJECTORY_DEFAULT_PRECISION_D_MM  10.0
#define TRAJECTORY_DEFAULT_PRECISION_A_DEG 5.0f //1.0


#define SMOOTH_TRAJ 0

class TrajectoryManager {
public:
	static TrajectoryManager Instance;

	void Init();

	void Task();

	/* Remove every points from the trajectory */
	void Reset();
	/* Check whether points remain in the trajectory*/
	int IsEnded();

	void NextPoint();

	uint32_t GetCurId();
	uint32_t GetLastId();

	bool IsPaused();
	void Pause();
	void Resume();

	void GotoDistance(float d_mm);

	/* Set absolute angle. Does not depend on current angle. */
	void GotoAbsoluteAngle(float a_deg_ref);

	/* Set relative angle. Depends on current angle. */
	void GotoRelativeAngle(float a_deg);

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


	bool IsFull();
	void DecreaseId(uint32_t *id);
	void ManageDistanceOrder(struct trajectory_dest *p);
	void ManageAbsoluteAngleOrder(struct trajectory_dest *p);
	void ManageRelativeAngleOrder(struct trajectory_dest *p);
	void ManagePauseOrder(struct trajectory_dest *p);
	void Update();
	//void trajectory_update_smoothly();
	void AddPoint(struct trajectory_dest point, enum trajectory_when when);

	struct trajectory_dest m_points[TRAJECTORY_MAX_NB_POINTS];
	uint32_t m_cur_id;
	uint32_t m_last_id;
};

#endif /* TRAJECTORY_MANAGER_H */
