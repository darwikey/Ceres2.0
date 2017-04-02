#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "ControlSystem.h"


#define SMOOTH_TRAJ_UPDATE_PERIOD_S 0.01 // 100 ms

#define SMOOTH_TRAJ_MAX_NB_POINTS 50

#define SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM  40.0
#define SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD (3.0f * 0.01745329f) //1 degrees

#define SMOOTH_TRAJ_STEER_DISTANCE_MM 70


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

	void GotoXY(float x_mm, float y_mm);

	void GotoDistance(float d_mm);
	void GotoDegreeAngle(float a);
	void GotoRadianAngle(float a);

private:
	enum TrajWhen {
		NOW, END
	};

	enum OrderType {
		COMMON, FORWARD, BACKWARD
	};

	struct TrajDest {

		float x;//mm
		float y;//mm
		float a;//rad
		OrderType movement;
	};

	bool TrajIsFull();
	void DecreaseId(uint32_t *id);
	void AddPoint(TrajDest point, TrajWhen when);
	void GotoTarget(TrajDest* next_point, float target_x, float target_y);
	void Update();

	TrajDest m_Points[SMOOTH_TRAJ_MAX_NB_POINTS];
	uint32_t m_CurId;
	uint32_t m_LastId;
	bool m_Pause;
};
#endif /* TRAJECTORY_MANAGER_H */
