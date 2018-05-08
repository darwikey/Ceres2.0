#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "Globals.h"
#include "ControlSystem.h"


#define SMOOTH_TRAJ_UPDATE_PERIOD_S 0.01 // 100 ms

#define SMOOTH_TRAJ_MAX_NB_POINTS 50

#define SMOOTH_TRAJ_DEFAULT_PRECISION_D_MM  40.0
#define SMOOTH_TRAJ_DEFAULT_PRECISION_A_RAD (DEG2RAD(5.0f))

#define SMOOTH_TRAJ_STEER_DISTANCE_MM 70


class TrajectoryManager {
public:
	static TrajectoryManager Instance;
	void Init();

	void Task();

	/* Remove every points from the trajectory */
	void Reset();
	/* Check whether points remain in the trajectory*/
	int IsEnded() { return m_Points.IsEmpty(); }

	void NextPoint();

	void Print();
	bool IsForwardMovement();
	bool IsOnlyRotation(); 

	bool IsPaused();
	void Pause();
	void Resume();

	void GotoXY(const Float2 &_pos_mm);

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

		Float2 pos;//mm
		float angle;//rad
		OrderType movement;
	};

	bool TrajIsFull() { return m_Points.IsFull(); }
	void AddPoint(TrajDest point, TrajWhen when);
	void GotoTarget(const TrajDest &_nextPoint, const Float2 &_target);
	void Update();

	CircularBuffer<TrajDest, SMOOTH_TRAJ_MAX_NB_POINTS> m_Points;
	bool m_Pause;
	float m_PauseDist, m_pauseAngle;
	bool m_IsOnlyRotation;
};
#endif /* TRAJECTORY_MANAGER_H */
