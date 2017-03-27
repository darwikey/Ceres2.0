//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <WProgram.h>

#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "CommandLineInterface.h"
#include "Platform.h"
#include "Astar.h"

CommandLineInterface CommandLineInterface::Instance;

#define ARG_LENGTH 20
void CommandLineInterface::Task()
{
	bool end_line = false;
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		m_Buffer[m_CurPos] = c;
		Serial.print(c);
		if (c == '\r' || c == '\n')
		{
			end_line = true;
			break;
		}
		if (m_CurPos < CLI_BUFFER_SIZE)
			m_CurPos++;
	}

	if (end_line)
	{

		// read the incoming byte:
		char command = m_Buffer[0];
		char arg[ARG_LENGTH] = { 0 };
		char arg2[ARG_LENGTH] = { 0 };

		int i = 2;
		for (int j = 0;
			i < m_CurPos && m_Buffer[i] != ' ' && j < ARG_LENGTH - 1;
			i++, j++)
		{
			arg[j] = m_Buffer[i];
		}
		i++;
		for (int j = 0;
			i < m_CurPos && m_Buffer[i] != ' ' && j < ARG_LENGTH - 1;
			i++, j++)
		{
			arg2[j] = m_Buffer[i];
		}

		m_CurPos = 0;
		//Serial.printf("commande %c \"%s\" \"%s\"\r\n", command, arg, arg2);

		if (command == 'd') {
			double value = atof(arg);
			TrajectoryManager::Instance.GotoDistance(value);
			Serial.printf("Distance: %f\r\n", value);
		}
		else if (command == 'a') {
			double value = atof(arg);
			TrajectoryManager::Instance.GotoRelativeAngle(value);
			Serial.printf("Angle: %f\r\n", (double)value);
		}
		else if (command == 'c') {
			//double x = atof(arg);
			//double y = atof(arg2);
			//trajectory_goto_xy_mm(x, y);
			//Serial.printf("Goto xy: %f  %f\r\n", x, y);
			astar_test(atoi(arg), atoi(arg2));
		}
		//else if (command == '*') {
		//	float y = cli_getfloat();
		//	set_startCoor(positionMmToCoordinate(position_get_x_mm(), position_get_y_mm()));
		//	set_goalCoor(positionMmToCoordinate(value, y));
		//	Serial.printf("Goto xy: %f  %f\r\n", (double)value, (double)y);
		//	astarMv();
		//}
		//else if (command == 'z') {
		//	smooth_traj_end();
		//	Serial.printf("stop mvt\r\n");
		//}
		else if (command == 's') {
			double value = atof(arg2);
			if (!strncmp(arg, "speed_high", ARG_LENGTH)) {
				ControlSystem::Instance.SetSpeedHigh();
				Serial.print("Max speed.\r\n");
			}
			else if (!strncmp(arg, "speed_medium", ARG_LENGTH)) {
				ControlSystem::Instance.SetSpeedMedium();
				Serial.print("Medium speed.\r\n");
			}
			else if (!strncmp(arg, "speed_low", ARG_LENGTH)) {
				ControlSystem::Instance.SetSpeedLow();
				Serial.print("Low speed.\r\n");
			}
			else if (!strncmp(arg, "speed", ARG_LENGTH)) {
				ControlSystem::Instance.SetSpeedRatio(value);
				Serial.printf("Speed: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_d_p", ARG_LENGTH)) {
				ControlSystem::Instance.GetDistancePID().SetKP(value);
				Serial.printf("Distance P: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_d_i", ARG_LENGTH)) {
				ControlSystem::Instance.GetDistancePID().SetKI(value);
				Serial.printf("Distance I: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_d_d", ARG_LENGTH)) {
				ControlSystem::Instance.GetDistancePID().SetKD(value);
				Serial.printf("Distance D: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_a_p", ARG_LENGTH)) {
				ControlSystem::Instance.GetAnglePID().SetKP(value);
				Serial.printf("Angle P: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_a_i", ARG_LENGTH)) {
				ControlSystem::Instance.GetAnglePID().SetKI(value);
				Serial.printf("Angle I: %f\r\n", value);
			}
			else if (!strncmp(arg, "pid_a_d", ARG_LENGTH)) {
				ControlSystem::Instance.GetAnglePID().SetKD(value);
				Serial.printf("Angle D: %f\r\n", value);
			}
			else if (!strncmp(arg, "axle_track", ARG_LENGTH)) {
				PositionManager::Instance.SetAxleTrackMm(value);
				Serial.printf("Axle track: %f\r\n", value);
			}
			else if (!strncmp(arg, "speed_d", ARG_LENGTH)) {
				ControlSystem::Instance.SetDistanceMaxSpeed(value);
				Serial.printf("distance max speed: %f\r\n", value);
			}
			else if (!strncmp(arg, "speed_a", ARG_LENGTH)) {
				ControlSystem::Instance.SetAngleMaxSpeed(value);
				Serial.printf("angle max speed: %f\r\n", value);
			}
			else if (!strncmp(arg, "acc_d", ARG_LENGTH)) {
				ControlSystem::Instance.SetDistanceMaxAcc(value);
				Serial.printf("distance max acceleration: %f\r\n", value);
			}
			else if (!strncmp(arg, "acc_a", ARG_LENGTH)) {
				ControlSystem::Instance.SetAngleMaxAcc(value);
				Serial.printf("angle max acceleration: %f\r\n", value);
			}
			else if (!strncmp(arg, "servo_baudrate", ARG_LENGTH)) {
				Platform::ForceServoBaudRate();
				Serial.print("Force servo baudrate\r\n");
			}
			else if (!strncmp(arg, "servo_id", ARG_LENGTH)) {
				int id = atoi(arg2);
				Platform::ForceServoId(id);
				Serial.printf("Force all connected servo to id %d\r\n", id);
			}
			else {
				Serial.printf("Invalid argument '%s'.\r\n", arg);
			}
		}
		else if (command == 'p') {
			if (!strncmp(arg, "x", ARG_LENGTH)) {
				Serial.printf("Robot x mm: %f\r\n", PositionManager::Instance.GetXMm());
			}
			else if (!strncmp(arg, "y", ARG_LENGTH)) {
				Serial.printf("Robot y mm: %f\r\n", PositionManager::Instance.GetYMm());
			}
			else if (!strncmp(arg, "a", ARG_LENGTH)) {
				Serial.printf("Robot angle deg: %f\r\n", PositionManager::Instance.GetAngleDeg());
			}
			else if (!strncmp(arg, "d", ARG_LENGTH)) {
				Serial.printf("Robot distance mm: %f\r\n", PositionManager::Instance.GetDistanceMm());
			}
			else if (!strncmp(arg, "enc_r", ARG_LENGTH)) {
				Serial.printf("Right encoder value: %f\r\n", PositionManager::Instance.GetRightEncoder());
			}
			else if (!strncmp(arg, "enc_l", ARG_LENGTH)) {
				Serial.printf("Left encoder value: %f\r\n", PositionManager::Instance.GetLeftEncoder());
			}
			/*else if (!strncmp(arg, "cur_id", ARG_LENGTH)) {
			  Serial.printf("Traj manager cur_id: %d\r\n", (int)GetCurId());
			}
			else if (!strncmp(arg, "last_id", ARG_LENGTH)) {
			  Serial.print("Traj manager last_id: %d\r\n", (int)GetLastId());
			}*/
			else if (!strncmp(arg, "pid", ARG_LENGTH)) {
				Serial.printf("Distance PID: %f, %f, %f\r\n", ControlSystem::Instance.GetDistancePID().GetKP(),
					ControlSystem::Instance.GetDistancePID().GetKI(),
					ControlSystem::Instance.GetDistancePID().GetKd());
				Serial.printf("Angle PID:    %f, %f, %f\r\n", ControlSystem::Instance.GetAnglePID().GetKP(),
					ControlSystem::Instance.GetAnglePID().GetKI(),
					ControlSystem::Instance.GetAnglePID().GetKd());
			}
			else if (!strncmp(arg, "graph", ARG_LENGTH)) {
				Graph::Instance.print();
			}
			else {
				Serial.printf("Invalid argument '%s'.\r\n", arg);
			}
		}
		else if (command == 'm') {
			if (!strncmp(arg, "servo1", ARG_LENGTH)) {
				Platform::SetServoPos(ServoID::SERVO1, atoi(arg2));
				Serial.print("Move servo 1\r\n");
				/*if (!strncmp(arg2, "up", ARG_LENGTH)) {
					action_raise_lift();
					Serial.print("Lift up\r\n");
				}
				else if (!strncmp(arg2, "down", ARG_LENGTH)) {
					action_lower_lift();
					Serial.print("Lift down\r\n");
				}
				else {
					Serial.print("Invalid argument '%s'.\r\n", arg2);
				}*/
			}
			/*else if (!strncmp(arg, "grip", ARG_LENGTH)) {
				ausbeeSetAngleServo(&servo_grip, atoi(arg2));
				Serial.print("command servo.\r\n");
			}
			else if (!strncmp(arg, "clapet", ARG_LENGTH)) {
				ausbeeSetAngleServo(&servo_clapet, atoi(arg2));
				Serial.print("command servo.\r\n");
			}*/
			else {
				Serial.printf("Invalid argument '%s'.\r\n", arg);
			}
		}
		else if (command == 'h') {
			Serial.print("Help:\r\n");
			Serial.print("  Available commands are:\r\n");
			Serial.print("  d <float>: Go forward/backward with the specified distance in mm.\r\n");
			Serial.print("  a <float>: Rotate with the specified angle in degrees.\r\n");
			Serial.print("  c <float> <float>: goto to the position (x, y) in mm.\r\n");
			Serial.print("  * <float> <float>: goto to the position (x, y) in mm with A*.\r\n");
			Serial.print("  s <arg> <value>:   Set internal value.\r\n");
			Serial.print("             <value> should be a float.\r\n");
			Serial.print("             <arg> can be one of:\r\n");
			Serial.print("             speed_high:   set highest translation and rotation speed.\r\n");
			Serial.print("             speed_medium: set medium translation and rotation speed.\r\n");
			Serial.print("             speed_low:    set low translation and rotation speed.\r\n");
			Serial.print("             speed :       set translation and rotation speed ratio to value (0 <= value <= 1).\r\n");
			Serial.print("             pid_d_p :     set distance PID proportional value.\r\n");
			Serial.print("             pid_d_i :     set distance PID integral value.\r\n");
			Serial.print("             pid_d_d :     set distance PID derivative value.\r\n");
			Serial.print("             pid_a_p :     set angle PID proportional value.\r\n");
			Serial.print("             pid_a_i :     set angle PID integral value.\r\n");
			Serial.print("             pid_a_d :     set angle PID derivative value.\r\n");
			Serial.print("             axle_track :  set axle track in mm.\r\n");
			Serial.print("             speed_d :     set max speed for distance.\r\n");
			Serial.print("             speed_a :     set max speed for angle.\r\n");
			Serial.print("             acc_d :       set max acceleration for distance.\r\n");
			Serial.print("             acc_a :       set max acceleration for angle.\r\n");
			Serial.print("  m <arg> <arg2> : move an actuator\r\n");
			Serial.print("             <arg> can be one of: \r\n");
			Serial.print("             arm_l: left_arm \r\n");
			Serial.print("             arm_r: right_arm \r\n");
			Serial.print("                <arg2> can be one of: \r\n");
			Serial.print("                close: close the arm \r\n");
			Serial.print("                open: open the arm \r\n");
			Serial.print("  p <arg>:   Print internal value.\r\n");
			Serial.print("             <arg> can be one of:\r\n");
			Serial.print("             x:        print robot's x position.\r\n");
			Serial.print("             y:        print robot's y position.\r\n");
			Serial.print("             a:        print robot's angle.\r\n");
			Serial.print("             d:        print robot's distance.\r\n");
			Serial.print("             enc_l:    print left encoder's value.\r\n");
			Serial.print("             enc_r:    print right encoder's value.\r\n");
			Serial.print("             cur_id:   print Traj manager's current point id.\r\n");
			Serial.print("             last_id:  print Traj manager's last point id.\r\n");
			Serial.print("             pid:      print PID.\r\n");
			Serial.print("             graphe:   print A* graphe.\r\n");
		}
		else {
			Serial.printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
		}

		m_CurPos = 0;
	}
}
