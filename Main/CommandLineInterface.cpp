//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <WProgram.h>

#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "CommandLineInterface.h"
#include "Platform.h"
#include "Astar.h"
#include "Strategy.h"

CommandLineInterface CommandLineInterface::Instance;

#define ARG_LENGTH 20
void CommandLineInterface::Task()
{
	bool end_line = false;
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		if (c == CLI_DEL_CHAR && m_CurPos > 0)
		{
			m_CurPos--;
			continue;
		}
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

		//Serial.printf("commande %c \"%s\" \"%s\"\r\n", command, arg, arg2);

		if (command == 'd') {
			double value = atof(arg);
			TrajectoryManager::Instance.GotoDistance(value);
			Serial.printf("Distance: %f\r\n", value);
		}
		else if (command == 'a') {
			double value = atof(arg);
			TrajectoryManager::Instance.GotoDegreeAngle(value);
			Serial.printf("Angle: %f\r\n", (double)value);
		}
		else if (command == 'c') {
			Float2 p(atof(arg), atof(arg2));
			TrajectoryManager::Instance.GotoXY(p);
			Serial.printf("Goto xy: %f  %f\r\n", p.x, p.y);
		}
		else if (command == '*') {
			AStarCoord coord;
			coord.FromWordPosition(Float2(atoi(arg), atoi(arg2)));
			astar_test(coord);
		}
		else if (command == 't') {
			TrajectoryManager::Instance.GotoXY(Float2(0, 500));
			TrajectoryManager::Instance.GotoXY(Float2(500, 500));
			TrajectoryManager::Instance.GotoXY(Float2(500, 0));
		}
		else if (command == 'z') {
			TrajectoryManager::Instance.Reset();
			Serial.printf("stop mvt\r\n");
		}
		//TODO deplacer dans une vraie commande quand le CLI sera mieux
		else if (command == 'x') {
			char arg3[ARG_LENGTH] = { 0 };
			i++;
			for (int j = 0;
				i < m_CurPos && m_Buffer[i] != ' ' && j < ARG_LENGTH - 1;
				i++, j++)
			{
				arg3[j] = m_Buffer[i];
			}
			int id = atoi(arg);
			int adr = atoi(arg2);
			int val = atoi(arg3);
			Platform::SendServoPacket(id, (XL320::Address)adr, val);
			Serial.printf("packet id:%d adr:%d val:%d\r\n", id, adr, val);
		}
		//else if (command == 'r') {
		//	int id = atoi(arg);
		//	int adr = atoi(arg2);
		//	Serial.printf("packet id:%d adr:%d\r\n", id, adr);
		//	Platform::ReadServoPacket(id, adr);
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
				Serial.printf("Force all connected servo to id %d\r\n. Should be the first instruction sent to the servo\r\n", id);
			}
			else if (!strncmp(arg, "side", ARG_LENGTH)) {
				if (arg2[0] == 'b')
					Strategy::Instance.SetSide(Side::BLUE);
				else if (arg2[0] == 'y')
					Strategy::Instance.SetSide(Side::YELLOW);
				else
					Serial.print("incorrect side, must be \'b\' or \'y\'\r\n");
				Strategy::Instance.Print();
			}
			else if (!strncmp(arg, "start", ARG_LENGTH)) {
				Strategy::Instance.Start();
			}
			else {
				Serial.printf("Invalid argument '%s'.\r\n", arg);
			}
		}
		else if (command == 'p') {
			if (!strncmp(arg, "pos", ARG_LENGTH)) {
				Serial.printf("Robot pos mm: %f, %f\r\n", PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm());
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
			else if (!strncmp(arg, "cur_id", ARG_LENGTH)) {
			  Serial.printf("Traj manager cur_id: %d\r\n", (int)TrajectoryManager::Instance.GetCurId());
			}
			else if (!strncmp(arg, "last_id", ARG_LENGTH)) {
			  Serial.printf("Traj manager last_id: %d\r\n", (int)TrajectoryManager::Instance.GetLastId());
			}
			else if (!strncmp(arg, "pid", ARG_LENGTH)) {
				Serial.printf("Distance PID: %f, %f, %f\r\n", ControlSystem::Instance.GetDistancePID().GetKP(),
					ControlSystem::Instance.GetDistancePID().GetKI(),
					ControlSystem::Instance.GetDistancePID().GetKd());
				Serial.printf("Angle PID:    %f, %f, %f\r\n", ControlSystem::Instance.GetAnglePID().GetKP(),
					ControlSystem::Instance.GetAnglePID().GetKI(),
					ControlSystem::Instance.GetAnglePID().GetKd());
			}
			else if (!strncmp(arg, "graph", ARG_LENGTH)) {
				Graph::Instance.Print();
			}
			else if (!strncmp(arg, "servo", ARG_LENGTH)) {
				Platform::DebugServoRam(atoi(arg2));
			}
			else if (!strncmp(arg, "strategy", ARG_LENGTH)) {
				Strategy::Instance.Print();
			}
			else if (!strncmp(arg, "gp2", ARG_LENGTH)) {
				Platform::DebugGP2();
			}
			else if (!strncmp(arg, "date", ARG_LENGTH)) {
				extern const char* gCompileDate;
				extern const char* gCompileTime;
				Serial.printf("%s - %s\r\n", gCompileDate, gCompileTime);
			}
			else {
				Serial.printf("Invalid argument '%s'.\r\n", arg);
			}
		}
		else if (command == 'm') {
			if (!strncmp(arg, "s1", ARG_LENGTH)) {
				Platform::SetServoPos(ServoID::SERVO1, atoi(arg2));
				Serial.print("Move servo 1\r\n");
			}
			else if (!strncmp(arg, "s2", ARG_LENGTH)) {
				Platform::SetServoPos(ServoID::SERVO2, atoi(arg2));
				Serial.print("Move servo 2\r\n");
			}
			else if (!strncmp(arg, "arm", ARG_LENGTH)) {
				if (!strncmp(arg2, "normal", ARG_LENGTH)) {
					Strategy::Instance.SetArmState(ArmState::NORMAL);
					Serial.print("arm normal.\r\n");
				}
				else if (!strncmp(arg2, "emptying", ARG_LENGTH)) {
					Strategy::Instance.SetArmState(ArmState::EMPTYING);
					Serial.print("arm emptying.\r\n");
				}
			}
			else if (!strncmp(arg, "grip", ARG_LENGTH)) {
				if (!strncmp(arg2, "close", ARG_LENGTH)) {
					Strategy::Instance.SetGripState(GripState::CLOSE);
					Serial.print("grip close.\r\n");
				}
				if (!strncmp(arg2, "normal", ARG_LENGTH)) {
					Strategy::Instance.SetGripState(GripState::NORMAL);
					Serial.print("grip normal.\r\n");
				}
				else if (!strncmp(arg2, "open", ARG_LENGTH)) {
					Strategy::Instance.SetGripState(GripState::FULLY_OPEN);
					Serial.print("grip fully open.\r\n");
				}
			}
			else if (!strncmp(arg, "pushrobot", ARG_LENGTH)) {
				Strategy::Instance.PushRobotAgainstWall();
			}
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
			Serial.print("  t: test trajectory\r\n");
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
			Serial.print("             servo_baudrate set servo baudrate.\r\n");
			Serial.print("             servo_id <int> set servo id.\r\n");
			Serial.print("             side <b|y> :  set color of the start zone\r\n");
			Serial.print("  m <arg> <arg2> : move an actuator\r\n");
			Serial.print("             <arg> can be one of: \r\n");
			Serial.print("             s1 <int>: set servo1 pos \r\n");
			Serial.print("             s2 <int>: set servo2 pos \r\n");
			Serial.print("             arm <arg2>: set arm pos. arg2: normal, emptying\r\n");
			Serial.print("             grip <arg2>: set grip pos. arg2: close, normal, open\r\n");
			Serial.print("             pushrobot: push robot against wall\r\n");
			Serial.print("  p <arg>:   Print internal value.\r\n");
			Serial.print("             <arg> can be one of:\r\n");
			Serial.print("             pos:      Print robot's position.\r\n");
			Serial.print("             a:        Print robot's angle.\r\n");
			Serial.print("             d:        Print robot's distance.\r\n");
			Serial.print("             enc_l:    Print left encoder's value.\r\n");
			Serial.print("             enc_r:    Print right encoder's value.\r\n");
			Serial.print("             cur_id:   Print Traj manager's current point id.\r\n");
			Serial.print("             last_id:  Print Traj manager's last point id.\r\n");
			Serial.print("             pid:      Print PID.\r\n");
			Serial.print("             strategy: Print strategy\r\n");
			Serial.print("             gp2:      Debug gp2\r\n");
			Serial.print("             graph:    Print A* graph.\r\n");
			Serial.print("             servo <int>: Print all servo registers.\r\n");
			Serial.print("             date      Print the binary creation timestamp.\r\n");
		}
		else {
			Serial.printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
		}

		m_CurPos = 0;
	}
}
