#include <string.h>
#include <WProgram.h>

#include "PositionManager.h"
#include "TrajectoryManager.h"
#include "CommandLineInterface.h"
#include "Platform.h"
#include "Astar.h"
#include "Strategy.h"
#include "MotorManager.h"

CommandLineInterface CommandLineInterface::Instance;

void CommandLineInterface::Init()
{
	auto REGISTER_COMMAND = [this](const char* cmd, const char* help, Command::FunctionDecl fct)
	{
		static int i = 0;
		m_Command[i].cmd = cmd;
		m_Command[i].help = help;
		m_Command[i].function = fct;
		i++;
	};

	REGISTER_COMMAND("d", "Goto distance", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		TrajectoryManager::Instance.GotoDistance(value);
		Serial.printf("Distance: %f\r\n", value);
	});

	REGISTER_COMMAND("a", "Goto angle (degree)", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		TrajectoryManager::Instance.GotoDegreeAngle(value);
		Serial.printf("Angle: %f\r\n", value);
	});

	REGISTER_COMMAND("c", "Goto xy", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Float2 p(atof(_argv[0]), atof(_argv[1]));
		TrajectoryManager::Instance.GotoXY(p);
		Serial.printf("Goto xy: %f  %f\r\n", p.x, p.y);
	});

	REGISTER_COMMAND("test", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		TrajectoryManager::Instance.GotoXY(Float2(0, 500));
		TrajectoryManager::Instance.GotoXY(Float2(500, 500));
		TrajectoryManager::Instance.GotoXY(Float2(500, 0));
	});

	REGISTER_COMMAND("stop", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		TrajectoryManager::Instance.Reset();
		Serial.printf("stop mvt\r\n");
	});

	REGISTER_COMMAND("sendServoPacket", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		int id = atoi(_argv[0]);
		int adr = atoi(_argv[1]);
		int val = atoi(_argv[2]);
		Platform::SendServoPacket(id, (XL320::Address)adr, val);
		Serial.printf("packet id:%d adr:%d val:%d\r\n", id, adr, val);
	});

	REGISTER_COMMAND("setSpeedRatio", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.SetSpeedRatio(value);
		Serial.printf("Speed: %f\r\n", value);
	});

	REGISTER_COMMAND("setPidDistP", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetDistancePID().SetKP(value);
		Serial.printf("Distance P: %f\r\n", value);
	});

	REGISTER_COMMAND("setPidDistI", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetDistancePID().SetKI(value);
		Serial.printf("Distance I: %f\r\n", value);
	});
		
	REGISTER_COMMAND("setPidDistD", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetDistancePID().SetKD(value);
		Serial.printf("Distance D: %f\r\n", value);
	});
	
	REGISTER_COMMAND("setPidAngleP", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetAnglePID().SetKP(value);
		Serial.printf("Angle P: %f\r\n", value);
	});
		
	REGISTER_COMMAND("setPidAngleI", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetAnglePID().SetKI(value);
		Serial.printf("Angle I: %f\r\n", value);
	});
		
	REGISTER_COMMAND("setPidAngleD", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.GetAnglePID().SetKD(value);
		Serial.printf("Angle D: %f\r\n", value);
	});

	REGISTER_COMMAND("setPidMotorP", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		MotorManager::Instance.SetMotorPidP(value);
		Serial.printf("Motor P: %f\r\n", value);
	});

	REGISTER_COMMAND("setPidMotorI", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		MotorManager::Instance.SetMotorPidI(value);
		Serial.printf("Motor I: %f\r\n", value);
	});

	REGISTER_COMMAND("setPidMotorD", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		MotorManager::Instance.SetMotorPidD(value);
		Serial.printf("Motor D: %f\r\n", value);
	});

	REGISTER_COMMAND("setAxleTrack", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		PositionManager::Instance.SetAxleTrackMm(value);
		Serial.printf("Axle track: %f\r\n", value);
	});

	REGISTER_COMMAND("setSpeedDist", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.SetDistanceMaxSpeed(value);
		Serial.printf("distance max speed: %f\r\n", value);
	});

	REGISTER_COMMAND("setSpeedAngle", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.SetAngleMaxSpeed(value);
		Serial.printf("angle max speed: %f\r\n", value);
	});

	REGISTER_COMMAND("setAccDist", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.SetDistanceMaxAcc(value);
		Serial.printf("distance max acceleration: %f\r\n", value);
	});

	REGISTER_COMMAND("setAccAngle", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		float value = atof(_argv[0]);
		ControlSystem::Instance.SetAngleMaxAcc(value);
		Serial.printf("angle max acceleration: %f\r\n", value);
	});

	REGISTER_COMMAND("setServoBaudrate", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::ForceServoBaudRate();
		Serial.print("Force servo baudrate\r\n");
	});

	REGISTER_COMMAND("setServoId", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		int id = atoi(_argv[0]);
		Platform::ForceServoId(id);
		Serial.printf("Force all connected servo to id %d\r\n. Should be the first instruction sent to the servo\r\n", id);
	});

	REGISTER_COMMAND("setSide", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		if (_argv[0][0] == 'b')
			Strategy::Instance.SetSide(Side::BLUE);
		else if (_argv[0][0] == 'y')
			Strategy::Instance.SetSide(Side::YELLOW);
		else
			Serial.print("incorrect side, must be \'b\' or \'y\'\r\n");
		Strategy::Instance.Print();
	});

	REGISTER_COMMAND("start", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
			Strategy::Instance.Start();
	});

	REGISTER_COMMAND("getPos", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Robot pos mm: %f, %f\r\n", PositionManager::Instance.GetXMm(), PositionManager::Instance.GetYMm());
	});
		
	REGISTER_COMMAND("getA", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Robot angle deg: %f\r\n", PositionManager::Instance.GetAngleDeg());
	});
		
	REGISTER_COMMAND("getD", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Robot distance mm: %f\r\n", PositionManager::Instance.GetDistanceMm());
	});

	REGISTER_COMMAND("getEncR", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Right encoder value: %d\r\n", PositionManager::Instance.GetRightEncoder());
	});

	REGISTER_COMMAND("getEncL", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Left encoder value: %d\r\n", PositionManager::Instance.GetLeftEncoder());
	});

	REGISTER_COMMAND("printTrajectory", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		TrajectoryManager::Instance.Print();
	});

	REGISTER_COMMAND("printPID", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Serial.printf("Distance PID: %f, %f, %f\r\n", ControlSystem::Instance.GetDistancePID().GetKP(),
			ControlSystem::Instance.GetDistancePID().GetKI(),
			ControlSystem::Instance.GetDistancePID().GetKd());
		Serial.printf("Angle PID:    %f, %f, %f\r\n", ControlSystem::Instance.GetAnglePID().GetKP(),
			ControlSystem::Instance.GetAnglePID().GetKI(),
			ControlSystem::Instance.GetAnglePID().GetKd());
		Serial.printf("Motors PID:    %f, %f, %f\r\n", MotorManager::Instance.GetLeftMotorPID().GetKP(),
			MotorManager::Instance.GetLeftMotorPID().GetKI(),
			MotorManager::Instance.GetLeftMotorPID().GetKd());
	});

	#ifdef ENABLE_ASTAR
	REGISTER_COMMAND("printGraph", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Graph::Instance.Print();
	});
	#endif

	REGISTER_COMMAND("printServo", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::DebugServoRam(atoi(_argv[0]));
	});
	
	REGISTER_COMMAND("printStrategy", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Strategy::Instance.Print();
	});

	REGISTER_COMMAND("printGP2", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::DebugGP2();
	});

	REGISTER_COMMAND("printButtons", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::DebugButtons();
	});

	REGISTER_COMMAND("printDate", "Date when the program was compiled", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		extern const char* gCompileDate;
		extern const char* gCompileTime;
		Serial.printf("%s - %s\r\n", gCompileDate, gCompileTime);
	});

	REGISTER_COMMAND("servo1", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::InitServo();
		Platform::SetServoPos(ServoID::SERVO1, atoi(_argv[0]));
		Serial.print("Move servo 1\r\n");
	});

	REGISTER_COMMAND("servo2", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::InitServo();
		Platform::SetServoPos(ServoID::SERVO2, atoi(_argv[0]));
		Serial.print("Move servo 2\r\n");
	});

	REGISTER_COMMAND("servo3", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Platform::InitServo();
		Platform::SetServoPos(ServoID::SERVO3, atoi(_argv[0]));
		Serial.print("Move servo 3\r\n");
	});

	REGISTER_COMMAND("pushRobot", "Push robot against wall", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		Strategy::Instance.PushRobotAgainstWall();
	});
	
	REGISTER_COMMAND("disableMotor", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		MotorManager::Instance.Enabled = false;
	});

	REGISTER_COMMAND("help", "", [](const char _argv[CLI_MAX_ARG][CLI_ARG_LENGTH], int) {
		CommandLineInterface::Instance.PrintHelp();
	});

	REGISTER_COMMAND(NULL, NULL, NULL);
}

void CommandLineInterface::Task()
{
	bool end_line = false;
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		if (c == CLI_DEL_CHAR && m_CurPos > 0)
		{
			m_CurPos--;
			Serial.print("\b");
			Serial.print((char)0x1B);
			Serial.print("[K");
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
		Serial.print("\r\n");
		// read the incoming byte:
		char args[CLI_MAX_ARG+1][CLI_ARG_LENGTH] = { 0 };

		int i = 0;
		for (int arg = 0; arg < CLI_MAX_ARG+1; arg++)
		{
			for (int j = 0;
				i < m_CurPos && m_Buffer[i] != ' ' && j < CLI_ARG_LENGTH - 1;
				i++, j++)
			{
				args[arg][j] = m_Buffer[i];
			}
			i++;
		}
		
		//Serial.printf("commande \"%s\", arg: \"%s\" \"%s\"\r\n", args[0], args[1], args[2]);
		bool CmdOk = false;
		for (int c = 0; m_Command[c].cmd; c++)
		{
			if (!stricmp(m_Command[c].cmd, args[0]))
			{
				m_Command[c].function(args + 1, CLI_MAX_ARG);//TODO argc
				CmdOk = true;
				break;
			}
		}

		if (!CmdOk)
			Serial.print("Invalid command\r\n");

		m_CurPos = 0;
	}
}

void CommandLineInterface::PrintHelp()
{
	for (int c = 0; m_Command[c].cmd; c++)
	{
		int CmdLen = Serial.print(m_Command[c].cmd);
		for (int i = 0; i < 20 - CmdLen; i++)
			Serial.print(' ');
		Serial.print(m_Command[c].help);
		Serial.print("\r\n");
	}
}
