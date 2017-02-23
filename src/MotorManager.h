#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#define RIGHT_MOTOR 0
#define LEFT_MOTOR  1
#define NB_MOTORS 2

void InitMotors();
void SendCommandToMotor(int m, long long cmd);


#endif

