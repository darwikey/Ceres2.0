#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#define RIGHT_MOTOR 1
#define LEFT_MOTOR  0
#define NB_MOTORS 2

void InitMotors();
void SendCommandToMotor(int m, long long cmd);


#endif

