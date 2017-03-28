#include "WProgram.h"
#include "MotorManager.h"

const int motorPWMs[] = { 22, 23 };
const int motorDirs[] = { 26, 31 };


void InitMotors()
{
  for(int i = 0; i < NB_MOTORS; ++i) 
  {
    pinMode(motorPWMs[i], OUTPUT);
    pinMode(motorDirs[i], OUTPUT);
  }
}


//void updateEnc() {
 // lastEnc[0] = curEnc[0];
  //lastEnc[1] = curEnc[1];

  //curEnc[0] = -encoder2.calcPosn();
  //curEnc[1] = encoder1.calcPosn();
//}

//static long long lastErr[2] = { 0, 0 };
//static float pidSpeeds[2][3] = {
//  //    P     I     D
//  {   0.5,   0.,  0.5 }, // Droit
//  {   0.5,   0.,  0.5 }, // Gauche
//};

void SendCommandToMotor(int m, long long cmd)
{
  if(m == RIGHT_MOTOR) cmd = -cmd;
  
  analogWrite(motorPWMs[m], (abs(cmd) > 255) ? 255 : abs(cmd));
  digitalWrite(motorDirs[m], (cmd >= 0) ? HIGH : LOW);
}

//void updateMotor(int m, int speed) {
//  long long actualSpeed = curEnc[m] - lastEnc[m];
//  long long err = speed - actualSpeed;
//  
//  long long P = pidSpeeds[m][0] * err;
//  long long I = 0;
//  long long D = pidSpeeds[m][2] * (err - lastErr[m]);
//
//  long long cmd = P + I + D;
//
//  SendCommandToMotor(m, cmd);
//  
//  lastErr[m] = err;
//}
//
//long long lastErrAngle = 0;
//static float pidAngle[3] = {
//  //    P     I     D
//      0.25,   0.,  0.25,
//};
//
//long long lastDistance = 0;
//long long lastErrSpeed = 0;
//static float pidSpeed[3] = {
//  //    P     I     D
//      0.5,   0.,  0.5,
//};
//
//void updateAngleSpeed(int angle, int speed) {
//  long long actualAngle = (curEnc[1] - curEnc[0]) / 2;
//  long long actualDistance = (curEnc[1] + curEnc[0]) / 2;
//
//  long long errAngle = angle - actualAngle;
//
//  long long PAngle = pidAngle[0] * errAngle;
//  long long IAngle = 0;
//  long long DAngle = pidAngle[2] * (errAngle - lastErrAngle);
//
//  long long actualSpeed = lastDistance - actualDistance;
//
//  long long errSpeed = speed - actualSpeed;
//
//  long long PSpeed = pidSpeed[0] * errSpeed;
//  long long ISpeed = 0;
//  long long DSpeed = pidSpeed[2] * (errSpeed - lastErrSpeed);
//
//  updateMotor(1, PSpeed + ISpeed + DSpeed + PAngle + IAngle + DAngle);
//  updateMotor(0, PSpeed + ISpeed + DSpeed - PAngle - IAngle - DAngle);
//
//  lastErrAngle = errAngle;
//  lastDistance = actualDistance;
//  lastErrSpeed = errSpeed;
//}

//void updateMotors(int speed) {
  //updateMotor(0, speed);
  //updateMotor(1, speed);
//}


