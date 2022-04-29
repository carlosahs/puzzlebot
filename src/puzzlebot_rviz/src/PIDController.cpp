#include "puzzlebot_rviz/PIDController.hpp"

namespace PuzzleBotSimulator{

PIDController::PIDController()
{
  u = 0;

  Kp = 1;
  Ti = 1;
  Td = 0;

  ek_0 = 0;
  ek_1 = 0;
  ek_2 = 0;

  yk_0 = 0;
  yk_1 = 0;
  yk_2 = 0;

  u_min = -1;
  u_max = 1;

  u_mid = 0;

  first = true;
}

void PIDController::SetParameters(double Kp,double Ti,double Td)
{
  this->Kp = Kp;
  this->Ti = Ti;
  this->Td = Td;
}

void PIDController::SetControlLimits(double u_min, double u_max)
{
  this->u_min = u_min;
  this->u_max = u_max;
}

void PIDController::SetControlMiddle(double u_mid)
{
  this->u_mid = u_mid;
  u = u_mid;
}

double PIDController::GetControl(double desired,double system)
{
  double du,dt;

  if(first)
  {
    clock_gettime(CLOCK_MONOTONIC,&start);
    first = false;
  }
  else
  {

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC,&now);

  dt = (double)(now.tv_sec-start.tv_sec) + (double)(now.tv_nsec-start.tv_nsec)/1e9;
  start = now;

  ek_2 = ek_1;
  ek_1 = ek_0;
  ek_0 = desired - system;

  yk_2 = yk_1;
  yk_1 = yk_0;
  yk_0 = system;

  du = Kp*(ek_0-ek_1) + Kp*dt/Ti*ek_0 + Kp*Td/dt*(yk_0-2*yk_1+yk_2);

  u += du;

  if(u<u_min)
    u=u_min;
  if(u>u_max)
    u=u_max;
  }

  return u;
}
}


