#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <time.h>

namespace PuzzleBotSimulator{

class PIDController
{
  double u,ek_0,ek_1,ek_2,yk_0,yk_1,yk_2;
  double Kp,Ti,Td;
  double u_min,u_max,u_mid;

  struct timespec start;
  bool first;

public:
  PIDController();

  void SetParameters(double Kp,double Ti,double Td);
  double GetControl(double desired,double system);
  void SetControlLimits(double u_min,double u_max);
  void SetControlMiddle(double u_mid);
};

}

#endif // PIDCONTROLLER_H
