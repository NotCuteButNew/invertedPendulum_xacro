#ifndef PID_PID_interface_HPP_
#define PID_PID_interface_HPP_

#include <iostream>
namespace PID {
template <typename T> 
class PID_interface {
private:
  T Kp;
  T Ki;
  T Kd;
  T target; // 目标值
  T actual; // 实际值
  T e;      // 误差
  T u;

public:
  PID_interface() : Kp(0), Ki(0), Kd(0){};
  PID_interface(T p, T i, T d) : Kp(p), Ki(i), Kd(d) {}
  bool setPID(T p, T i, T d);
  virtual T proportional() = 0;
  virtual T integral() = 0;
  virtual T derivative() = 0;
  virtual T calculate(const T target, const T actuall);
  virtual void show();
};

template <typename T>
bool PID_interface<T>::setPID(T p, T i, T d)
{
  Kp=p;
  Ki=i;
  Kd=d;
  return true;
}

template <typename T>
T PID_interface<T>::calculate(const T target, const T actuall) {
  u = Kp * proportional() + Ki * integral() + Kd * derivative();
  return u;
}

template <typename T>
void PID_interface<T>::show() {
  std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
}

} // namespace PID

#endif