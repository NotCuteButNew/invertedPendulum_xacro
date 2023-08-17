#ifndef PID_pid_interface_HPP_
#define PID_pid_interface_HPP_

#include <iostream>
namespace pid {
template <typename T> class PidInterface {
public:
  virtual inline bool SetPid(const T p, const T i, const T d) = 0;
  virtual inline T Proportional() = 0;
  virtual inline T Integral() = 0;
  virtual inline T Derivative() = 0;
  virtual T Calculate(const T target, const T actual) = 0;
};
} // namespace pid
#endif