#ifndef PID_PID_INCREMENTAL_HPP_
#define PID_PID_INCREMENTAL_HPP_

#include "pid_interface.hpp"

namespace pid {
template <typename T> class IncrementalPid : public PidInterface<T> {
private:
  T kp_;
  T ki_;
  T kd_;
  T target_;
  T actual_;
  T e_;
  T last_e_;
  T past_past_e_;
  T u_;

public:
  IncrementalPid<T>()
      : kp_(0), ki_(0), kd_(0), e_(0), last_e_(0), past_past_e_(0), u_(0) {}
  IncrementalPid<T>(T p, T i, T d)
      : kp_(p), ki_(i), kd_(d), e_(0), last_e_(0), past_past_e_(0), u_(0) {}
  inline bool SetPid(const T p, const T i, const T d) override;
  inline T Proportional() override;
  inline T Integral() override;
  inline T Derivative() override;
  T Calculate(const T target, const T actual) override;
};

template <typename T>
inline bool IncrementalPid<T>::SetPid(const T p, const T i, const T d) {
  kp_ = p;
  ki_ = i;
  kd_ = d;
  return true;
}

template <typename T> T IncrementalPid<T>::Proportional() {
  return e_ - last_e_;
}

template <typename T> T IncrementalPid<T>::Integral() { return e_; }

template <typename T> T IncrementalPid<T>::Derivative() {
  return e_ - 2 * last_e_ + past_past_e_;
}

template <typename T>
T IncrementalPid<T>::Calculate(const T target, const T actual) {
  e_ = target - actual;
  u_ = kp_ * Proportional() + ki_ * Integral() + kd_ * Derivative();
  past_past_e_ = last_e_;
  last_e_ = e_;
  return u_;
}

} // namespace pid

#endif