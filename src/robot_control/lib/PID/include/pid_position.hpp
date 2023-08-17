#ifndef PID_PID_POSITION_HPP_
#define PID_PID_POSITION_HPP_

#include "pid_interface.hpp"
#include <algorithm>

namespace pid {
template <typename T> class PidPosition : public PidInterface<T> {
private:
  T kp_;
  T ki_;
  T kd_;
  T target_;
  T actual_;
  T e_;
  T last_e_;
  T integral_0_t_;
  bool enable_limit_;
  T limit_max_;
  T limit_min_;
  T u_;

public:
  PidPosition<T>()
      : kp_(0), ki_(0), kd_(0), enable_limit_(false), limit_min_(0),
        limit_max_(0), e_(0), last_e_(0), integral_0_t_(0), u_(0) {}
  PidPosition<T>(T p, T i, T d, bool enable = false, T min = 0, T max = 0)
      : kp_(p), ki_(i), kd_(d), enable_limit_(enable), limit_min_(min),
        limit_max_(max), e_(0), last_e_(0), integral_0_t_(0), u_(0) {}
  inline bool SetPid(const T p, const T i, const T d) override;
  inline bool SetLimiterStatus(bool status);
  inline bool SetLimits(T min, T max);
  inline T Proportional() override;
  inline T Integral() override;
  inline T Derivative() override;
  T Calculate(const T target, const T actual) override;
  void Show() override;
};

template <typename T>
inline bool PidPosition<T>::SetPid(const T p, const T i, const T d) {
  kp_ = p;
  ki_ = i;
  kd_ = d;
  return true;
}

template <typename T>
inline bool PidPosition<T>::SetLimiterStatus(bool status) {
  enable_limit_ = status;
  return enable_limit_ == status;
}

template <typename T> inline bool PidPosition<T>::SetLimits(T min, T max) {
  limit_min_ = min;
  limit_max_ = max;
  return true;
}

template <typename T> T PidPosition<T>::Proportional() { return e_; }

template <typename T> T PidPosition<T>::Integral() {
  return integral_0_t_ += e_;
}

template <typename T> T PidPosition<T>::Derivative() {
  auto diff = e_ - last_e_;
  last_e_ = e_;
  return diff;
}

template <typename T>
T PidPosition<T>::Calculate(const T target, const T actual) {
  e_ = target - actual;
  u_ = kp_ * Proportional() + ki_ * Integral() + kd_ * Derivative();
  if (enable_limit_) {
    u_ = std::clamp(u_, limit_min_, limit_max_);
  }
  return u_;
}

} // namespace pid

#endif