#ifndef PID_interface_HPP
#define PID_interface_HPP

class PID_interface {
private:
  float Kp;
  float Ki;
  float Kd;
  float target; // 目标值
  float actual; // 实际值
  float e;      // 误差
public:
  void setPID(float p, float i, float d);
  virtual float proportional() = 0;
  virtual float integral() = 0;
  virtual float derivative() = 0;
  virtual float calculate();
  virtual void show();
};

#endif