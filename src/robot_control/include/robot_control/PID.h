#ifndef PID_H
#define PID_H

#include <iostream>

//位置式PID
class PID_position
{
private:
    float kp;//比例系数
    float ki;//积分系数
    float kd;//微分系数
    float target;//目标值
    float actual;//实际值
    float e;//误差
    float e_pre;//上一次误差
    float integral;//积分项
    float limit_max;
    float limit_min;
    bool enable_limit;
public:
    PID_position();
    ~PID_position(){};
    PID_position(float p,float i,float d);
    void limit(float min,float max,bool enable_limit);
    float pid_control(float tar,float act);//执行PID控制
    void pid_show();//显示PID控制器的内部参数
};

#endif