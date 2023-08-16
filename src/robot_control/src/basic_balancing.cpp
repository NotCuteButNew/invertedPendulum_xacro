#include "PID.h"
#include "config.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <codecvt>
#include <vector>

ros::Publisher pub;
std_msgs::Float64 msg;
PID_position pid_rad(Kp_rad, Ki_rad, Kd_rad);
PID_position pid_pos(Kp_pos, Ki_pos, Kd_pos);

void doMsg(sensor_msgs::JointState::ConstPtr ptr) {
  float force_pos=0,force_rad=0;
  if (ptr->position[0] > min_rad && ptr->position[0] < max_rad)
    {
      // force_pos=pid_pos.pid_control(0,ptr->position[1]);
      force_pos=0;
      force_rad=-1*pid_rad.pid_control(0, ptr->position[0]);
    }
  else {
    ROS_INFO("Out Of Range!!!");
    force_pos=pid_pos.pid_control(0,ptr->position[1]);
    force_rad=0;
  }
  msg.data=force_pos+force_rad;
  pub.publish(msg);
  ROS_INFO("force:%.2f", msg.data);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "balance");
  ros::NodeHandle pub_joint1, sub_joint2;
  pub = pub_joint1.advertise<std_msgs::Float64>(
    "/inverted_pendulum/joint1_effort_controller/command", 10);
  ros::Subscriber sub = sub_joint2.subscribe<sensor_msgs::JointState>(
      "/inverted_pendulum/joint_states", 10, doMsg);
  ros::spin();
  return 0;
}