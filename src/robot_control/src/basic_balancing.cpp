#include "config.h"
#include "pid.h"
#include "pid_position.hpp"
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
pid::PositionalPid<float> pid_rad(KP_RAD, KI_RAD, KD_RAD);
pid::IncrementalPid<float> pid_vel(KP_VEL, KI_VEL, KD_VEL);
pid::PositionalPid<float> pid_block_pos(KP_BLOCK_POS, KI_BLOCK_POS,
                                        KD_BLOCK_POS);

void DoMsg(sensor_msgs::JointState::ConstPtr ptr) {
  float force_vel = 0, force_rad = 0, force_block_pos = 0;
  if (ptr->position[0] > MIN_RAD && ptr->position[0] < MAX_RAD) {
    // force_rad = pid_rad.Calculate(0, ptr->position[0]);
    force_vel = pid_vel.Calculate(0, ptr->velocity[0]);
    force_block_pos = pid_block_pos.Calculate(0, ptr->position[1]);
    msg.data = force_vel;
  } else {
    ROS_INFO("Out Of Range!!!");
    force_vel = pid_vel.Calculate(0, ptr->position[1]);
    msg.data = force_vel;
  }
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
      "/inverted_pendulum/joint_states", 10, DoMsg);
  ros::spin();
  return 0;
}