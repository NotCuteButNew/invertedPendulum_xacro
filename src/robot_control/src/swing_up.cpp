#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

double joint2_state;

void doMsg(const sensor_msgs::JointState::ConstPtr& ptr)
{
    auto it=ptr->position.begin()++;
    ROS_INFO("joint2 status:%.2lf\n",*it);
    joint2_state=*it;
    return ;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"swing_up");
    ros::NodeHandle pub_joint1,sub_joint2;
    ros::Publisher pub=pub_joint1.advertise<std_msgs::Float64>("/inverted_pendulum/joint1_velocity_controller/command",10);
    // ros::Subscriber sub=sub_joint2.subscribe<sensor_msgs::JointState>("/inverted_pendulum/joint_states",10,doMsg); 
    ros::Rate rate(5);
    static std_msgs::Float64 vel;
    vel.data=5;
    while (ros::ok()) {
        vel.data*=-1;
        pub.publish(vel);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}