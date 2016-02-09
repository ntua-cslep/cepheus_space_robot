#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>

#include "cepheus_hardware.h"


CepheusHW robot;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
  g_request_shutdown = 1;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{   
    double thrust[4];

    thrust[0] = cmd->linear.x;
    thrust[2] = cmd->linear.x;
    thrust[1] = cmd->angular.z;
    thrust[3] = 0;
    robot.setThrustPwm(thrust, 0.87, 0.05, 0.95);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cepheus_interface_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, ctrl_C_Handler);
    ros::NodeHandle nh;
    ros::Rate loop_rate(200.00);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);

    //ros::param::param<int>("~frequency", freq, 10);
    
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();

    while(!g_request_shutdown)
    {
        ros::Time curr_time = ros::Time::now();
        ros::Duration time_step = curr_time - prev_time;
        prev_time = curr_time;

        robot.readEncoders(time_step);
        cm.update(curr_time, time_step);
        robot.writeMotors();


        ros::spinOnce();
        loop_rate.sleep();
    }
    robot.safeClose();
    return 0;
}
