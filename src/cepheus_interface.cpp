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
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include "cepheus_hardware.h"


CepheusHW robot;
double rw_torque = 0.0;
double rw_cur_vel = 0.0;
double rw_last_vel = 0.0;

void thrusterCallback(const geometry_msgs::Vector3::ConstPtr& cmd)
{
    double thrust[4];
    thrust[0] = (double)cmd->x;
    thrust[1] = (double)cmd->y;
    thrust[2] = (double)cmd->z;
    thrust[3] = (double)0;
    robot.setThrustPwm(thrust, 0.001, 0.9);
    return;
}

void torqueCallback(const std_msgs::Float64::ConstPtr& cmd)
{
    rw_torque = (double)cmd->data;
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void ctrl_C_Handler(int sig)
{
  g_request_shutdown = 1;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cepheus_interface_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, ctrl_C_Handler);
    ros::NodeHandle nh;

    double rate;
    ros::param::param<double>("~loop_rate", rate, 500); 
    ros::Rate loop_rate(rate);

    double max_thrust;
    double rw_max_torque, rw_max_speed, rw_max_power, rw_total_inertia;
    ros::param::param<double>("~thruster_force", max_thrust, 1.5); //the thrust of an open thruster in Newtons
    //ros::param::param<double>("~max_motor_current", max_cur, 1.72); //the max current of the motor
    ros::param::param<double>("~rw_max_torque", rw_max_torque, 0.5); 
    ros::param::param<double>("~rw_max_speed",  rw_max_speed, 100); 
    ros::param::param<double>("~rw_max_power",  rw_max_power, 60); 
    ros::param::param<double>("~rw_total_inertia", rw_total_inertia, 0.00197265); 

    double max_cur[8];
    max_cur[0] = 1.72;
    max_cur[1] = 3.1;
    max_cur[2] = 3.1;
    max_cur[3] = 3.1;
    max_cur[4] = 3.1;
    max_cur[5] = 3.1;
    max_cur[6] = 3.1;
    max_cur[7] = 3.1;

    robot.setParam(max_cur, max_thrust);

    ros::Subscriber thrust_sub =  nh.subscribe("cmd_thrust", 1, thrusterCallback);
    ros::Subscriber torque_sub =  nh.subscribe("cmd_torque", 1, torqueCallback);
    // ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_velocity_controller/command", 1);
    ros::Publisher  torque_pub =  nh.advertise<std_msgs::Float64>("reaction_wheel_effort_controller/command", 1);
    
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();

    while(!g_request_shutdown)
    {
        ros::Time curr_time = ros::Time::now();
        ros::Duration time_step = curr_time - prev_time;
        prev_time = curr_time;

        ros::spinOnce();

        robot.readEncoders(time_step);
        cm.update(curr_time, time_step);
        robot.writeMotors();
        robot.heartbeat();

        // //saturate acceleration og reaction wheel
        // double rw_available_torque;
        // double rw_real_vel = robot.getVel(0);
        // if(fabs(rw_real_vel)<=0.01) {
        //     rw_available_torque = rw_max_torque;
        // }
        // else {
        //     rw_available_torque = rw_max_power/fabs(rw_real_vel);
        // }
        
        // // rw_available_torque = rw_max_torque;
        // if (rw_torque > rw_available_torque) {
        //     rw_torque = rw_available_torque;
        // }
        // else if (rw_torque < -rw_available_torque) {
        //     rw_torque = -rw_available_torque;
        // }
        // //calculate velocity cmd for reaction wheel
        // double rw_cmd_vel;
        // rw_cmd_vel = (rw_torque/rw_total_inertia) * time_step.toSec() + rw_cur_vel;
        // if(fabs(rw_cmd_vel) > rw_max_speed)
        // {
        //     rw_cmd_vel = rw_cur_vel;
        // }
        // rw_cur_vel = rw_cmd_vel;

        // // need rifinement
        // std_msgs::Float64 cmd_rw_vel;
        // cmd_rw_vel.data = -1*rw_cmd_vel;
        if(rw_torque!=0.0) {
            std_msgs::Float64 cmd;
            cmd.data = rw_torque;
            torque_pub.publish(cmd);
            rw_last_vel = robot.getVel(0);
            rw_cur_vel = rw_last_vel;
        }
        else {
            rw_cur_vel = robot.getVel(0);
            double error = rw_last_vel - rw_cur_vel;
            std_msgs::Float64 cmd;
            cmd.data = -10*error;
            torque_pub.publish(cmd);
        }
        // ROS_INFO("cmd: %f", cmd_rw_vel.data);

        loop_rate.sleep();
    }
    robot.safeClose();
    return 0;
}
