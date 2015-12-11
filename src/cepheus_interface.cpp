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

#include "cepheus_hardware.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cepheus_interface_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(200.00);

    CepheusHW robot;
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();


    while(ros::ok())
    {
        ros::Time curr_time = ros::Time::now();
        ros::Duration period = curr_time - prev_time;
        prev_time = curr_time;

        robot.readEncoders(period);
        cm.update(curr_time, period);
        robot.writeMotors();


        loop_rate.sleep();
    }

    return 0;
}
