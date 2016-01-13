#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cepheus_interface_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(150.00);

    ros::Publisher FT_pub = n.advertise<geometry_msgs::Wrench>("FT", 1000);
    geometry_msgs::Wrench ft_sensor;

    //int port, baud;

    //ros::param::param<ros::String>("~port", port, "/dev/ttyUSB0");

    int USB = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NDELAY);
    if (USB == -1)
    {
        ROS_ERROR("Error: Could not open serial port.");
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 )
    {
        ROS_ERROR("Error: Could not open serial port.");
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;                  // no remapping, no delays
    tty.c_cc[VMIN]      =   0;                  // read doesn't block
    tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag     &=  ~OPOST;              // make raw

    // Flush Port
    tcflush( USB, TCIFLUSH );
    //applies attributes
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0)
    {
        ROS_ERROR("Error: Could not open serial port.");
    }

    char buf;
    std::string line;
    int count =0;
    int num=1;
    int x,y,z,u,v,w;
    std::string::size_type sz;   // alias of size_t

    while(ros::ok())
    {          
        while(num>0)
        {
        num = read( USB, &buf , 1);
            switch(buf)
            {
                case '\r':
                    break;
                case '\n':
                    break;
                default:
                    line+=buf;
            }
        }

        if (line=="A") count = 0;
        else
        {
            switch(count)
            {
                case 0:
                    x = std::atoi (line.c_str());
                case 1:
                    y = std::atoi (line.c_str());
                case 2:
                    z = std::atoi (line.c_str());
                case 3:
                    u = std::atoi (line.c_str());
                case 4:
                    v = std::atoi (line.c_str());
                case 5:
                    w = std::atoi (line.c_str());
            }
            count++;
        }
        ROS_INFO("x %d", x);

        ft_sensor.force.x =(double)x *0.06;
        ft_sensor.force.y =(double)y *0.06;
        ft_sensor.force.z =(double)z *0.06;
        ft_sensor.torque.x=(double)u *0.06;
        ft_sensor.torque.y=(double)v *0.06;
        ft_sensor.torque.z=(double)w *0.06;

        //FT_pub.publish(ft_sensor);
        //ros::spinOnce();
        loop_rate.sleep();
    }
    close(USB);
    return 0;
}