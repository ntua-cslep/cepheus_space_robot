#include <ros/ros.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/resource.h>
#include <dm6604.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pthread.h>

#define THRUST 0.87

DM6604Device dac; //0x280 
int count=1;
char port = 0;
double duty[8];

int freq; //hz
int res; //second
ros::Publisher pub12, pub34, pub56;
geometry_msgs::WrenchStamped t12, t34, t56;


void timerCallback(const ros::TimerEvent& event)
{

    for(char i=0; i<8; i++)
    {
        if(count < duty[i]*(double)res) 
            dac.setDigitalPin(PORTA, i);
        else 
            dac.clearDigitalPin(PORTA, i);
    }
    count++;
    if(count == res) count = 1;
    // ROS_INFO("tick-tock");
}

void controllerCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{   

    if (cmd->linear.x >= 0.05)
    {
        duty[0] = cmd->linear.x;
        duty[1] = 0.0;
        duty[4] = cmd->linear.x;
        duty[5] = 0.0;
    }
    else if (cmd->linear.x <=-0.05) 
    {
        duty[0] = 0.0;
        duty[1] = -1*(cmd->linear.x);
        duty[4] = 0.0;
        duty[5] = -1*(cmd->linear.x);
    }
    else
    {
        duty[0] = 0.0;
        duty[1] = 0.0;
        duty[4] = 0.0;
        duty[5] = 0.0;
    }

    if (cmd->angular.z >= 0.05)
    {
        duty[2] = 0.0;
        duty[3] = cmd->angular.z;
    }
    else if (cmd->angular.z <=-0.05)
    {
        duty[2] = -1*(cmd->angular.z);
        duty[3] = 0.0;
    }
    else 
    {
        duty[2] = 0.0;
        duty[3] = 0.0;
    }

    t12.header.stamp = ros::Time::now();
    t34.header.stamp = ros::Time::now();
    t56.header.stamp = ros::Time::now();

    t12.wrench.force.x = (duty[0] - duty[1])*THRUST;
    t56.wrench.force.x = (duty[5] - duty[4])*THRUST;
    t34.wrench.force.x = (duty[3] - duty[2])*THRUST;

    pub12.publish(t12);
    pub34.publish(t34);
    pub56.publish(t56);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "thruster_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    dac.init(640); //0x280 
    dac.configureIOPorts(OUTPUT,OUTPUT,OUTPUT);

    ros::param::param<int>("~frequency", freq, 10);
    ros::param::param<int>("~resolution", res, 100);

    t12.header.frame_id = "T1";
    t34.header.frame_id = "T2";
    t56.header.frame_id = "T3";

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, controllerCallback);
    pub12 = n.advertise<geometry_msgs::WrenchStamped>("thruster12", 1000);
    pub34 = n.advertise<geometry_msgs::WrenchStamped>("thruster34", 1000);
    pub56 = n.advertise<geometry_msgs::WrenchStamped>("thruster56", 1000);

    // Set thread's scheduling to realtime
    struct sched_param param;
    param.sched_priority = 98;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        ROS_ERROR("Couldn't set thruster control to real-time");
    }

    duty[0] = 0;
    duty[1] = 0;
    duty[2] = 0;
    duty[3] = 0;
    duty[4] = 0;
    duty[5] = 0;
    duty[6] = 0;
    duty[7] = 0;


    ros::Timer timer = n.createTimer(ros::Duration(1/(double)(freq*res)), timerCallback);


    ros::spin();

    
    for (int i; i<8; i++) duty[i]=0;
    //dac.writeDigitalPort(PORTA, 0); //shut off all thrusters

    //exit(signum);
    return 0;
}