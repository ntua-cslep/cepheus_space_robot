#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <angles/angles.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace Eigen;
using namespace angles;
using namespace geometry_msgs;

geometry_msgs::Vector3 real_pos;
geometry_msgs::Vector3 goal_pos;
geometry_msgs::Vector3 cmd_pos;
geometry_msgs::Vector3 cmd_vel;
geometry_msgs::Vector3 cmd_acc;

ros::Time pose_stamp;

bool controller_enabled=false;
bool first_time=true;

bool controllerControl(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
{
  ROS_INFO("requested to: %s the controller", req.data ? "enable" : "disable" );
  if(controller_enabled) {
    if(req.data) {
      first_time=true;
      res.message = "Controller is already enabled";
    }
    else {
      res.message = "Disabling Controller";
    }
  }
  else {
    if(req.data) {
      first_time=true;
      res.message = "Enabling Controller";
    }
    else {
      res.message = "Controller is already disabled";
    }
  }
  controller_enabled = req.data;
  res.success = true;
  return true;
}

class DigitalFilter {
public:
    DigitalFilter(int order, double * xGains, double init_value)
    {   
        len = order + 1;
        x_gain.resize(len);
        x.resize(len);
        y.resize(len);

        for(int i=0;i<len;i++)
        {
            x_gain(i) = xGains[i];
            x(i) = init_value;
            y(i) = init_value;
        }
        // ROS_INFO_STREAM("Filter initialized with:");
        // ROS_INFO_STREAM("gains\n" << x_gain);
        // ROS_INFO_STREAM("x\n" << x);
        // ROS_INFO_STREAM("y\n" << y);
    }

    DigitalFilter(int order, double init_value)
    {
        len = order + 1;
        x_gain.resize(len);
        x.resize(len);
        y.resize(len);

        for(int i=0;i<len;i++)
        {
            x_gain(i) = 1.0/(double)len;
            x(i) = init_value;
            y(i) = init_value;
        }
        // ROS_INFO_STREAM("Simple Filter initialized with:");
        // ROS_INFO_STREAM("gains\n" << x_gain);
        // ROS_INFO_STREAM("x\n" << x);
        // ROS_INFO_STREAM("y\n" << y);
    }

    double filter(double newVal)
    {
        for(int i=(len-1); i>0; i--)//will run len-1 times because the last value will set after
        {
            x(i) = x(i-1);
            y(i) = y(i-1);
        }
        x(0) = newVal;
        y(0) = x * x_gain;
        return y(0);
    }

private:
    int len;
    VectorXd x_gain;
    RowVectorXd x, y;
};


class BaseController {
public:
    BaseController() 
    {
        //initialize params
        x=0;
        y=0;        
        th=0;      

        x_prev = 0.0;
        y_prev = 0.0;
        th_prev = 0.0;

        double g[] = {
          0.00012007374898776981,
          0.0002767496205367667,
          0.00048015900158427614,
          0.000628498993458411,
          0.0005809274633175113,
          0.00019793927061766286,
          -0.00057970940482609,
          -0.0016484394616127838,
          -0.0026971108911555816,
          -0.0032416972306314826,
          -0.00276062573291664,
          -0.0009122418867167238,
          0.002229527763036331,
          0.006018023820277788,
          0.009253615718237242,
          0.010438148878123178,
          0.008244492924719507,
          0.0020903785026598607,
          -0.007366331395230152,
          -0.018013327792288918,
          -0.026500248311975755,
          -0.02891651625209736,
          -0.021808288070764712,
          -0.0032748261203597397,
          0.026185031661902784,
          0.06337320882158314,
          0.10280117944014414,
          0.1377665476108595,
          0.16184932081943304,
          0.17043583689034478,
          0.16184932081943304,
          0.1377665476108595,
          0.10280117944014414,
          0.06337320882158314,
          0.026185031661902784,
          -0.0032748261203597397,
          -0.021808288070764705,
          -0.028916516252097353,
          -0.026500248311975755,
          -0.018013327792288918,
          -0.007366331395230152,
          0.002090378502659864,
          0.008244492924719501,
          0.010438148878123178,
          0.009253615718237242,
          0.006018023820277788,
          0.002229527763036331,
          -0.0009122418867167238,
          -0.00276062573291664,
          -0.0032416972306314826,
          -0.0026971108911555816,
          -0.0016484394616127838,
          -0.0005797094048260938,
          0.00019793927061766663,
          0.0005809274633175151,
          0.0006284989934584072,
          0.00048015900158427614,
          0.0002767496205367667,
          0.00012007374898776981
        };
        int sz = sizeof(g)/sizeof(g[0]);

        // x_fir = new DigitalFilter(sz-1,g,0);
        // y_fir = new DigitalFilter(sz-1,g,0);
        // z_fir = new DigitalFilter(sz-1,g,0);

        x_fir = new DigitalFilter(10,0.0);
        y_fir = new DigitalFilter(10,0.0);
        z_fir = new DigitalFilter(10,0.0);

        xd_fir = new DigitalFilter(10,0.0);
        yd_fir = new DigitalFilter(10,0.0);
        zd_fir = new DigitalFilter(10,0.0);


        rw_present = false;

        Kp << 1.5, 0, 0,
              0, 1.5, 0,
              0, 0, 0.85875;
         //9.2
         //.1035
        Kd << 3.6, 0, 0,
              0, 3.6, 0,
              0,   0, 0.77035;

        r = 0.15; //thrusters radius
        m = 9.2;  //total weight (kg)
        j = 0.5*m*pow(r,2);  //Moment of Inertia

        M << m, 0, 0,
             0, m, 0,
             0, 0, j;

        // the center of mass
        d << -0.015, 0.010 , 0.000;

        //geometry matrix 
        const double f1th =  M_PI/6;
        const double f2th = -M_PI/2;
        const double f3th = -M_PI/6;

        D << cos(f1th), cos(f2th), cos(f3th),
             sin(f1th), sin(f2th), sin(f3th),
                    -r,        -r,         r;

        D_pinv.resize(4,3);
        D_pinv <<   0.5774,  0.3333, -0.1405,
                    0.0000, -0.6667, -0.1405,
                    0.5774, -0.3333,  0.1405,
                   -0.0000,  0.0000,  0.9368;

        D_pinv_noRW <<   0.5774,  0.3333, -2.2222,
                        -0.0000, -0.6667, -2.2222,
                         0.5774, -0.3333,  2.2222;
    }

    void hasReactionWheel(bool val)
    {
        rw_present = val;
    }

    void update(double dt, double pose[3], double des[9], double output[], double err[], Vector3d & F_robot)
    {
        if (dt>0.0000001)
        {
            x = pose[0];
            y = pose[1];
            th= pose[2];

            // double dx = round((x  - x_prev)*10000.0) / 10000.0;
            x = x_fir->filter(x);
            y = y_fir->filter(y);
            th= z_fir->filter(th);

            xd = (x  - x_prev) / dt;
            yd = (y  - y_prev) / dt;
            thd= (th -th_prev) / dt;

            x_prev = x;
            y_prev = y;
            th_prev = th;

            xd = xd_fir->filter(xd);
            yd = yd_fir->filter(yd);
            thd= zd_fir->filter(thd);

            x_des     = des[0];
            y_des     = des[1];
            th_des    = des[2];
            xd_des    = des[3];
            yd_des    = des[4];
            thd_des   = des[5];
            xdd_des   = des[6];
            ydd_des   = des[7];
            thdd_des  = des[8];

            // gain=[1; 1; 1; 10];
            // weight=gain/sum(gain);
            // W=diag(weight);//weight

            Vector3d   e( x_des-x   , y_des-y   , shortest_angular_distance(th,th_des) ); //angle difference super important
            Vector3d  ed( xd_des-xd , yd_des-yd , thd_des-thd );
            Vector3d edd( xdd_des   , ydd_des   , thdd_des );
            
            err[0]=e[0];
            err[1]=e[1];
            err[2]=e[2];
            err[3]=xd;
            err[4]=yd;
            err[5]=thd;
            // err[3]=ed[0];
            // err[4]=ed[1];
            // err[5]=ed[2];
            // ROS_INFO_STREAM("Error\n" << e);
            // ROS_INFO_STREAM("eDot\n" << ed);
            // ROS_INFO_STREAM("eDDot\n" << edd);

            //rotation matrix
            Matrix3d R;
            R <<  cos(th), -sin(th), 0,
                  sin(th),  cos(th), 0,
                  0,        0,       1;
            
            Matrix3d R_trans = R.transpose();

            //Vector3d F_robot;
            F_robot = R_trans*(M*edd + Kd*ed + Kp*e);
            // ROS_INFO_STREAM("F_robot\n" << F_robot);

            if (rw_present) {
                Vector4d F_actuator;
                F_actuator = D_pinv*F_robot;
                //ROS_INFO_STREAM("F_actuator\n" << F_actuator);

                for (int i=0; i<4;i++){
                    output[i]=F_actuator[i];
                }
            }
            else {
                Vector3d F_actuator;
                F_actuator = D_pinv_noRW*F_robot;
                //ROS_INFO_STREAM("F_actuator\n" << F_actuator);

                for (int i=0; i<3;i++){
                    output[i]=F_actuator[i];
                }
                output[3]=0.0;
            }
        }
    }

private:
    double x;
    double y;
    double th;

    DigitalFilter * x_fir;
    DigitalFilter * xd_fir;
    DigitalFilter * y_fir;
    DigitalFilter * yd_fir;
    DigitalFilter * z_fir;
    DigitalFilter * zd_fir;

    double x_prev;
    double y_prev;
    double th_prev;

    double xd; 
    double yd;
    double thd;

    double x_des;
    double y_des;
    double th_des;
    double xd_des;
    double yd_des;
    double thd_des;
    double xdd_des;
    double ydd_des;
    double thdd_des;

    double r;
    double m;
    double j;

    Matrix3d Kp;
    Matrix3d Kd;
    Matrix3d M;
    Vector3d d;
    Matrix3d D;
    MatrixXd D_pinv;
    Matrix3d D_pinv_noRW;

    bool rw_present;
};


BaseController control;


void PhaseSpaceCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  geometry_msgs::TransformStamped temp;
  temp = *msg;
  double x = temp.transform.rotation.x;
  double y = temp.transform.rotation.y;
  double z = temp.transform.rotation.z;
  double w = temp.transform.rotation.w;
  double roll,pitch,yaw;

  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);

  pose_stamp = temp.header.stamp;
  real_pos.x = temp.transform.translation.x;
  real_pos.y = temp.transform.translation.y;
  real_pos.z = yaw;
  //ROS_INFO("phaseSpace called");
  return;
}

void remoteCallback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    geometry_msgs::Twist temp;
    temp = *msg;
    cmd_vel.x = temp.linear.x/100;
    cmd_vel.y = temp.linear.y/100;
    cmd_vel.z = temp.angular.z/10;
    return;
}

void moveBasePlannerGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped temp;
  temp = *msg;

  double x = temp.pose.orientation.x;
  double y = temp.pose.orientation.y;
  double z = temp.pose.orientation.z;
  double w = temp.pose.orientation.w;   
  double roll,pitch,yaw;

  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);

  goal_pos.x = temp.pose.position.x;
  goal_pos.y = temp.pose.position.y;
  goal_pos.z = yaw;  
}

void moveBaseSimpleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped temp;
  temp = *msg;

  double x = temp.pose.orientation.x;
  double y = temp.pose.orientation.y;
  double z = temp.pose.orientation.z;
  double w = temp.pose.orientation.w;   
  double roll,pitch,yaw;

  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);

  cmd_pos.x = temp.pose.position.x;
  cmd_pos.y = temp.pose.position.y;
  cmd_pos.z = yaw;

  return;
}

void plannerPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{   
    cmd_pos = *msg;
    // ROS_INFO_STREAM("cmd_pos recieved");
    return;
}

void plannerVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{   
    cmd_vel = *msg;
    // ROS_INFO_STREAM("cmd_vel recieved");
    return;
}

void plannerAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{   
    cmd_acc = *msg;
    // ROS_INFO_STREAM("cmd_acc recieved");
    return;
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
    ros::init(argc, argv, "base_controller_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, ctrl_C_Handler);
    ros::NodeHandle nh;

    double rate;
    ros::param::param<double>("~loop_rate", rate, 400.0); //the max current of the motor
    ros::Rate loop_rate(rate);
    bool react_wheel;
    ros::param::param<bool>("~use_reaction_wheel", react_wheel, false); //the max current of the motor
    control.hasReactionWheel(react_wheel);
    bool planner_exist;
    ros::param::param<bool>("~use_with_planner", planner_exist, false); //the max current of the motor

    ros::Subscriber cmd_pose_sub;
    ros::Subscriber cmd_pos_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber cmd_acc_sub;
    ros::Subscriber simple_cmd_pose_sub;
    if (planner_exist==true) {
      cmd_pose_sub = nh.subscribe("move_base_simple/goal", 1000, moveBasePlannerGoalCallback);
      cmd_pos_sub = nh.subscribe("planner_pos", 1000, plannerPositionCallback);
      cmd_vel_sub = nh.subscribe("planner_vel", 1000, plannerVelocityCallback);
      cmd_acc_sub = nh.subscribe("planner_acc", 1000, plannerAccelerationCallback);
      ROS_INFO("Controller started with Planner interface");
    }
    else {
      simple_cmd_pose_sub = nh.subscribe("/move_base_simple/goal", 1000, moveBaseSimpleCallback);
      ROS_INFO("Controller started in Standalone mode");
    }
    ros::Subscriber phase_space_sub =  nh.subscribe("map_to_cepheus", 1000, PhaseSpaceCallback);
    ros::Subscriber rmt_vel_sub = nh.subscribe("cmd_vel", 1000, remoteCallback);

    ros::ServiceServer service = nh.advertiseService("controller_cmd", controllerControl);

    double out[4];
    geometry_msgs::Vector3 thrust_vector;
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3>("cmd_thrust", 1);
    std_msgs::Float64 torque;
    // ros::Publisher rwTorque_pub = nh.advertise<std_msgs::Float64>("reaction_wheel_effort_controller/command", 1);
    ros::Publisher rwTorque_pub = nh.advertise<std_msgs::Float64>("cmd_torque", 1);
    
    double error[6];
    geometry_msgs::Vector3 e, ed;
    ros::Publisher  e_pub = nh.advertise<geometry_msgs::Vector3>("error", 1);
    ros::Publisher ed_pub = nh.advertise<geometry_msgs::Vector3>("error_dot", 1);

    Vector3d f_robot;
    geometry_msgs::WrenchStamped frobot;
    ros::Publisher frobot_pub = nh.advertise<geometry_msgs::WrenchStamped>("frobot", 1);

    ros::Time prev_time = ros::Time::now()-ros::Duration(1000000);

    double pose[3];
    double des[9];
        

    ros::Duration(8).sleep(); //wait to recieve phasespace 
    ros::spinOnce();

    while(!g_request_shutdown)
    {
        ros::Duration time_step = pose_stamp - prev_time;
        prev_time = pose_stamp;

        if(first_time==true) {
          //hold the current position
          cmd_pos.x = real_pos.x;
          cmd_pos.y = real_pos.y;
          cmd_pos.z = real_pos.z;
          cmd_vel.x = 0.0;
          cmd_vel.y = 0.0;
          cmd_vel.z = 0.0;
          cmd_acc.x = 0.0;
          cmd_acc.y = 0.0;
          cmd_acc.z = 0.0;
          first_time = false;
          ROS_INFO("Controller Initialized");
        }

        pose[0] = real_pos.x;
        pose[1] = real_pos.y;
        pose[2] = real_pos.z;

        des[0] = cmd_pos.x;
        des[1] = cmd_pos.y;
        des[2] = cmd_pos.z;
        des[3] = cmd_vel.x;
        des[4] = cmd_vel.y;
        des[5] = cmd_vel.z;
        des[6] = cmd_acc.x;
        des[7] = cmd_acc.y;
        des[8] = cmd_acc.z;

        //ROS_INFO("dt: %f, x: %f, y: %f, theta: %f",time_step.toSec(), ps_transform.transform.translation.x, ps_transform.transform.translation.y, theta);
        if(controller_enabled) {

          control.update(time_step.toSec(), pose, des, out, error, f_robot);

          thrust_vector.x = out[0];
          thrust_vector.y = out[1];
          thrust_vector.z = out[2];
          torque.data     = 3*out[3];
          thrust_pub.publish(thrust_vector);
          rwTorque_pub.publish(torque);

          e.x = error[0];
          e.y = error[1];
          e.z = error[2];
          ed.x = error[3];
          ed.y = error[4];
          ed.z = error[5];
          e_pub.publish(e);
          ed_pub.publish(ed);
        }
        else {
          f_robot[0] = 0.0;
          f_robot[1] = 0.0;
          f_robot[2] = 0.0;

          thrust_vector.x = 0.0;
          thrust_vector.y = 0.0;
          thrust_vector.z = 0.0;
          torque.data     = 0.0;
          thrust_pub.publish(thrust_vector);
          rwTorque_pub.publish(torque);
        }

        frobot.header.frame_id = "base_link";
        frobot.header.stamp = pose_stamp;
        frobot.wrench.force.x  = f_robot[0];
        frobot.wrench.force.y  = f_robot[1];
        frobot.wrench.torque.z = f_robot[2];
        frobot_pub.publish(frobot);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}