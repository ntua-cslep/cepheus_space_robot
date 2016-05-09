#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>

geometry_msgs::TransformStamped ps_transform;
geometry_msgs::PoseStamped cmd_pose;

nav_msgs::Path path;
Eigen::MatrixXd path_matrix;

double path_t_step;
double target_speed;
double target_accel;
bool new_path=false;

void PhaseSpaceCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    ps_transform = *msg;
    return;
}

void moveBaseSimpleCallback(const geometry_msgs::PoseStamped::ConstPtr& cmd_p)
{   
	geometry_msgs::PoseStamped step_pose;
    cmd_pose = *cmd_p;
    path.header.frame_id = cmd_pose.header.frame_id;
    path.header.stamp = cmd_pose.header.stamp;
    //produce the path
    double x0 = ps_transform.transform.translation.x;
    double y0 = ps_transform.transform.translation.y;

    double xf = cmd_pose.pose.position.x;
    double yf = cmd_pose.pose.position.y;

    tf::Quaternion qf(cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z, cmd_pose.pose.orientation.w);
    tf::Matrix3x3 mf(qf);
    double qr,qp,thf; 
    mf.getRPY(qr, qp, thf);

    double heading = atan2(yf-y0 , xf-x0);
	double ax = target_accel*cos(heading);
	double ay = target_accel*sin(heading);
	double t_acc = target_speed/target_accel;

	double t_no_acc = ( xf - x0 - (0.5)*ax*pow(t_acc,2) ) / (target_speed*cos(heading));
	double t_total = t_acc + t_no_acc;

	ROS_INFO_STREAM("t_total: " << t_total << " = "<< t_acc << " + " << t_no_acc);

	int n = (int)(t_total/path_t_step) + 1;
	ROS_INFO_STREAM("Number of position: " << n);

	path_matrix.resize(9, n);

	int count=0;
	for(double t=0; t < t_total; t+=path_t_step)
	{	
		if (t < t_acc) //pose during acceleration
		{
			path_matrix(0,count) = x0 + 0.5*ax*pow(t,2);
			path_matrix(1,count) = y0 + 0.5*ay*pow(t,2);
			path_matrix(2,count) = thf;
			path_matrix(3,count) = ax*t;
			path_matrix(4,count) = ay*t;
			path_matrix(5,count) = 0.0;
			path_matrix(6,count) = ax;
			path_matrix(7,count) = ay;
			path_matrix(8,count) = 0.0;
		}
		else //pose during constant speed
		{

			path_matrix(0,count) = x0 + (0.5)*ax*pow(t_acc,2) + ax*t_acc*(t-t_acc);//x0 + const amount of accel + speed*t-ta
			path_matrix(1,count) = y0 + (0.5)*ay*pow(t_acc,2) + ay*t_acc*(t-t_acc);
			path_matrix(2,count) = thf;
			path_matrix(3,count) = ax*t_acc;
			path_matrix(4,count) = ay*t_acc;
			path_matrix(5,count) = 0.0;
			path_matrix(6,count) = 0.0;
			path_matrix(7,count) = 0.0;
			path_matrix(8,count) = 0.0;
		}
		step_pose.header.seq = (uint)count;
		step_pose.header.frame_id = path.header.frame_id;
		step_pose.header.stamp = path.header.stamp + ros::Duration(t);

		step_pose.pose.position.x = path_matrix(0, count);
		step_pose.pose.position.y = path_matrix(1, count);
		step_pose.pose.position.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromYaw( path_matrix(2,count) );
		step_pose.pose.orientation.x = q.x();
		step_pose.pose.orientation.y = q.y();
		step_pose.pose.orientation.z = q.z();
		step_pose.pose.orientation.w = q.w();
		path.poses.push_back(step_pose);
		
		count++;
		//ROS_INFO_STREAM(count);
	}

	new_path = true;
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
    ros::init(argc, argv, "base_planner_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, ctrl_C_Handler);
    ros::NodeHandle nh;

    ros::ServiceClient controller_srv_client = nh.serviceClient<std_srvs::SetBool>("controller_cmd");

    ros::param::param<double>("~time_step" , path_t_step , 0.1); 
    ros::param::param<double>("~target_speed", target_speed, 0.01); 
    ros::param::param<double>("~target_accel", target_accel, 0.02); 

    ros::Duration time_step(path_t_step);

    ros::Subscriber cmd_pose_sub = nh.subscribe("/move_base_simple/goal", 1, moveBaseSimpleCallback);
    ros::Subscriber phase_space_sub =  nh.subscribe("map_to_cepheus", 1, PhaseSpaceCallback);
       
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1000);

    geometry_msgs::Vector3 cmd_pos;
	geometry_msgs::Vector3 cmd_vel;
	geometry_msgs::Vector3 cmd_acc;

    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("planner_pos", 1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3>("planner_vel", 1);
    ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3>("planner_acc", 1);

    bool path_running=false;
    int cnt;

    while(!g_request_shutdown)
    {
        if (new_path) {
        	ROS_INFO("New Path calculated");
        	new_path=false;
        	path_running=true;
        	cnt = 0;
    		//path_pub.publish(path);

    		//request to enable controller
			std_srvs::SetBool srv;
			srv.request.data = true;
			if (controller_srv_client.call(srv)) {
				ROS_INFO_STREAM("Controller response: " << srv.response.message);
			}
			else{
				ROS_ERROR("Failed to call Controller");
			}
    	}


    	if(path_running) 
    	{
			cmd_pos.x = path_matrix(0,cnt);
			cmd_pos.y = path_matrix(1,cnt);
			cmd_pos.z = path_matrix(2,cnt);
	 		cmd_vel.x = path_matrix(3,cnt);
	 		cmd_vel.y = path_matrix(4,cnt);
	 		cmd_vel.z = path_matrix(5,cnt);
			cmd_acc.x = path_matrix(6,cnt);
	 		cmd_acc.y = path_matrix(7,cnt);
			cmd_acc.z = path_matrix(8,cnt);

			pos_pub.publish(cmd_pos);
			vel_pub.publish(cmd_vel);
			acc_pub.publish(cmd_acc);	

			cnt++;
			if(cnt >= path_matrix.cols()) {
				path_running = false;

				//request to disable controller
				std_srvs::SetBool srv;
				srv.request.data = false;
				if (controller_srv_client.call(srv)) {
					ROS_INFO_STREAM("Controller response: " << srv.response.message);
				}
				else{
					ROS_ERROR("Failed to call Controller");
				}
			}
		}

        ros::spinOnce(); 
        //wait one time step
        time_step.sleep();
    }
    return 0;
}