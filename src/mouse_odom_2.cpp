#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include <kdl_parser/kdl_parser.hpp>

#define L 0
#define R 1
#define F 2

class MouseOdom{
public:
  MouseOdom(std::string device, std::string frame)
  {
    this->ready = 0;
    this->dev = device;
    this->data.header.frame_id = frame;
    //openDevice();
  }

  bool openDevice()
  {
    if((this->fd = open(this->dev.c_str(), O_RDONLY | O_NONBLOCK )) == -1) 
    {
      ROS_ERROR("Mevice %s can't open. The device may not exist or you dont have the permission to access it \nIf exist try to run ros as root or this command: sudo chmod a+rw %s",this->dev.c_str(),this->dev.c_str());
      exit(EXIT_FAILURE);
      return 1;
    }
    else
    {
      ROS_INFO("Mouse %s open OK",this->dev.c_str());
      return 0;
    }
  }

  void closeDevice()
  {
    close(this->fd);
  }

  bool readRelativeMove()
  {
    this->ready = 0;
    if(read(this->fd, &this->ie, sizeof(struct input_event))!=-1)
    {
      //ROS_INFO("time %ld.%06ld\ttype %d\tcode %d\tvalue %d", ie.time.tv_sec, ie.time.tv_usec, ie.type, ie.code, ie.value);
      if (this->ie.type == EV_REL) //update x or y 
      {
        if (this->ie.code == REL_X)  this->dy = (float)(-this->ie.value);//its opposite and switched axes
        if (this->ie.code == REL_Y)  this->dx = (float)(-this->ie.value);
      }  
      else if (ie.type == EV_SYN) //update mouse position and reset for next iteration 
      {
        this->data.header.stamp.sec  = (int)this->ie.time.tv_sec;
        this->data.header.stamp.nsec = (int)this->ie.time.tv_usec*1000;
        this->data.vector.y = this->dy;
        this->data.vector.x = this->dx;
        //ROS_INFO("Data ready: time %f X %f Y %f", this->data.header.stamp.toSec(), this->data.vector.x, data.vector.y);
        //reset 
        this->dy=0.0;
        this->dx=0.0;
        this->ready = 1;
      }         
    }
    return this->ready;
  }

  geometry_msgs::Vector3Stamped getVector()
  {
    if (this->ready)
    {
      this->ready = 0;
      return this->data;  
    }
    else
    {
      this->data.header.stamp = ros::Time::now();
      this->data.vector.y = 0.0;
      this->data.vector.x = 0.0;
      return this->data;  
    }
  }

private:
  int fd;
  float dx ,dy;
  bool ready;
  std::string dev;
  struct input_event ie;
  geometry_msgs::Vector3Stamped data;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mouse_odom_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1000);

    //ros parameters
    std::string dev[2];
    ros::param::param<std::string>("~left_dev_event", dev[L], "/dev/input/event3"); //left mouse
    ros::param::param<std::string>("~right_dev_event", dev[R], "/dev/input/event6"); //right mouse
    std::string frame[2];
    ros::param::param<std::string>("~left_frame_id", frame[L], "left_mouse");
    ros::param::param<std::string>("~right_frame_id", frame[R], "right_mouse");
    float cpi;
    ros::param::param<float>("~counts_per_meter", cpi, 800);
    //publishing odom
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("mouse/odom", 1000);


    // //transforms
    // tf::StampedTransform base_to_front_mouse;
    // tf::StampedTransform base_to_left_mouse;

    // tf::TransformListener f_listener, l_listener;
    // ros::Duration(5.0).sleep();

    // try{
    //   f_listener.lookupTransform("base_link", "front_mouse", ros::Time(0), base_to_front_mouse);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    // try{
    //   l_listener.lookupTransform("base_link", "left_mouse", ros::Time(0), base_to_left_mouse);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    // tf::Vector3 front_mouse_vector = base_to_front_mouse.getOrigin();
    // tf::Vector3  left_mouse_vector = base_to_left_mouse.getOrigin();

    // ROS_WARN("origin: x %d, y %d", (int)(front_mouse_vector.x()*100), (int)(front_mouse_vector.y()*100));
    // ROS_WARN("origin: x %d, y %d", (int)(left_mouse_vector.x()*100), (int)(left_mouse_vector.y()*100));

    // //KDL PARSER
    // KDL::Tree my_tree;
    // std::string robot_desc_string;
    // ros::param::param<std::string>("robot_description", robot_desc_string, "string()");
    // if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    // {
    //   ROS_ERROR("Failed to construct kdl tree");
    //   return false;
    // }
    
    //transformations
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_to_base;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";
    odom_to_base.header.stamp = ros::Time::now();
    odom_to_base.transform.translation.x = 0;
    odom_to_base.transform.translation.y = 0;
    odom_to_base.transform.translation.z = 0;
    odom_to_base.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    ros::Time start_time = ros::Time::now();
    
    MouseOdom left_mouse(dev[L], frame[L]);
    MouseOdom right_mouse(dev[R], frame[R]);
    left_mouse.openDevice();
    right_mouse.openDevice();

    geometry_msgs::Vector3Stamped data, data2;
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    double th =0, th1 =0, th2 =0;
    double dx, dy, dth;
    ros::Duration dt;
    ros::Time data_stamp = ros::Time::now() - ros::Duration(1);
    double r1=0, r2=0;
    double scale = cpi/0.0254; //31496.06 : devide counts with this to take meters


    while(ros::ok())
    {
      left_mouse.readRelativeMove();
      right_mouse.readRelativeMove();

      data = left_mouse.getVector();
      data2 = right_mouse.getVector();


      //Stamp data based on latest data from mouses
      if (data2.header.stamp.toSec() > data.header.stamp.toSec())
        dt = data2.header.stamp - data_stamp;
      else
        dt = data.header.stamp - data_stamp;

      data_stamp += dt;
      // ROS_INFO("mouse real data stamp: %f",       data_stamp.toSec());
      // ROS_INFO(" ros::Time data stamp: %f", ros::Time::now().toSec());

      //calculation
      odom.header.stamp = data_stamp;
      dth = atan2( (-data.vector.x + data2.vector.x) , (0.200*39.3700787*cpi) );
      th += dth;

      r1 = pow( (pow( data.vector.x,2) + pow( data.vector.y,2)) ,0.5 );
      r2 = pow( (pow(data2.vector.x,2) + pow(data2.vector.y,2)) ,0.5 );
      th1= atan2( data.vector.y,  data.vector.x);
      th2= atan2(data2.vector.y, data2.vector.x);
      dx = ( r1*cos(th+th1) + r2*cos(th+th2) ) / (2*scale);
      dy = ( r1*sin(th+th1) + r2*sin(th+th2) ) / (2*scale);
      
      odom.pose.pose.position.x += dx;
      odom.pose.pose.position.y += dy;
      odom.twist.twist.linear.x = dx/dt.toSec();
      odom.twist.twist.linear.y = dy/dt.toSec();

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
      odom.pose.pose.orientation = odom_quat;
      ROS_INFO("odom: x %f y %f th %f u %f v %f", odom.pose.pose.position.x, odom.pose.pose.position.y, th, odom.twist.twist.linear.x, odom.twist.twist.linear.y);
  
      odom_pub.publish(odom);
 
      
      odom_to_base.header.stamp = data_stamp;
      odom_to_base.transform.translation.x = odom.pose.pose.position.x;
      odom_to_base.transform.translation.y = odom.pose.pose.position.y;
      odom_to_base.transform.translation.z = 0.0;
      odom_to_base.transform.rotation = odom.pose.pose.orientation;

      odom_broadcaster.sendTransform(odom_to_base);

      ros::spinOnce();
      loop_rate.sleep();
    }
    left_mouse.closeDevice();
    right_mouse.closeDevice();
    return 0;
}
