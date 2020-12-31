#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "diagnostic_tools.h"

geometry_msgs::Twist _cmd_msg;
geometry_msgs::Twist _cmd_rc, _cmd_joy, _cmd_auto;

bool _is_rc, _is_joy, _is_auto;

double _curr_time_rc, _curr_time_joy, _curr_time_auto;

ros::Publisher cmd_vel_pub;

void rcCmdVelCallback(const geometry_msgs::Twist &msg)
{
  _cmd_rc=msg;

  if(_cmd_rc.linear.z==1.0)
    _is_rc = true;
  else
    _is_rc = false;

  _curr_time_rc = ros::Time::now().toSec();
}

void joyCmdVelCallback(const geometry_msgs::Twist &msg)
{
  _cmd_joy=msg;

  if(_cmd_joy.linear.z==1.0)
    _is_joy = true;
  else
    _is_joy = false;

  _curr_time_joy = ros::Time::now().toSec();
}

void autoCmdVelCallback(const geometry_msgs::Twist &msg)
{
  _cmd_auto = msg;

  _is_auto = true;

  _curr_time_auto = ros::Time::now().toSec();
}

/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "velocity_priority");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  diag_tools::DiagnosticTools diagnostic(n, "Final Velocity Cmd", "velocity_priority");

  std::string topic_velocity_rc_sub, topic_velocity_joy_sub, topic_velocity_auto_sub, topic_velocity_pub;

  n.param<std::string>("topic_velocity_pub",topic_velocity_pub, "/cmd_vel");
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic_velocity_pub, 1);

  n.param<std::string>("topic_velocity_rc_sub", topic_velocity_rc_sub, "/cmd_vel_rc");
  n.param<std::string>("topic_velocity_joy_sub", topic_velocity_joy_sub, "/cmd_vel_joy");
  n.param<std::string>("topic_velocity_auto_sub", topic_velocity_auto_sub, "/cmd_vel_auto");

  ros::Subscriber cmd_vel_rc=nh.subscribe(topic_velocity_rc_sub, 1, rcCmdVelCallback);
  ros::Subscriber cmd_vel_joy=nh.subscribe(topic_velocity_joy_sub, 1, joyCmdVelCallback);
  ros::Subscriber cmd_vel_auto=nh.subscribe(topic_velocity_auto_sub, 1, autoCmdVelCallback);

  double timeout_rc, timeout_joy, timeout_auto;

  n.param<double>("timeout_rc",timeout_rc, 0.5);
  n.param<double>("timeout_joy",timeout_joy, 0.5);
  n.param<double>("timeout_auto",timeout_auto, 0.5);

  double frate;
  n.param<double>("freq_rate", frate, 40.0);

  bool debug, global_verbose;
  nh.param<bool>("global_verbose", global_verbose, true);
  n.param<bool>("debug", debug, true);
  debug = debug && global_verbose;

  _curr_time_rc = _curr_time_joy = _curr_time_auto = ros::Time::now().toSec();

  _is_rc = _is_joy = _is_auto = false;

  geometry_msgs::Twist stop_vel;

  stop_vel.linear.x             = 0.0;
  stop_vel.linear.y             = 0.0;
  stop_vel.linear.z             = 0.0;

  stop_vel.angular.x            = 0.0;
  stop_vel.angular.y            = 0.0;
  stop_vel.angular.z            = 0.0;

  ros::Rate rate (frate);

  std::string mode_str = "Stop";
  std::string diag_msg;
  char diag_level = diagnostic_msgs::DiagnosticStatus::ERROR;

  while(ros::ok())
  {
    ros::spinOnce();

    if(ros::Time::now().toSec() - _curr_time_rc > timeout_rc)
      _is_rc = false;

    if(ros::Time::now().toSec() - _curr_time_joy > timeout_joy)
      _is_joy = false;

    if(ros::Time::now().toSec() - _curr_time_auto > timeout_auto)
      _is_auto = false;

    if(_is_rc)
    {
      _cmd_msg=_cmd_rc;

      mode_str = "RC";
      diag_level = diagnostic_msgs::DiagnosticStatus::OK;
      diag_msg = "RC is being used!";

      if(debug)
        ROS_INFO_STREAM("RC is being used!");
    }
    else if(_is_joy)
    {
      _cmd_msg=_cmd_joy;

      mode_str = "Joy";
      diag_level = diagnostic_msgs::DiagnosticStatus::OK;
      diag_msg = "Joy is being used!";

      if(debug)
        ROS_INFO_STREAM("Joy is being used!");
    }
    else if(_is_auto)
    {
      _cmd_msg=_cmd_auto;

      mode_str = "Autonomous";
      diag_level = diagnostic_msgs::DiagnosticStatus::OK;
      diag_msg = "Autonomous is being used!";

      if(debug)
        ROS_INFO_STREAM("Auto is being used!");
    }
    else
    {
      _cmd_msg=stop_vel;

      mode_str = "Stop";
      diag_level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_msg = "Stop is being used!";

      if(debug)
        ROS_INFO_STREAM("Stop velocity is being used!");
    }

    diagnostic.setLevel(diag_level, diag_msg);
    diagnostic.add("Mode:", mode_str);
    diagnostic.add("V:", _cmd_msg.linear.x);
    diagnostic.add("W:", _cmd_msg.angular.z);

    diagnostic.publish();

    if(debug)
      ROS_WARN_STREAM("Velocity given is: " << _cmd_msg);

    cmd_vel_pub.publish(_cmd_msg);

    rate.sleep();
  }
}
