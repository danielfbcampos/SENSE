/**
  *
  * @file      twist2thrust.cpp
  * @version   0.1
  * @date      2019-12-05
  * @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
  *
  * @brief     ROS interface for kinematics control for the SENSE ASV
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * @copyright Copyright (c) 2019, INESC TEC - CRAS, All rights reserved.
  *
  */

#include "twist2thrust.h"

namespace t2t
{
  ROSParam::ROSParam(const double & max_rpm_            /*= 1000*/,
                     const double & scale_rpm_          /*= 1.0*/,
                     const double & rate_               /*= 40.0*/,
                     const double & timeout_            /*= 0.5*/,                     
                     const bool   & is_simulated_       /*= false*/,
                     const bool   & debug_              /*= true*/,
                     const bool   & global_verbose_     /*= true*/):
    max_rpm(max_rpm_),
    scale_rpm(scale_rpm_),
    rate(1.0/rate_),
    timeout(timeout_),
    is_simulated(is_simulated_),
    debug(debug_),
    global_verbose(global_verbose_)
  {}

  void Twist2Thrust::init()
  {
    m_cmd_vel.linear.x  = m_cmd_vel.linear.y  = m_cmd_vel.linear.z    = 0.0;
    m_cmd_vel.angular.x = m_cmd_vel.angular.y = m_cmd_vel.angular.z = 0.0;
    m_safety_stop       = true;
  }

  int Twist2Thrust::setup(ros::NodeHandle& n_public, ros::NodeHandle& n_private)
  {
    kinematics::Param kin_param_;

    n_public.param<double> ("max_rpm",        m_ros_params.max_rpm,        1000.0);
    n_public.param<bool>   ("global_verbose", m_ros_params.global_verbose, true);
    n_public.param<bool>   ("use_sim_time",   m_ros_params.is_simulated,   true);

    n_private.param<bool>  ("debug",          m_ros_params.debug,          true);
    m_ros_params.debug = kin_param_.debug = m_ros_params.debug && m_ros_params.global_verbose;

    n_private.param<double>("timeout",        m_ros_params.timeout,        0.5);
    n_private.param<double>("rate",           m_ros_params.rate,           40.0);
    m_ros_params.rate = 1.0/m_ros_params.rate;

    n_private.param<double>("scale_rpm",      m_ros_params.scale_rpm,      0.1);

    if(m_ros_params.scale_rpm > 1.0)
      m_ros_params.scale_rpm = 1.0;
    else if(m_ros_params.scale_rpm < 0.0)
      m_ros_params.scale_rpm = 0.0;

    kin_param_.rpm = m_ros_params.scale_rpm * m_ros_params.max_rpm;

    n_private.param<double>("max_sim_vel",       kin_param_.max_sim_vel,       3.0);
    n_private.param<double>("linear_scaling",    kin_param_.linear_scaling,    1.0);
    n_private.param<double>("angular_scaling",   kin_param_.angular_scaling,   1.0);
    n_private.param<double>("max_speed_linear",  kin_param_.max_speed_linear,  2.0);
    n_private.param<double>("max_speed_angular", kin_param_.max_speed_angular, 2.0);

    if(m_ros_params.is_simulated)
      kin_param_.mode = kinematics::Mode::SIM;
    else
      kin_param_.mode = kinematics::Mode::SIMPLE_DIFF;

    m_kin.configure(kin_param_);

    //SUBSCRIBERS
    std::string topic_sub_cmd;
    n_private.param<std::string>("topic_sub_cmd", topic_sub_cmd, "/cmd_vel");
    m_sub_cmd = n_public.subscribe<geometry_msgs::Twist>(topic_sub_cmd, 1, &Twist2Thrust::cbCMDVel, this);

    std::string topic_sub_safety;
    n_private.param<std::string>("topic_sub_safety", topic_sub_safety, "/safety_stop");
    m_sub_safety= n_public.subscribe<std_msgs::Bool>(topic_sub_safety, 1, &Twist2Thrust::cbSafetyStop, this);

    m_timer = n_public.createTimer(ros::Duration(m_ros_params.rate), &Twist2Thrust::cbTimer0, this);

    //PUBLISHERS
    std::string topic_pub_cmd_left;
    n_private.param<std::string>("topic_pub_cmd_left", topic_pub_cmd_left, "/left/cmd");

    std::string topic_pub_cmd_right;
    n_private.param<std::string>("topic_pub_cmd_right", topic_pub_cmd_right, "/right/cmd");

    if(m_ros_params.is_simulated)
    {
      m_pub_cmd_left  = n_public.advertise<std_msgs::Float32>(topic_pub_cmd_left,  1);
      m_pub_cmd_right = n_public.advertise<std_msgs::Float32>(topic_pub_cmd_right, 1);
    }
    else
    {
      m_pub_cmd_left  = n_public.advertise<roboteq_msgs::Command>(topic_pub_cmd_left,  1);
      m_pub_cmd_right = n_public.advertise<roboteq_msgs::Command>(topic_pub_cmd_right, 1);
    }

    if(m_ros_params.debug)
    {
      CLog::CLog::SetLevel(CLog::CLog::All);
      CLog::CLog::Write(CLog::CLog::Info, "%s## -> Debug info is set true!\n\n%s",             CLog::WHITE.c_str(), CLog::CLEAR.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   ## PARAMETERS CONFIGURATION ##\n%s",           CLog::WHITE.c_str(), CLog::CLEAR.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> topic_sub_cmd is set to: %s%s\n",           CLog::WHITE.c_str(), CLog::CLEAR.c_str(), topic_sub_cmd.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> topic_sub_safety is set to: %s%s\n",        CLog::WHITE.c_str(), CLog::CLEAR.c_str(), topic_sub_safety.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> topic_pub_cmd_left is set to: %s%s\n",      CLog::WHITE.c_str(), CLog::CLEAR.c_str(), topic_pub_cmd_left.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> topic_pub_cmd_right is set to: %s%s\n",     CLog::WHITE.c_str(), CLog::CLEAR.c_str(), topic_pub_cmd_right.c_str());
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> use_sim_time is: %s%s\n",                   CLog::WHITE.c_str(), CLog::CLEAR.c_str(), m_ros_params.is_simulated ? "True" : "False");
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> max_rpm is set to: %s%f rpm\n",             CLog::WHITE.c_str(), CLog::CLEAR.c_str(), m_ros_params.max_rpm);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> scale_rpm is set to: %s%f\n",               CLog::WHITE.c_str(), CLog::CLEAR.c_str(), m_ros_params.scale_rpm);

      CLog::CLog::Write(CLog::CLog::Info, "%s   -> rpm to use is set to: %s%f rpm\n",          CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.rpm);

      CLog::CLog::Write(CLog::CLog::Info, "%s   -> max_sim_vel is set to: %s%f m/s\n",         CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.max_sim_vel);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> rate is set to: %s%f s\n",                  CLog::WHITE.c_str(), CLog::CLEAR.c_str(), m_ros_params.rate);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> timeout is set to: %s%f s\n",               CLog::WHITE.c_str(), CLog::CLEAR.c_str(), m_ros_params.timeout);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> linear_scaling is set to: %s%f\n",          CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.linear_scaling);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> angular_scaling is set to: %s%f\n",         CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.angular_scaling);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> max_speed_linear is set to: %s%f m/s\n",    CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.max_speed_linear);
      CLog::CLog::Write(CLog::CLog::Info, "%s   -> max_speed_angular is set to: %s%f rad/s\n", CLog::WHITE.c_str(), CLog::CLEAR.c_str(), kin_param_.max_speed_angular);
    }
    else
    {
      CLog::CLog::SetLevel(CLog::CLog::Warning);
      CLog::CLog::Write(CLog::CLog::Warning, "%s## -> Debug info is set false!\n\n%s", CLog::YELLOW.c_str(), CLog::CLEAR.c_str());
      CLog::CLog::SetLevel(CLog::CLog::None);
    }

    return State::OK;
  }

  int Twist2Thrust::spin()
  {
    ros::spin();

    return State::NOK;
  }

  void Twist2Thrust::cbCMDVel(const geometry_msgs::TwistConstPtr& msg_)
  {
    m_cmd_vel       = *msg_;
    m_timer_cmd_msg = ros::Time::now();
    return;
  }

  void Twist2Thrust::cbSafetyStop(const std_msgs::BoolConstPtr& msg_)
  {
    m_safety_stop = msg_->data;
    return;
  }

  void Twist2Thrust::cbTimer0(const ros::TimerEvent&)
  {
    std_msgs::Float32 right_, left_;
    roboteq_msgs::Command rob_right_, rob_left_;

    auto vel_pub = [&](float right_vel_, float left_vel_)
    {
      if(m_ros_params.is_simulated)
      {
        right_.data = right_vel_;
        left_.data  = left_vel_;
        publishData<std_msgs::Float32>(m_pub_cmd_right, right_);
        publishData<std_msgs::Float32>(m_pub_cmd_left,  left_);
      }
      else
      {
        rob_right_.setpoint = right_vel_;
        rob_left_.setpoint  = left_vel_;

        if(rob_right_.setpoint == 0.0f && rob_left_.setpoint == 0.0f)
          rob_right_.mode   = rob_left_.mode     = ROBOTEQ_MODE::MODE_STOPPED;
        else
          rob_right_.mode   = rob_left_.mode     = ROBOTEQ_MODE::MODE_VELOCITY;

        publishData<roboteq_msgs::Command>(m_pub_cmd_right, rob_right_);
        publishData<roboteq_msgs::Command>(m_pub_cmd_left,  rob_left_);
      }
    };

    if(m_safety_stop)
    {
      CLog::CLog::Write(CLog::CLog::Fatal, "%s Safety is on! %s\n", CLog::RED.c_str(), CLog::CLEAR.c_str());
      vel_pub(0.0f, 0.0f);
      return;
    }
    else if(ros::Time::now().toSec()-m_timer_cmd_msg.toSec() > m_ros_params.timeout)
    {
      CLog::CLog::Write(CLog::CLog::Fatal, "%s No cmd msg received for %f s%s\n", CLog::RED.c_str(), ros::Time::now().toSec()-m_timer_cmd_msg.toSec(), CLog::CLEAR.c_str());
      vel_pub(0.0f, 0.0f);
      return;
    }

    kinematics::CmdVel cmd_;
    cmd_.linear  = Eigen::Vector3d(m_cmd_vel.linear.x, m_cmd_vel.linear.y, /*m_cmd_vel.linear.z*/0.0);
    cmd_.angular = Eigen::Vector3d(m_cmd_vel.angular.x, m_cmd_vel.angular.y, m_cmd_vel.angular.z);
    kinematics::MotVel mot_ = m_kin.getMotVel(cmd_);

    vel_pub(static_cast<float>(mot_[kinematics::Motor::RIGHT]), static_cast<float>(mot_[kinematics::Motor::LEFT]));

    return;
  }
}
