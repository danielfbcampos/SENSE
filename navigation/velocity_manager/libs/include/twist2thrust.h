/**
  *
  * @file      twist2thrust.h
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

#ifndef TWIST2THRUST_H
#define TWIST2THRUST_H

///ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

///ROS MSGs includes
#include <geometry_msgs/Twist.h>
#include <roboteq_msgs/Command.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

///LIBRARIES
#include <kinematics.h>
#include "console_log.h"

namespace t2t
{
  enum State { NOK=0, OK };
  enum ROBOTEQ_MODE { MODE_STOPPED=-1, MODE_VELOCITY=0, MODE_POSITION=1 };

  /**
   * @brief The ROSParam class gives the configuration for the ros parts
   */
  class ROSParam
  {
    public:
      ROSParam(const double & max_rpm_            = 1000,
               const double & scale_rpm_          = 1.0,
               const double & rate_               = 40.0,
               const double & timeout_            = 0.5,
               const bool   & is_simulated_       = false,
               const bool   & debug_              = true,
               const bool   & global_verbose_     = true);
      ~ROSParam()=default;

      double max_rpm;           //!< Maximum RPM for each motor
      double scale_rpm;         //!< RPM scale percentage in use
      double rate;              //!< Publishing rate
      double timeout;           //!< No velocity watchdog timeout      
      bool   is_simulated;     //!< Check if it's in simulation mode
      bool   debug;             //!< Debug messages flag
      bool   global_verbose;    //!< Deployment verbosity flag
  };

  /**
   * @brief publishData - data publisher template for any type of msg
   * @param pub_
   * @param data_
   */
  template <class DataType>
  inline void publishData(ros::Publisher& pub_, const DataType& data_)
  {
    if(pub_.getNumSubscribers()>0)
      pub_.publish(data_);

    return;
  }

  class Twist2Thrust
  {
    public:
      //Constructors
      Twist2Thrust()
      {
        init();
      }

      //Destructors
      ~Twist2Thrust()=default; //Avoid memory leaks forcing destructor
      //Getters

      //Setters

      //Callbacks

      //Configurations
      /**
             * @brief setup - Configure ROS parameters, publishers, subscribers, services and parameters
             * @param n_public - public Nodehandle
             * @param n_private - private Nodehandle
             * @return State::OK if everything is ok
             */
      int setup(ros::NodeHandle& n_public, ros::NodeHandle& n_private);

      //Run Functions
      /**
             * @brief spin - Generates rated loop ros::spinOnce() with process execution
             * @return State::OK if first ros::ok() was viable
             */
      int spin();

    private:
      //Ros dependent variables
      ros::Subscriber m_sub_cmd;
      ros::Subscriber m_sub_safety;
      ros::Publisher  m_pub_cmd_left;
      ros::Publisher  m_pub_cmd_right;
      ros::Timer      m_timer;

      //Variables
      ros::Time m_timer_cmd_msg;
      ROSParam  m_ros_params;
      geometry_msgs::Twist m_cmd_vel;
      bool m_safety_stop;

      kinematics::Kinematics m_kin;

      //Functions
      /**
       * @brief init - initializes variables
       */
      void init();


      //Callbacks
      /**
       * @brief cbCMDVel - Subscribes latest velocity command
       * @param msg_
       */
      void cbCMDVel(const geometry_msgs::TwistConstPtr& msg_);

      /**
       * @brief cbSafetyStop - Subscribes latest safety state
       * @param msg_
       */
      void cbSafetyStop(const std_msgs::BoolConstPtr& msg_);

      /**
       * @brief cbTimer0 - Evaluates and publish per motor commands
       */
      void cbTimer0(const ros::TimerEvent&);

    protected:
  };
}
#endif
