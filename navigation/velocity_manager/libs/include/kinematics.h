/**
  *
  * @file      kinematics.h
  * @version   0.1
  * @date      2019-12-05
  * @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
  *
  * @brief     Kinematic control for the SENSE ASV
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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <tuple>
#include <vector>
#include <memory>

#include <eigen3/Eigen/StdVector>

#include "console_log.h"

namespace kinematics
{
  typedef std::vector<double> MotVel;
  enum State { NOK=0, OK };
  enum Mode  { SIM=0, SIMPLE_DIFF };
  enum Motor { RIGHT=0, LEFT=1 };

  /**
   * @brief The Param class gives the configuration for the kinematics
   */
  class Param
  {
    public:
      Param(const double & rpm_                = 1000,
            const double & max_sim_vel_        = 3.0,
            const double & linear_scaling_     = 1.0,
            const double & angular_scaling_    = 1.0,
            const double & max_speed_linear_   = 2.0,
            const double & max_speed_angular_  = 2.0,
            const bool   & debug_              = true,
            const int    & n_motors_           = 2,
            const int    & mode_               = Mode::SIM);
     // ~Param()=default;

      double rpm;               //!< RPM for each motor to consider
      double max_sim_vel;       //!< Maximum motor velocity allowed for the simulator (m/s)
      double linear_scaling;    //!< Linear velocity scale gain
      double angular_scaling;   //!< Angular velocity scale gain
      double max_speed_linear;  //!< Maximum linear velocity allowed (m/s)
      double max_speed_angular; //!< Maximum angular velocity allowed (rad/s)
      bool   debug;             //!< Debug messages flag
      int    n_motors;          //!< Number of thrusters
      int    mode;              //!< Kinematic mode to consider
  };

  /**
   * @brief The Param class gives the configuration for the kinematics
   */
  class CmdVel
  {
    public:
      typedef std::shared_ptr<CmdVel> Ptr;
      typedef std::shared_ptr<const CmdVel> ConstPtr;

      CmdVel(const Eigen::Vector3d linear_  = Eigen::Vector3d(0.0, 0.0, 0.0),
             const Eigen::Vector3d angular_ = Eigen::Vector3d(0.0, 0.0, 0.0)):
      linear(linear_), angular(angular_){}
      ~CmdVel()=default;

      Eigen::Vector3d linear;
      Eigen::Vector3d angular;
  };

  /**
   * @brief The Kinematics class creates the calculations for each thruster vector
   */
  class Kinematics
  {
    public:
      //Constructors
      Kinematics(){init();}
      Kinematics(Param config_)
      {
        configure(config_);
        init();
      }
      //Destructors
      ~Kinematics()=default; //Avoid memory leaks forcing destructor
      //Getters
      Param const& config(){ return m_config; }

      //Functions
      /**
       * @brief configure - defines private value of configurations
       * @param config_ - Parameters to use
       */
      void configure(const Param& config_ = Param())
      {
        m_config = config_;
        init();
      }

      /**
       * @brief init - initializes variables
       */
      void init();

      /**
       * @brief getMotVel - Calculates the velocities for each motor from the velocity
       * @param cmd_vel - velocity command
       * @return
       */
      MotVel getMotVel(const CmdVel& cmd_vel_);

    private:
      //Variables
      Param m_config;
      MotVel m_mot_vel; //!< Array of floats with the thrusters command

      //Functions
      /**
       * @brief limitVel - Truncates the velocity up to a maximum velocity
       * @param vel_ - velocity value
       * @param max_vel_ - maximum velocity
       * @return
       */
      double limitVel(const double &vel_, const double &max_vel_);

      /**
       * @brief simKinematics - Simulator kinematics calc
       * @param cmd_ - velocity command
       * @param config_ - kinematics config
       * @return
       */
      MotVel simKinematics(const CmdVel &cmd_, const Param &config_);

      /**
       * @brief simpleDiffKinematics - Simple differential drive kinematics calc
       * @param cmd_ - velocity command
       * @param config_ - kinematics config
       * @return
       */
      MotVel simpleDiffKinematics(const CmdVel &cmd_, const Param &config_);

      protected:
      /**
       * @brief
       * sgn - get signal from number
       * @tparam T - Type independent (float, double, int)
       * @param val - number to get signal from
       * @return T
       */
      template <typename T> inline T sgn(T val){ return (T(0) < val) - (val < T(0)); }

  };
}
#endif
