/**
  *
  * @file      kinematics.cpp
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

#include "kinematics.h"


namespace kinematics
{
  Param::Param(const double & rpm_                /*= 1000*/,
               const double & max_sim_vel_        /*= 3.0*/,
               const double & linear_scaling_     /*= 1.0*/,
               const double & angular_scaling_    /*= 1.0*/,
               const double & max_speed_linear_   /*= 2.0*/,
               const double & max_speed_angular_  /*= 2.0*/,
               const bool   & debug_              /*= true*/,
               const int    & n_motors_           /*= 2*/,
               const int    & mode_               /*= Mode::SIM*/):
    rpm(rpm_),
    max_sim_vel(max_sim_vel_),
    linear_scaling(linear_scaling_),
    angular_scaling(angular_scaling_),
    max_speed_linear(max_speed_linear_),
    max_speed_angular(max_speed_angular_),
    debug(debug_),
    n_motors(n_motors_),
    mode(mode_)
  {}


  void Kinematics::init()
  {
    m_mot_vel.resize(static_cast<size_t>(m_config.n_motors), 0.0);
    return;
  }

  double Kinematics::limitVel(const double &vel_, const double &max_vel_)
  {
    double vel_min_ = std::min(std::fabs(vel_), max_vel_);
    return sgn(vel_) * vel_min_;
  }

  MotVel Kinematics::simKinematics(const CmdVel &cmd_, const Param &config_)
  {
    MotVel mot_vel_ = m_mot_vel;

    ///@todo - generalize linfac gain to avoid specification towards gazebo simulator
    double linfac = 0.4353*std::fabs(cmd_.linear.x())+0.3064;

    //Conversion to m/s for each motor
    mot_vel_[Motor::LEFT]  = linfac*cmd_.linear.x() - config_.angular_scaling*cmd_.angular.z();
    mot_vel_[Motor::RIGHT] = linfac*cmd_.linear.x() + config_.angular_scaling*cmd_.angular.z();

    //Limit per motor velocity
    mot_vel_[Motor::LEFT]  = limitVel(mot_vel_[Motor::LEFT], config_.max_sim_vel);
    mot_vel_[Motor::RIGHT] = limitVel(mot_vel_[Motor::RIGHT], config_.max_sim_vel);

    return mot_vel_;
  }

  MotVel Kinematics::simpleDiffKinematics(const CmdVel &cmd_, const Param &config_)
  {
    MotVel mot_vel_ = m_mot_vel;

    //Conversion to m/s for each motor
    mot_vel_[Motor::LEFT]  = config_.linear_scaling*cmd_.linear.x() - config_.angular_scaling*cmd_.angular.z();
    mot_vel_[Motor::RIGHT] = config_.linear_scaling*cmd_.linear.x() + config_.angular_scaling*cmd_.angular.z();

    //Get linear and angular velocity ratio
    double alpha_ = 0.0;
    if(cmd_.angular.z() == 0.0)
      alpha_ = 1.0;
    else
      alpha_ = std::fabs(cmd_.linear.x()/cmd_.angular.z());

    //Get maximum reachable per motor velocity
    double max_motor_vel_ = alpha_*config_.linear_scaling*config_.max_speed_linear + (1.0-alpha_)*config_.angular_scaling*config_.max_speed_angular;

    //Normalize for RPM
    mot_vel_[Motor::LEFT]  *= (config_.rpm/max_motor_vel_);
    mot_vel_[Motor::RIGHT] *= (config_.rpm/max_motor_vel_);

    //Limit to maximum RPM reference
    mot_vel_[Motor::LEFT]  = limitVel(mot_vel_[Motor::LEFT], config_.rpm);
    mot_vel_[Motor::RIGHT] = limitVel(mot_vel_[Motor::RIGHT], config_.rpm);

    //Conversion from RPM to rad/s
    mot_vel_[Motor::LEFT]  *= (2.0*M_PI/60.0);
    mot_vel_[Motor::RIGHT] *= (2.0*M_PI/60.0);

    return mot_vel_;
  }

  MotVel Kinematics::getMotVel(const CmdVel &cmd_vel_)
  {
    MotVel mot_vel_ = m_mot_vel;
    CmdVel cmd_ = cmd_vel_;

    if(cmd_.linear.x()  == 0.0 && cmd_.linear.y()  == 0.0 && cmd_.linear.z()  == 0.0 &&
       cmd_.angular.x() == 0.0 && cmd_.angular.y() == 0.0 && cmd_.angular.z() == 0.0)
      return mot_vel_;

    cmd_.linear.x()  = limitVel(cmd_.linear.x(), m_config.max_speed_linear);
    cmd_.linear.y()  = limitVel(cmd_.linear.y(), m_config.max_speed_linear);
    cmd_.linear.z()  = limitVel(cmd_.linear.z(), m_config.max_speed_linear);

    cmd_.angular.x() = limitVel(cmd_.angular.x(), m_config.max_speed_angular);
    cmd_.angular.y() = limitVel(cmd_.angular.y(), m_config.max_speed_angular);
    cmd_.angular.z() = limitVel(cmd_.angular.z(), m_config.max_speed_angular);


    if(m_config.mode == Mode::SIM)
      mot_vel_ = simKinematics(cmd_, m_config);
    else if(m_config.mode == Mode::SIMPLE_DIFF)
      mot_vel_ = simpleDiffKinematics(cmd_, m_config);

    return mot_vel_;
  }
}
