/**
  *
  * @file      twist2thrust_node.cpp
  * @version   0.1
  * @date      2019-12-05
  * @authors   Daniel Campos <daniel.f.campos@inesctec.pt>
  *
  * @brief     ROS node for the conversion of twist msgs to thruster velocities
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

#include "twist2thrust_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist2thrust_node");

    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");

    t2t::Twist2Thrust twist2tthrust;

    if(twist2tthrust.setup(n_public, n_private)==t2t::State::OK)
        twist2tthrust.spin();

    return 0;
}
