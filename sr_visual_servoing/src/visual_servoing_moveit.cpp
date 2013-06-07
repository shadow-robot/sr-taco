/**
 * @file   visual_servoing_moveit.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  7 09:37:10 2013
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Visual servoing using moveit
 *
 *
 */

#include <sr_visual_servoing/visual_servoing_moveit.hpp>

namespace sr_taco
{
  VisualServoing::VisualServoing()
  {
    right_arm_.reset(new move_group_interface::MoveGroup("right_arm"));
  }

  VisualServoing::~VisualServoing()
  {}

  sr_visual_servoing::VisualServoingFeedback VisualServoing::get_closer()
  {
    right_arm_->setRandomTarget();
    right_arm_->move();

    return visual_servoing_feedback_;
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visual_servoing");

  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sr_taco::VisualServoing vs;
  vs.get_closer();

  ros::waitForShutdown();

  return 0;
}



/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
