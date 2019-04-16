/*********************************************************************
* Author: Anirudh Yadav
*********************************************************************/
#ifndef ANIRUDH_NAV_STACK_VELOCITY_CONTROLLER_H
#define ANIRUDH_NAV_STACK_VELOCITY_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace anirudh_nav_stack
  class Velocity_Controller{
    public:

      virtual ~Velocity_Controller(){}

    protected:
      Velocity_Controller(){}
  };
};  // namespace anirudh_nav_stack

#endif  // ANIRUDH_NAV_STACK_VELOCITY_CONTROLLER_H
