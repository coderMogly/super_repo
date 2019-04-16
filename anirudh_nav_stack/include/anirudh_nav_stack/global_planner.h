/*********************************************************************
* Author: Anirudh Yadav
*********************************************************************/
#ifndef ANIRUDH_NAV_STACK_GLOBAL_PLANNER_H
#define ANIRUDH_NAV_STACK_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace anirudh_nav_stack
  class GlobalPlanner{
    public:
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost) = 0;

      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      virtual ~GlobalPlanner(){}

    protected:
      GlobalPlanner(){}
  };
};  // namespace anirudh_nav_stack

#endif  // ANIRUDH_NAV_STACK_GLOBAL_PLANNER_H
