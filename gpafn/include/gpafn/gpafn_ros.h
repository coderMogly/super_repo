/*********************************************************************
* Author: Anirudh Yadav
*********************************************************************/
#ifndef GPAFN_GPAFN_ROS_H_
#define GPAFN_GPAFN_ROS_H_

#include <ros/ros.h>
#include <gpafn/gpafn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <anirudh_nav_stack/global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <gpafn/potarr_point.h>
#include <pcl_ros/publisher.h>

namespace GpaFN {
  /**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
  class GPAFNROS : public anirudh_nav_stack::GlobalPlanner {
    public:
      /**
       * @brief  Default constructor for the NavFnROS object
       */
      GPAFNROS();

      /**
       * @brief  Constructor for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      GPAFNROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Computes the full navigation function for the map given a point in the world to start from
       * @param world_point The point to use for seeding the navigation function 
       * @return True if the navigation function was computed successfully, false otherwise
       */
      bool computePotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
       * @param goal The goal pose to create a plan to
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return The navigation function's value at that point in the world
       */
      double getPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return True if the navigation function is valid at that point in the world, false otherwise
       */
      bool validPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @param tolerance The tolerance on searching around the world_point specified
       * @return True if the navigation function is valid at that point in the world, false otherwise
       */
      bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~GPAFNROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      costmap_2d::Costmap2D* costmap_;
      boost::shared_ptr<GPAFN> planner_;
      ros::Publisher plan_pub_;
      pcl_ros::Publisher<PotarrPoint> potarr_pub_;
      bool initialized_, allow_unknown_, visualize_potential_;


    private:
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
      }

      void mapToWorld(double mx, double my, double& wx, double& wy);
      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
      double planner_window_x_, planner_window_y_, default_tolerance_;
      std::string tf_prefix_;
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;
      std::string global_frame_;
  };
};

#endif
