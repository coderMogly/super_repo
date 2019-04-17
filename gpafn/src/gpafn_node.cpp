/*********************************************************************
* Author: Anirudh Yadav
*********************************************************************/
#include <gpafn/gpafn_ros.h>
#include <gpafn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace GpaFN {

class GPAFNWithCostmap : public GPAFNROS
{
public:
  GPAFNWithCostmap(string name, Costmap2DROS* cmap);
  bool makePlanService(gpafn::MakeNavPlan::Request& req, gpafn::MakeNavPlan::Response& resp);

private:
  void poseCallback(const rm::PoseStamped::ConstPtr& goal);
  Costmap2DROS* cmap_;
  ros::ServiceServer make_plan_service_;
  ros::Subscriber pose_sub_;
};


bool GPAFNWithCostmap::makePlanService(gpafn::MakeNavPlan::Request& req, gpafn::MakeNavPlan::Response& resp)
{
  vector<PoseStamped> path;

  req.start.header.frame_id = "/map";
  req.goal.header.frame_id = "/map";
  bool success = makePlan(req.start, req.goal, path);
  resp.plan_found = success;
  if (success) {
    resp.path = path;
  }

  return true;
}

void GPAFNWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
    tf::Stamped<tf::Pose> global_pose;
    cmap_->getRobotPose(global_pose);
  vector<PoseStamped> path;
  rm::PoseStamped start;
  start.header.stamp = global_pose.stamp_;
    start.header.frame_id = global_pose.frame_id_;
    start.pose.position.x = global_pose.getOrigin().x();
    start.pose.position.y = global_pose.getOrigin().y();
    start.pose.position.z = global_pose.getOrigin().z();
    start.pose.orientation.x = global_pose.getRotation().x();
    start.pose.orientation.y = global_pose.getRotation().y();
    start.pose.orientation.z = global_pose.getRotation().z();
    start.pose.orientation.w = global_pose.getRotation().w();
    makePlan(start, *goal, path);
}


GPAFNWithCostmap::GPAFNWithCostmap(string name, Costmap2DROS* cmap) : 
  GPAFNROS(name, cmap)
{
  ros::NodeHandle private_nh("~");
  cmap_ = cmap;
  make_plan_service_ = private_nh.advertiseService("make_plan", &GPAFNWithCostmap::makePlanService, this);
  pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &GPAFNWithCostmap::poseCallback, this);
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  tf::TransformListener tf(ros::Duration(10));

  costmap_2d::Costmap2DROS lcr("costmap", tf);

  GpaFN::GPAFNWithCostmap navfn("navfn_planner", &lcr);

  ros::spin();
  return 0;
}
















