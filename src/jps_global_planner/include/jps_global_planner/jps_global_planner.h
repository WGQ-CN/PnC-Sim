#ifndef _JPS_GLOBAL_PLANNER_H
#define _JPS_GLOBAL_PLANNER_H
/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

using std::string;

namespace jps_global_planner {

class JPSGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:

        JPSGlobalPlanner();
        JPSGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        double manhattanDistance(double x1, double y1, double x2, double y2);
        double euclideanDistance(double x1, double y1, double x2, double y2);

    protected:
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_;
    
    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);

    };
};

#endif
