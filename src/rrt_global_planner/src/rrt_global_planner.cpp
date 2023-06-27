#include <pluginlib/class_list_macros.h>
#include "rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_global_planner::RRTGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_global_planner {

    RRTGlobalPlanner::RRTGlobalPlanner () {

    }

    RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }

    void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            costmap_ = costmap;
            frame_id_ = frame_id;

            nx_ = costmap_->getSizeInCellsX();
            ny_ = costmap_->getSizeInCellsY();
            ns_ = nx_ * ny_;

            costs_ = costmap_->getCharMap();

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        } else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

    bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        plan.clear();

        ros::NodeHandle nh;
        std::string global_frame = frame_id_;

        if (goal.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != global_frame) {
            ROS_ERROR(
                    "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
            return false;
        }

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        double start_x, start_y, goal_x, goal_y;

        if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
            ROS_WARN_THROTTLE(1.0,
                    "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }
        start_x = start_x_i;
        start_y = start_y_i;

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
            ROS_WARN_THROTTLE(1.0,
                    "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }
        goal_x = goal_x_i;
        goal_y = goal_y_i;

        //clear the starting cell within the costmap because we know it can't be an obstacle
        clearRobotCell(start, start_x_i, start_y_i);

        int start_i = toIndex(start_x, start_y);
        int goal_i = toIndex(goal_x, goal_y);

        publishPlan(plan);
        return true;
    }

    void RRTGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    void RRTGlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    }
    
    double RRTGlobalPlanner::manhattanDistance(double x1, double y1, double x2, double y2) {
        double res = fabs(x2 - x1) + fabs(y1 - y2);
        return res;
    }

    double RRTGlobalPlanner::euclideanDistance(double x1, double y1, double x2, double y2) {
        double res = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        return res;
    }

    double RRTGlobalPlanner::calculateHeuristics(double x, double y, double goal_x, double goal_y) {
        return euclideanDistance(x, y, goal_x, goal_y);;
    }

    bool RRTGlobalPlanner::isCellFree(int index) {
        if (index < 0 || index >= ns_) {
            return false;
        } else if(costs_[index] >= 10 && costs_[index] != costmap_2d::NO_INFORMATION) {
            return false;
        } else {
            return true;
        }
    }

    bool RRTGlobalPlanner::isCellFree(int x, int y) {
        int tmp_i = toIndex(x, y);
        return isCellFree(tmp_i);
    }

    void RRTGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
        wx = mx * costmap_->getResolution() + costmap_->getOriginX();
        wy = my * costmap_->getResolution() + costmap_->getOriginY();
    }

};
