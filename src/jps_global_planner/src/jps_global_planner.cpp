#include <pluginlib/class_list_macros.h>
#include "jps_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(jps_global_planner::JPSGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace jps_global_planner {

    JPSGlobalPlanner::JPSGlobalPlanner () {

    }

    JPSGlobalPlanner::JPSGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }

    void JPSGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void JPSGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
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

    bool JPSGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        plan.clear();
        open_set_.clear();
        close_set_.clear();
        edges_.clear();

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

        GridNodePtr start_ptr = new GridNode(start_i, start_x, start_y);
        start_ptr->g_cost_ = 0;
        open_set_.insert(make_pair(0, start_ptr));
        start_ptr->id_ = 1;

        int c = 0;
        int cs = ns_ * 2;

        ROS_INFO("Start Planning... goal_i is %d", goal_i);

        while (!open_set_.empty() && c < cs) {
            GridNodePtr top = open_set_.begin()->second;
            open_set_.erase(open_set_.begin());
            close_set_.insert(top->index_);

            if (top->index_ == goal_i) {
                backtrack(plan, start_i, goal_i);
                ROS_INFO("Over Planning!");
                break;
            }

            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (i == 0 && j == 0)
                        continue;
                    int neig_x = top->x_ + i;
                    int neig_y = top->y_ + j;
                    int neig_i = toIndex(neig_x, neig_y);

                    if(!isCellFree(neig_i))
                        continue;

                    if (close_set_.find(neig_i) != close_set_.end())
                        continue;
                    if (edges_.find(neig_i) != edges_.end())
                        continue;

                    // ROS_INFO("neig_i: %d", neig_i);

                    GridNodePtr neig_ptr = new GridNode(neig_i, neig_x, neig_y);
                    neig_ptr->g_cost_ = top->g_cost_ + manhattanDistance(neig_x, neig_y, top->x_, top->y_);

                    double h_cost = calculateHeuristics(neig_x, neig_y, goal_x, goal_y);
                    double f_cost = neig_ptr->g_cost_ + h_cost;
                    
                    auto it = std::find_if(open_set_.begin(), open_set_.end(), [neig_i](const std::pair<double, GridNodePtr> &e){return e.second->index_ == neig_i;});
                    if (it != open_set_.end()) {
                        if (f_cost < it->first) {
                            open_set_.erase(it);
                            open_set_.insert(make_pair(f_cost, neig_ptr));
                            edges_[neig_i] = top->index_;
                        }
                    } else {
                        open_set_.insert(make_pair(f_cost, neig_ptr));
                        edges_.insert(make_pair(neig_i, top->index_));
                    }
                }
            }
        }

        publishPlan(plan);
        return true;
    }

    void JPSGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
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

    void JPSGlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //set the associated costs in the cost map to be free
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    }
    
    double JPSGlobalPlanner::manhattanDistance(double x1, double y1, double x2, double y2) {
        double res = fabs(x2 - x1) + fabs(y1 - y2);
        return res;
    }

    double JPSGlobalPlanner::euclideanDistance(double x1, double y1, double x2, double y2) {
        double res = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        return res;
    }

    double JPSGlobalPlanner::calculateHeuristics(double x, double y, double goal_x, double goal_y) {
        return euclideanDistance(x, y, goal_x, goal_y);;
    }

    bool JPSGlobalPlanner::isCellFree(int index) {
        if (index < 0 || index >= ns_) {
            return false;
        } else if(costs_[index] >= 10 && costs_[index] != costmap_2d::NO_INFORMATION) {
            return false;
        } else {
            return true;
        }
    }

    bool JPSGlobalPlanner::isCellFree(int x, int y) {
        int tmp_i = toIndex(x, y);
        return isCellFree(tmp_i);
    }

    bool JPSGlobalPlanner::backtrack(std::vector<geometry_msgs::PoseStamped>& plan, int start_i, int goal_i) {
        int curr_i = goal_i;

        geometry_msgs::PoseStamped tmp_pose;

        while (curr_i != start_i) {
            // ROS_INFO("backtrack: curr_i is %d", curr_i);
            tmp_pose.header.frame_id = frame_id_;
            mapToWorld((curr_i % nx_), (curr_i / nx_), tmp_pose.pose.position.x, tmp_pose.pose.position.y);
            // tmp_pose.pose.position.x = (curr_i % nx_) * costmap_->getResolution() + costmap_->getOriginX();
            // tmp_pose.pose.position.y = (curr_i / nx_) * costmap_->getResolution() + costmap_->getOriginY();

            tmp_pose.pose.position.z = 0.0;
            tmp_pose.pose.orientation.w = 1.0;
            tmp_pose.pose.orientation.x = 0.0;
            tmp_pose.pose.orientation.y = 0.0;
            tmp_pose.pose.orientation.z = 0.0;

            plan.push_back(tmp_pose);
            curr_i = edges_[curr_i];
        }

        tmp_pose.header.frame_id = frame_id_;
        mapToWorld((start_i % nx_), (start_i / nx_), tmp_pose.pose.position.x, tmp_pose.pose.position.y);
        // tmp_pose.pose.position.x = (start_i % nx_) * costmap_->getResolution() + costmap_->getOriginX();
        // tmp_pose.pose.position.y = (start_i / nx_) * costmap_->getResolution() + costmap_->getOriginY();

        tmp_pose.pose.position.z = 0.0;
        tmp_pose.pose.orientation.w = 1.0;
        tmp_pose.pose.orientation.x = 0.0;
        tmp_pose.pose.orientation.y = 0.0;
        tmp_pose.pose.orientation.z = 0.0;

        plan.push_back(tmp_pose);
        std::reverse(plan.begin(), plan.end());
        return true;
    }

    void JPSGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
        wx = mx * costmap_->getResolution() + costmap_->getOriginX();
        wy = my * costmap_->getResolution() + costmap_->getOriginY();
    }

    void JPSGlobalPlanner::getDirections(std::vector<std::pair<int, int>>& directions, GridNodePtr curr_ptr, int start_i) {
        directions.clear();
        if (curr_ptr->x_ == start_i % nx_ && curr_ptr->y_ == start_i / nx_) {
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (i == 0 && j == 0) {
                        continue;
                    }
                    if (isCellFree(curr_ptr->x_ + i, curr_ptr->y_ + j)) {
                        directions.push_back(make_pair(i, j));
                    }
                }
            }
        } else {
            int tmp_i = 0;
            std::pair<int, int> pre_direction(1, 1);
            if (abs(curr_ptr->x_ - curr_ptr->came_from_->x_) > 1) {
                pre_direction.first = (curr_ptr->x_ - curr_ptr->came_from_->x_) / abs(curr_ptr->x_ - curr_ptr->came_from_->x_);
            } else {
                pre_direction.first = curr_ptr->x_ - curr_ptr->came_from_->x_;
            }
            if (abs(curr_ptr->y_ - curr_ptr->came_from_->y_) > 1) {
                pre_direction.second = (curr_ptr->y_ - curr_ptr->came_from_->y_) / abs(curr_ptr->y_ - curr_ptr->came_from_->y_);
            } else {
                pre_direction.second = curr_ptr->y_ - curr_ptr->came_from_->y_;
            }

            if (pre_direction.first * pre_direction.second != 0) {
                if (isCellFree(curr_ptr->x_ + pre_direction.first, curr_ptr->y_)) {
                    directions.push_back(make_pair(pre_direction.first, 0));
                }
                if (isCellFree(curr_ptr->x_, curr_ptr->y_ + pre_direction.second)) {
                    directions.push_back(make_pair(0, pre_direction.second));
                }
                if (isCellFree(curr_ptr->x_ + pre_direction.first, curr_ptr->y_ + pre_direction.second)) {
                    directions.push_back(make_pair(pre_direction.first, pre_direction.second));
                }

                tmp_i = toIndex(curr_ptr->x_ - pre_direction.first, curr_ptr->y_);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_, tmp_i / nx_ + pre_direction.second)) {
                        directions.push_back(make_pair(-pre_direction.first, pre_direction.second));
                    }
                }
                tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ - pre_direction.second);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_ + pre_direction.first, tmp_i / nx_)) {
                        directions.push_back(make_pair(pre_direction.first, -pre_direction.second));
                    }
                }
            } else {
                if (pre_direction.first == 0) {
                    if (isCellFree(curr_ptr->x_, curr_ptr->y_ + pre_direction.second)) {
                        directions.push_back(make_pair(0, pre_direction.second));
                    }
                    tmp_i = toIndex(curr_ptr->x_ + 1, curr_ptr->y_);
                    if (!isCellFree(tmp_i)) {
                        if (isCellFree(tmp_i % nx_, tmp_i / nx_ + pre_direction.second)) {
                            directions.push_back(make_pair(1, pre_direction.second));
                        }
                    }
                    tmp_i = toIndex(curr_ptr->x_ - 1, curr_ptr->y_);
                    if (!isCellFree(tmp_i)) {
                        if (isCellFree(tmp_i % nx_, tmp_i / nx_ + pre_direction.second)) {
                            directions.push_back(make_pair(-1, pre_direction.second));
                        }
                    }
                } else {
                    if (isCellFree(curr_ptr->x_ + pre_direction.first, curr_ptr->y_)) {
                        directions.push_back(make_pair(pre_direction.first, 0));
                    }
                    tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ + 1);
                    if (!isCellFree(tmp_i)) {
                        if (isCellFree(tmp_i % nx_ + pre_direction.first, tmp_i / nx_)) {
                            directions.push_back(make_pair(pre_direction.first, 1));
                        }
                    }
                    tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ - 1);
                    if (!isCellFree(tmp_i)) {
                        if (isCellFree(tmp_i % nx_ + pre_direction.first, tmp_i / nx_)) {
                            directions.push_back(make_pair(pre_direction.first, -1));
                        }
                    }
                }
            }
        }
    }

    bool JPSGlobalPlanner::hasForcedNeighbours(GridNodePtr curr_ptr, const std::pair<int, int>& direction) {
        int tmp_i = 0;
        if (direction.first * direction.second != 0) {
            tmp_i = toIndex(curr_ptr->x_ - direction.first, curr_ptr->y_);
            if (!isCellFree(tmp_i)) {
                if (isCellFree(tmp_i % nx_, tmp_i / nx_ + direction.second)) {
                    return true;
                }
            }
            tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ - direction.second);
            if (!isCellFree(tmp_i)) {
                if (isCellFree(tmp_i % nx_ + direction.first, tmp_i / nx_)) {
                    return true;
                }
            }
        } else {
            if (direction.first == 0) {
                tmp_i = toIndex(curr_ptr->x_ + 1, curr_ptr->y_);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_, tmp_i / nx_ + direction.second)) {
                        return true;
                    }
                }
                tmp_i = toIndex(curr_ptr->x_ - 1, curr_ptr->y_);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_, tmp_i / nx_ + direction.second)) {
                        return true;
                    }
                }
            } else {
                tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ + 1);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_ + direction.first, tmp_i / nx_)) {
                        return true;
                    }
                }
                tmp_i = toIndex(curr_ptr->x_, curr_ptr->y_ - 1);
                if (!isCellFree(tmp_i)) {
                    if (isCellFree(tmp_i % nx_ + direction.first, tmp_i / nx_)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    GridNodePtr JPSGlobalPlanner::jump(GridNodePtr curr_ptr, const std::pair<int, int>& direction, int goal_i) {
        int next_i = toIndex(curr_ptr->x_ + direction.first, curr_ptr->y_ + direction.second);
        GridNodePtr next_ptr = new GridNode(next_i, curr_ptr->x_ + direction.first, curr_ptr->y_ + direction.second);
        
        if (!isCellFree(next_i)) {
            return nullptr;
        }
        
        if (next_ptr->x_ == goal_i % nx_ && next_ptr->y_ == goal_i / nx_) {
            return next_ptr;
        }

        if (direction.first * direction.second != 0) {
            if (hasForcedNeighbours(next_ptr, direction)) {
                return next_ptr;
            }
            if (jump(next_ptr, make_pair(direction.first, 0), goal_i) != nullptr) {
                return next_ptr;
            }
            if (jump(next_ptr, make_pair(0, direction.second), goal_i) != nullptr) {
                return next_ptr;
            }
        } else if (direction.second == 0) {
            if (hasForcedNeighbours(next_ptr, direction)) {
                return next_ptr;
            }
        } else if (direction.first == 0) {
            if (hasForcedNeighbours(next_ptr, direction)) {
                return next_ptr;
            }
        }
        
        return jump(next_ptr, direction, goal_i);
    }

};
