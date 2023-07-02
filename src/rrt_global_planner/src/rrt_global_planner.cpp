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
            path_nodes_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("path_nodes", 1);

            rand_area_ = std::pair<int, int>{0, min(nx_, ny_)};
            expand_dis_ = 10;
            path_resolution_ = 2;
            goal_sample_rate_ = 30;
            max_iter_ = 50000;
            play_area_ = std::vector<int>{0, nx_, 0, ny_};

            srand((int)time(0));

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

        NodePtr start_node(new Node(start_i, start_x, start_y));
        NodePtr goal_node(new Node(goal_i, goal_x, goal_y));

        std::vector<NodePtr> node_list = {start_node};
        
        for (int i = 0; i < max_iter_; ++i) {
            NodePtr rnd_node = getRandomNode(goal_i);
            int nearest_ind = getNearestNodeIndex(node_list, rnd_node);
            NodePtr nearest_node = node_list[nearest_ind];

            NodePtr new_node = steer(nearest_node, rnd_node);

            if (checkIfOutsidePlayArea(new_node) && checkCollision(new_node)) {
                node_list.push_back(new_node);
            }

            if (calcDist2Goal(node_list.back()->x_, node_list.back()->y_, goal_i) <= expand_dis_) {
                NodePtr final_node = steer(node_list.back(), goal_node);
                if (checkCollision(final_node)) {
                    node_list.push_back(final_node);
                    generateFinalCourse(plan, node_list);
                    break;
                }
            }
        }

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

    void RRTGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
        wx = mx * costmap_->getResolution() + costmap_->getOriginX();
        wy = my * costmap_->getResolution() + costmap_->getOriginY();
    }
    
    double RRTGlobalPlanner::manhattanDistance(double x1, double y1, double x2, double y2) {
        double res = fabs(x2 - x1) + fabs(y1 - y2);
        return res;
    }

    double RRTGlobalPlanner::euclideanDistance(double x1, double y1, double x2, double y2) {
        double res = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        return res;
    }

    NodePtr RRTGlobalPlanner::steer(const NodePtr& from_node, const NodePtr& to_node) {

        NodePtr new_node(new Node(from_node->index_, from_node->x_, from_node->y_));
        std::pair<double, double> dist_and_theta = clacDistanceAndAngle(new_node, to_node);

        new_node->path_x_.push_back(new_node->x_);
        new_node->path_y_.push_back(new_node->y_);

        double extend_length = expand_dis_;
        if (extend_length > dist_and_theta.first) {
            extend_length = dist_and_theta.first;
        }

        int n_expand = extend_length / path_resolution_;

        for (int i = 0; i < n_expand; ++i) {
            new_node->x_ = new_node->x_ + path_resolution_ * cos(dist_and_theta.second);
            new_node->y_ = new_node->y_ + path_resolution_ * sin(dist_and_theta.second);
            new_node->path_x_.push_back(new_node->x_);
            new_node->path_y_.push_back(new_node->y_);
        }

        dist_and_theta = clacDistanceAndAngle(new_node, to_node);
        if (dist_and_theta.first <= path_resolution_) {
            new_node->path_x_.push_back(to_node->x_);
            new_node->path_y_.push_back(to_node->y_);
            new_node->x_ = to_node->x_;
            new_node->y_ = to_node->y_;
        }

        new_node->index_ = toIndex(new_node->x_, new_node->y_);
        new_node->came_from_ = from_node;

        return new_node;
    }

    void RRTGlobalPlanner::generateFinalCourse(std::vector<geometry_msgs::PoseStamped>& plan, const std::vector<NodePtr>& node_list) {
        NodePtr node = node_list.back();
        geometry_msgs::PoseStamped tmp_pose;
        visualization_msgs::MarkerArray path_nodes;
        int cnt = 0;
        ros::Time time = ros::Time::now();
        do {
            visualization_msgs::Marker path_node;
            path_node.header.frame_id = frame_id_;
            path_node.header.stamp = time;
            path_node.id = cnt++;
            path_node.type = visualization_msgs::Marker::SPHERE;
            path_node.scale.x = 0.1;
            path_node.scale.y = 0.1;
            path_node.scale.z = 0.1;
            path_node.color.a = 1.0;
            path_node.color.r = 1.0;
            path_node.pose.orientation.w = 1.0;
            path_node.pose.orientation.x = 0.0;
            path_node.pose.orientation.y = 0.0;
            path_node.pose.orientation.z = 0.0;
            mapToWorld(node->x_, node->y_, path_node.pose.position.x, path_node.pose.position.y);
            path_nodes.markers.push_back(path_node);

            tmp_pose.header.frame_id = frame_id_;
            mapToWorld(node->x_, node->y_, tmp_pose.pose.position.x, tmp_pose.pose.position.y);
            
            tmp_pose.pose.position.z = 0.0;
            tmp_pose.pose.orientation.w = 1.0;
            tmp_pose.pose.orientation.x = 0.0;
            tmp_pose.pose.orientation.y = 0.0;
            tmp_pose.pose.orientation.z = 0.0;

            plan.push_back(tmp_pose);

            node = node->came_from_;
        } while (!(node->x_ == node_list[0]->x_ && node->y_ == node_list[0]->y_));

        std::reverse(plan.begin(), plan.end());

        path_nodes_pub_.publish(path_nodes);
    }

    double RRTGlobalPlanner::calcDist2Goal(int x, int y, int goal_i) {
        double dx = x - goal_i % nx_;
        double dy = y - goal_i / nx_;
        double dist = hypot(dx, dy);
        return dist;
    }

    NodePtr RRTGlobalPlanner::getRandomNode(int goal_i) {

        NodePtr rnd(new Node());

        if (rand() % 100 > goal_sample_rate_) {
            std::random_device rd;
            std::mt19937 gen(rd());
            
            std::uniform_real_distribution<> x(0, nx_);
            std::uniform_real_distribution<> y(0, ny_);
        
            rnd->x_ = x(gen);
            rnd->y_ = y(gen);
            rnd->index_ = toIndex(rnd->x_, rnd->y_);
            
            return rnd;
        } else {
            rnd->x_ = goal_i % nx_;
            rnd->y_ = goal_i / nx_;
            rnd->index_ = goal_i;

            return rnd;
        }

    }

    int RRTGlobalPlanner::getNearestNodeIndex(const std::vector<NodePtr>& node_list, const NodePtr& rnd_node) {
        int min_ind = 0;
        double min_dist = 10000;
        int p_count = node_list.size();
        for (int i = 0; i < p_count; ++i) {
            double dx = node_list[i]->x_ - rnd_node->x_;
            double dy = node_list[i]->y_ - rnd_node->y_;
            double dist = hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                min_ind = i;
            }
        }
        return min_ind;
    }

    bool RRTGlobalPlanner::checkIfOutsidePlayArea(const NodePtr& node) {
        if (play_area_.size() < 4) {
            return true;
        }
        if (node->x_ < play_area_[0] || node->x_ > play_area_[1] || node->y_ < play_area_[2] || node->y_ > play_area_[3]) {
            return false;
        } else {
            return true;
        }
    }

    bool RRTGlobalPlanner::checkCollision(const NodePtr& node) {
        int p_count = node->path_x_.size();
        for (int i = 0; i < p_count; ++i) {
            if (!isCellFree(node->path_x_[i], node->path_y_[i])) {
                return false;
            }
        }
        return true;
    }

    std::pair<double, double> RRTGlobalPlanner::clacDistanceAndAngle(const NodePtr& from_node, const NodePtr& to_node) {
        double dx = to_node->x_ - from_node->x_;
        double dy = to_node->y_ - from_node->y_;
        double d = hypot(dx, dy);
        double theta = atan2(dy, dx);
        std::pair<double, double> res(d, theta);
        return res;
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

};
