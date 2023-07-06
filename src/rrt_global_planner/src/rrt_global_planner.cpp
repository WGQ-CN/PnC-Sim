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
            optimize_path_pub_ = private_nh.advertise<nav_msgs::Path>("optimize_path", 1);
            smooth_path_pub_ = private_nh.advertise<nav_msgs::Path>("smooth_path", 1);

            rand_area_ = std::pair<int, int>{0, min(nx_, ny_)};
            expand_dis_ = 30;
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

        optimize_path_.clear();
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
        publishPlanPoints(plan);

        waypointOptimize();
        publishOptimizePath();

        pathSmooth();
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

    void RRTGlobalPlanner::publishPlanPoints(const std::vector<geometry_msgs::PoseStamped>& path) {
        visualization_msgs::MarkerArray path_nodes;
        int p_cnt = path.size();
        ros::Time time = ros::Time::now();
        for (int i = 0; i < p_cnt; ++i) {
            visualization_msgs::Marker path_node;
            path_node.header.frame_id = frame_id_;
            path_node.header.stamp = time;
            path_node.id = i;
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
            path_node.pose.position.x = path[i].pose.position.x;
            path_node.pose.position.y = path[i].pose.position.y;
            path_nodes.markers.push_back(path_node);
        }
        path_nodes_pub_.publish(path_nodes);
    }

    void RRTGlobalPlanner::publishOptimizePath() {
        nav_msgs::Path gui_path;
        gui_path.poses.resize(optimize_path_.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < optimize_path_.size(); i++) {
            gui_path.poses[i].header.frame_id = frame_id_;
            mapToWorld(optimize_path_[i]->x_, optimize_path_[i]->y_, gui_path.poses[i].pose.position.x, gui_path.poses[i].pose.position.y);
            
            gui_path.poses[i].pose.position.z = 0.0;
            gui_path.poses[i].pose.orientation.w = 1.0;
            gui_path.poses[i].pose.orientation.x = 0.0;
            gui_path.poses[i].pose.orientation.y = 0.0;
            gui_path.poses[i].pose.orientation.z = 0.0;
        }

        optimize_path_pub_.publish(gui_path);
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
            // new_node->x_ = round(double(new_node->x_) + double(path_resolution_) * cos(dist_and_theta.second));
            // new_node->y_ = round(double(new_node->y_) + double(path_resolution_) * sin(dist_and_theta.second));
            // new_node->x_ = new_node->x_ + path_resolution_ * cos(dist_and_theta.second) + 0.5;   // 四舍五入会导致movebase崩溃
            // new_node->y_ = new_node->y_ + path_resolution_ * sin(dist_and_theta.second) + 0.5;
            new_node->x_ = new_node->x_ + path_resolution_ * cos(dist_and_theta.second);            // int类型计算会存在bug，距离不准
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
        // visualization_msgs::MarkerArray path_nodes;
        // int cnt = 0;
        // ros::Time time = ros::Time::now();
        do {
            // visualization_msgs::Marker path_node;
            // path_node.header.frame_id = frame_id_;
            // path_node.header.stamp = time;
            // path_node.id = cnt++;
            // path_node.type = visualization_msgs::Marker::SPHERE;
            // path_node.scale.x = 0.1;
            // path_node.scale.y = 0.1;
            // path_node.scale.z = 0.1;
            // path_node.color.a = 1.0;
            // path_node.color.r = 1.0;
            // path_node.pose.orientation.w = 1.0;
            // path_node.pose.orientation.x = 0.0;
            // path_node.pose.orientation.y = 0.0;
            // path_node.pose.orientation.z = 0.0;
            // mapToWorld(node->x_, node->y_, path_node.pose.position.x, path_node.pose.position.y);
            // path_nodes.markers.push_back(path_node);
            optimize_path_.push_back(node);

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

        tmp_pose.header.frame_id = frame_id_;
        mapToWorld(node->x_, node->y_, tmp_pose.pose.position.x, tmp_pose.pose.position.y);
        
        tmp_pose.pose.position.z = 0.0;
        tmp_pose.pose.orientation.w = 1.0;
        tmp_pose.pose.orientation.x = 0.0;
        tmp_pose.pose.orientation.y = 0.0;
        tmp_pose.pose.orientation.z = 0.0;

        optimize_path_.push_back(node);
        plan.push_back(tmp_pose);

        std::reverse(optimize_path_.begin(), optimize_path_.end());
        std::reverse(plan.begin(), plan.end());

        // path_nodes_pub_.publish(path_nodes);
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

    void RRTGlobalPlanner::waypointOptimize() {
        size_t p_count = optimize_path_.size();

        int i = p_count - 1;
        while (i > 1)
        {
            int j = 0;
            while (j <= i - 2)
            {
                if (isPathFree(optimize_path_[j], optimize_path_[i])) {
                    auto it = optimize_path_.begin();
                    for (int k = 0; k < j + 1; ++k) {
                        ++it;
                    }
                    for (int k = j + 1; k <= i - 1; ++k) {
                        it = optimize_path_.erase(it);
                    }
                    i = j;
                    break;
                }
                j = j + 1;
            }
            if (i - 1 == j) i = i - 1;
        }
    }

    bool RRTGlobalPlanner::isPathFree(const NodePtr& from_pose, const NodePtr& to_pose) {
        double dx = to_pose->x_ - from_pose->x_;
        double dy = to_pose->y_ - from_pose->y_;
        double dist = hypot(dx, dy);
        double theta = atan2(dy, dx);
        
        if (!isCellFree(to_pose->x_, to_pose->y_) || !isCellFree(from_pose->x_, from_pose->y_)) {
            return false;
        }

        int n_points = int(floor(dist / path_resolution_));

        double tmp_x = from_pose->x_, tmp_y = from_pose->y_;
        for (int i = 0; i < n_points; ++i) {
            tmp_x = tmp_x + double(path_resolution_) * cos(theta);
            tmp_y = tmp_y + double(path_resolution_) * sin(theta);
            if (!isCellFree(int(round(tmp_x)), int(round(tmp_y)))) {
                return false;
            }
            // if (!isCellFree(int(tmp_x + 0.5), int(tmp_y + 0.5))) {
            //     return false;
            // }
        }
        return true;
    }

    void RRTGlobalPlanner::pathSmooth() {

        std::vector<geometry_msgs::PoseStamped> smooth_path;
        unsigned int optimize_path_cnt = optimize_path_.size();

        for (int i = 0; i < optimize_path_cnt; ++i) {
            geometry_msgs::PoseStamped tmp_pose;
        
            tmp_pose.header.frame_id = frame_id_;
            mapToWorld(optimize_path_[i]->x_, optimize_path_[i]->y_, tmp_pose.pose.position.x, tmp_pose.pose.position.y);
            
            tmp_pose.pose.position.z = 0.0;
            tmp_pose.pose.orientation.w = 1.0;
            tmp_pose.pose.orientation.x = 0.0;
            tmp_pose.pose.orientation.y = 0.0;
            tmp_pose.pose.orientation.z = 0.0;

            smooth_path.push_back(tmp_pose);
        }


        unsigned int smooth_path_cnt = smooth_path.size();
        geometry_msgs::PoseStamped p_start = smooth_path[0];
        geometry_msgs::PoseStamped p_goal = smooth_path[smooth_path_cnt - 1];
        double p_tmp_x = p_start.pose.position.x;
        double p_tmp_y = p_start.pose.position.y;
        std::vector<geometry_msgs::PoseStamped> smooth_path_res = {p_start};

        unsigned int n_points = 10;

        for (int i = 0; i < smooth_path_cnt - 2; ++i) {
            std::vector<geometry_msgs::PoseStamped> P = {smooth_path[i], smooth_path[i + 1], smooth_path[i + 2]};
            std::pair<double, double> p2p1 = {P[0].pose.position.x - P[1].pose.position.x, P[0].pose.position.y - P[1].pose.position.y};
            std::pair<double, double> p2p3 = {P[2].pose.position.x - P[1].pose.position.x, P[2].pose.position.y - P[1].pose.position.y};
            double x1 = p2p1.first;
            double y1 = p2p1.second;
            double x2 = p2p3.first;
            double y2 = p2p3.second;

            geometry_msgs::PoseStamped A, B, C;
            B = P[1];
            if (x1 > 0 && y1 > 0) {
                if (x2 > 0 && y2 > 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[2];
                        C = P[0];
                    } else {
                        A = P[0];
                        C = P[2];
                    }
                } else if (x2 <= 0 && y2 >= 0) {
                    A = P[2];
                    C = P[0];
                } else if (x2 >= 0 && y2 <= 0) {
                    A = P[0];
                    C = P[2];
                } else if (x2 < 0 && y2 < 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[0];
                        C = P[2];
                    } else {
                        A = P[2];
                        C = P[0];
                    }
                }
            } else if (x1 < 0 && y1 > 0) {
                if (x2 < 0 && y2 > 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[2];
                        C = P[0];
                    } else {
                        A = P[0];
                        C = P[2];
                    }
                } else if (x2 >= 0 && y2 >= 0) {
                    A = P[0];
                    C = P[2];
                } else if (x2 <= 0 && y2 <= 0) {
                    A = P[2];
                    C = P[0];
                } else if (x2 > 0 && y2 < 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[0];
                        C = P[2];
                    } else {
                        A = P[2];
                        C = P[0];
                    }
                }
            } else if (x1 < 0 && y1 < 0) {
                if (x2 < 0 && y2 < 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[2];
                        C = P[0];
                    } else {
                        A = P[0];
                        C = P[2];
                    }
                } else if (x2 <= 0 && y2 >= 0) {
                    A = P[0];
                    C = P[2];
                } else if (x2 >= 0 && y2 <= 0) {
                    A = P[2];
                    C = P[0];
                } else if (x2 > 0 && y2 > 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[0];
                        C = P[2];
                    } else {
                        A = P[2];
                        C = P[0];
                    }
                }
            } else if (x1 > 0 && y1 < 0) {
                if (x2 > 0 && y2 < 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[2];
                        C = P[0];
                    } else {
                        A = P[0];
                        C = P[2];
                    }
                } else if (x2 >= 0 && y2 >= 0) {
                    A = P[2];
                    C = P[0];
                } else if (x2 <= 0 && y2 <= 0) {
                    A = P[0];
                    C = P[2];
                } else if (x2 < 0 && y2 > 0) {
                    if (y2 / x2 > y1 / x1) {
                        A = P[0];
                        C = P[2];
                    } else {
                        A = P[2];
                        C = P[0];
                    }
                }
            } else if (x1 == 0 && y1 > 0) {
                if (x2 > 0) {
                    A = P[0];
                    C = P[2];
                } else if (x2 < 0) {
                    A = P[2];
                    C = P[0];
                }
            } else if (x1 == 0 && y1 < 0) {
                if (x2 > 0) {
                    A = P[2];
                    C = P[0];
                } else if (x2 < 0) {
                    A = P[0];
                    C = P[2];
                }
            } else if (x1 > 0 && y1 == 0) {
                if (y2 > 0) {
                    A = P[2];
                    C = P[0];
                } else if (y2 < 0) {
                    A = P[0];
                    C = P[2];
                }
            } else if (x1 < 0 && y1 == 0) {
                if (y2 > 0) {
                    A = P[0];
                    C = P[2];
                } else if (y2 < 0) {
                    A = P[2];
                    C = P[0];
                }
            }

            std::pair<double, double> BA = {A.pose.position.x - B.pose.position.x, A.pose.position.y - B.pose.position.y};
            std::pair<double, double> BC = {C.pose.position.x - B.pose.position.x, C.pose.position.y - B.pose.position.y};

            double L_BA = sqrt(double((A.pose.position.x - B.pose.position.x) * (A.pose.position.x - B.pose.position.x) + (A.pose.position.y - B.pose.position.y) * (A.pose.position.y - B.pose.position.y)));
            double L_BC = sqrt(double((C.pose.position.x - B.pose.position.x) * (C.pose.position.x - B.pose.position.x) + (C.pose.position.y - B.pose.position.y) * (C.pose.position.y - B.pose.position.y)));
            double L = min(L_BA, L_BC) / 3;

            double theta = acos(double(BA.first * BC.first + BA.second * BC.second) / (L_BA * L_BC));

            double L_BO = L / cos(0.5 * theta);
            double R = L * tan(0.5 * theta);

            geometry_msgs::PoseStamped D, E, F;
            E.pose.position.x = L / L_BA * BA.first + B.pose.position.x;
            E.pose.position.y = L / L_BA * BA.second + B.pose.position.y;
            F.pose.position.x = L / L_BC * BC.first + B.pose.position.x;
            F.pose.position.y = L / L_BC * BC.second + B.pose.position.y;
            D.pose.position.x = (E.pose.position.x + F.pose.position.x) / 2;
            D.pose.position.y = (E.pose.position.y + F.pose.position.y) / 2;

            double L_BD = sqrt(double((D.pose.position.x - B.pose.position.x) * (D.pose.position.x - B.pose.position.x) + (D.pose.position.y - B.pose.position.y) * (D.pose.position.y - B.pose.position.y)));

            std::pair<double, double> BD = {D.pose.position.x - B.pose.position.x, D.pose.position.y - B.pose.position.y};

            geometry_msgs::PoseStamped O;
            O.pose.position.x = L_BO / L_BD * BD.first + B.pose.position.x;
            O.pose.position.y = L_BO / L_BD * BD.second + B.pose.position.y;

            double xo = O.pose.position.x;
            double yo = O.pose.position.y;

            // if (BA.second == 0) {
            //     xo = c1 / BA.first;
            //     yo = (c2 - BC.first * xo) / BC.second;
            // } else {
            //     xo = (BC.second / BA.second * c1 - c2) / (BC.second * BA.first / BA.second - BC.first);
            //     yo = (c1 - BA.first * xo) / BA.second;
            // }
            
            // if (BA.first == 0) {
            //     if (BA.second > 0) {theta_BA = 0.5 * M_PI;}
            //     else {theta_BA = - 0.5 * M_PI;}
            // } else if (BA.first > 0) {
            //     theta_BA = std::fmod(std::fmod(atan(double(double(BA.second) / double(BA.first))) + 2 * M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            // } else {
            //     theta_BA = atan(double(double(BA.second) / double(BA.first))) + M_PI;
            // }

            // if (BC.first == 0) {
            //     if (BC.second > 0) {theta_BC = 0.5 * M_PI;}
            //     else {theta_BC = - 0.5 * M_PI;}
            // } else if (BC.first > 0) {
            //     theta_BC = std::fmod(std::fmod(atan(double(double(BC.second) / double(BC.first))) + 2 * M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            // } else {
            //     theta_BC = atan(double(double(BC.second) / double(BC.first))) + M_PI;
            // }

            // theta1 = std::fmod(std::fmod((theta_BA + 0.5 * M_PI) + 2 * M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            // theta2 = std::fmod(std::fmod((theta_BC - 0.5 * M_PI) + 2 * M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

            
            double theta_BA, theta_BC;
            if (BA.first == 0) {
                if (BA.second > 0) {theta_BA = 0.5 * M_PI;}
                else {theta_BA = - 0.5 * M_PI;}
            } else if (BA.first > 0) {
                theta_BA = std::fmod(atan(double(double(BA.second) / double(BA.first))) + 2 * M_PI, 2 * M_PI);
            } else {
                theta_BA = atan(double(double(BA.second) / double(BA.first))) + M_PI;
            }

            if (BC.first == 0) {
                if (BC.second > 0) {theta_BC = 0.5 * M_PI;}
                else {theta_BC = - 0.5 * M_PI;}
            } else if (BC.first > 0) {
                theta_BC = std::fmod(atan(double(double(BC.second) / double(BC.first))) + 2 * M_PI, 2 * M_PI);
            } else {
                theta_BC = atan(double(double(BC.second) / double(BC.first))) + M_PI;
            }

            // ROS_INFO("%lf, %lf", theta_BA, theta_BC);

            double theta1 = std::fmod((theta_BA + 0.5 * M_PI) + 2 * M_PI, 2 * M_PI);
            double theta2 = std::fmod((theta_BC - 0.5 * M_PI) + 2 * M_PI, 2 * M_PI);

            if (theta2 < theta1) {
                theta2 = theta2 + 2 * M_PI;
            }

            double delta_theta = theta2 - theta1;
            std::vector<double> theta_list;
            for (int i = 0; i <= n_points; ++i) {
                theta_list.push_back(theta1 + i * delta_theta / n_points);
            }

            std::vector<double> rho(n_points + 1, R);

            std::vector<double> X(n_points + 1);
            std::vector<double> Y(n_points + 1);

            for (int i = 0; i <= n_points; ++i) {
                X[i] = rho[i] * cos(theta_list[i]) + xo;
                Y[i] = rho[i] * sin(theta_list[i]) + yo;
                // if (X[i] < play_area_[0] || X[i] > play_area_[1] || Y[i] < play_area_[2] || Y[i] > play_area_[3]) {
                //     ROS_INFO("theta:%lf, x:%lf, y:%lf", theta_list[i], X[i], Y[i]);
                // }
            }

            if (manhattanDistance(X[0], Y[0], p_tmp_x, p_tmp_y) < manhattanDistance(X[n_points], Y[n_points], p_tmp_x, p_tmp_y)) {
                for (int i = 0; i <= n_points; ++i) {
                    geometry_msgs::PoseStamped tmp_pose;
        
                    tmp_pose.header.frame_id = frame_id_;
                    tmp_pose.pose.position.x = X[i];
                    tmp_pose.pose.position.y = Y[i];
                    
                    tmp_pose.pose.position.z = 0.0;
                    tmp_pose.pose.orientation.w = 1.0;
                    tmp_pose.pose.orientation.x = 0.0;
                    tmp_pose.pose.orientation.y = 0.0;
                    tmp_pose.pose.orientation.z = 0.0;

                    smooth_path_res.push_back(tmp_pose);
                }
                p_tmp_x = X[n_points];
                p_tmp_y = Y[n_points];
            } else {
                for (int i = n_points; i >= 0; --i) {
                    geometry_msgs::PoseStamped tmp_pose;
        
                    tmp_pose.header.frame_id = frame_id_;
                    tmp_pose.pose.position.x = X[i];
                    tmp_pose.pose.position.y = Y[i];
                    
                    tmp_pose.pose.position.z = 0.0;
                    tmp_pose.pose.orientation.w = 1.0;
                    tmp_pose.pose.orientation.x = 0.0;
                    tmp_pose.pose.orientation.y = 0.0;
                    tmp_pose.pose.orientation.z = 0.0;

                    smooth_path_res.push_back(tmp_pose);
                }
                p_tmp_x = X[0];
                p_tmp_y = Y[0];
            }
        }
        smooth_path_res.push_back(p_goal);

        nav_msgs::Path gui_path;
        gui_path.poses.resize(smooth_path_res.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < smooth_path_res.size(); i++) {
            gui_path.poses[i].header.frame_id = frame_id_;
            gui_path.poses[i].pose.position.x = smooth_path_res[i].pose.position.x;
            gui_path.poses[i].pose.position.y = smooth_path_res[i].pose.position.y;
            
            gui_path.poses[i].pose.position.z = 0.0;
            gui_path.poses[i].pose.orientation.w = 1.0;
            gui_path.poses[i].pose.orientation.x = 0.0;
            gui_path.poses[i].pose.orientation.y = 0.0;
            gui_path.poses[i].pose.orientation.z = 0.0;
        }

        smooth_path_pub_.publish(gui_path);
    }

};
