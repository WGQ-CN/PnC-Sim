#ifndef _RRT_GLOBAL_PLANNER_H
#define _RRT_GLOBAL_PLANNER_H
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

#include <cmath>
#include <random>
#include <time.h>
#include <cstdlib>

#include <visualization_msgs/MarkerArray.h>

using std::string;

namespace rrt_global_planner {

#define inf 1>>20
#define N 999
struct Node;
typedef std::shared_ptr<Node> NodePtr;

struct Node
{
    int x_, y_;
    int index_;

    NodePtr came_from_;

    std::vector<int> path_x_;
    std::vector<int> path_y_;

    Node(int index, int x, int y) {
        index_ = index;
        x_ = x;
        y_ = y;

        came_from_ = nullptr;

        path_x_.clear();
        path_y_.clear();
    }

    Node(){};
    ~Node(){};
};

class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:

        RRTGlobalPlanner();
        RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void mapToWorld(double mx, double my, double& wx, double& wy);
        double manhattanDistance(double x1, double y1, double x2, double y2);
        double euclideanDistance(double x1, double y1, double x2, double y2);
        bool isCellFree(int index);
        bool isCellFree(int x, int y);

        NodePtr steer(const NodePtr& from_node, const NodePtr& to_node);
        void generateFinalCourse(std::vector<geometry_msgs::PoseStamped>& plan, const std::vector<NodePtr>& node_list);
        double calcDist2Goal(int x, int y, int goal_i);
        NodePtr getRandomNode(int goal_i);
        int getNearestNodeIndex(const std::vector<NodePtr>& node_list, const NodePtr& rnd_node);
        bool checkIfOutsidePlayArea(const NodePtr& node);
        bool checkCollision(const NodePtr& node);
        std::pair<double, double> clacDistanceAndAngle(const NodePtr& from_node, const NodePtr& to_node);

        void waypointOptimize(std::vector<geometry_msgs::PoseStamped>& plan);
        bool isPathFree(const geometry_msgs::PoseStamped& from_pose, const geometry_msgs::PoseStamped& to_pose);

        void publishPlanPoints(const std::vector<geometry_msgs::PoseStamped>& path);

    protected:
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_;

        ros::Publisher path_nodes_pub_;

    private:
        int nx_, ny_, ns_;
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        inline float randone() {
            return rand() % (N + 1) / (float)(N + 1);
        }

        unsigned char* costs_;

        std::pair<int, int> rand_area_;
        double expand_dis_;
        double path_resolution_;
        double goal_sample_rate_;
        double max_iter_;
        std::vector<int> play_area_;

    };
};

#endif
