#ifndef _PURE_PURSUIT_H
#define _PURE_PURSUIT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <cmath>

class PurePursuit {
    public:

        PurePursuit();
        ~PurePursuit() = default;

        void initialize();
        void calculateTwistCommand();
        void globalPlanCallback(const nav_msgs::PathConstPtr& global_plan);
        void runPurePursuit();

    protected:
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber global_plan_sub_;

        tf::TransformListener tf_listener_;

    private:
        nav_msgs::Path global_plan_;
        double look_ahead_distance_;
        double stop_distance_;
        double slow_distance_;

        double wheelbase_;
        double max_vel_x_, min_vel_x_;
        
        double pose_x_, pose_y_;
        double pose_yaw_;

        bool is_planned_;
};

#endif
