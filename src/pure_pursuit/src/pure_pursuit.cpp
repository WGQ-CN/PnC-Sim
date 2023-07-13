#include "pure_pursuit.h"

PurePursuit::PurePursuit() {
    initialize();
}

void PurePursuit::initialize() {
    ros::NodeHandle nh;

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    global_plan_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 1, &PurePursuit::globalPlanCallback, this);

    look_ahead_distance_ = 1.2;
    wheelbase_ = 0.498;
    stop_distance_ = 0.1;
    slow_distance_ = 0.3;
    max_vel_x_ = 1.2;
    min_vel_x_ = 0.2;
}

void PurePursuit::globalPlanCallback(const nav_msgs::PathConstPtr& global_plan_ptr) {
    global_plan_ = *global_plan_ptr;
}

void PurePursuit::calculateTwistCommand() {
    double lad_acc = 0.0;
    unsigned int target_index = global_plan_.poses.size() - 1;
    geometry_msgs::Twist twist_cmd;

    if (target_index < 1) {
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(twist_cmd);
        return ;
    }

    for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
        double this_x = global_plan_.poses[i].pose.position.x;
        double this_y = global_plan_.poses[i].pose.position.y;
        double next_x = global_plan_.poses[i + 1].pose.position.x;
        double next_y = global_plan_.poses[i + 1].pose.position.y;
        lad_acc += hypot(next_x - this_x, next_y - this_y);
        if (lad_acc > look_ahead_distance_) {
            target_index = i + 1;
            break;
        }
    }

    double target_x = global_plan_.poses[target_index].pose.position.x;
    double target_y = global_plan_.poses[target_index].pose.position.y;

    double alpha = atan2(target_y - pose_y_, target_x - pose_x_) - pose_yaw_;
    double final_x = global_plan_.poses[global_plan_.poses.size() - 1].pose.position.x;
    double final_y = global_plan_.poses[global_plan_.poses.size() - 1].pose.position.y;

    double final_dis = sqrt(pow(pose_x_ - final_x, 2) + pow(pose_y_ - final_y, 2));
    double l = sqrt(pow(pose_x_ - target_x, 2) + pow(pose_y_ - target_y, 2));

    double theta, vel_a, vel_x;

    if (final_dis > stop_distance_) {
        theta = atan(2 * wheelbase_ * sin(alpha) / l) * 2;
        vel_a = tan(theta) * max_vel_x_ / wheelbase_;

        if (final_dis < slow_distance_) {
            vel_x = (l - stop_distance_) / (slow_distance_ - stop_distance_) * max_vel_x_;				
            vel_a = 0.5 * (l - stop_distance_) / (slow_distance_ - stop_distance_) * vel_a;
            if (vel_x < min_vel_x_) {
                vel_x = min_vel_x_;
            }
        } else {
            vel_x = max_vel_x_;
        }
        twist_cmd.linear.x = vel_x;
        twist_cmd.angular.z = vel_a;
        cmd_vel_pub_.publish(twist_cmd);
        return ;
    } else {
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(twist_cmd);
        return ;
    }
}

void PurePursuit::runPurePursuit() {
    while (!ros::isShuttingDown()) {
        tf::StampedTransform transform;
        if (tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3))) {
            tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
            pose_x_ = transform.getOrigin().x();
            pose_y_ = transform.getOrigin().y();
            pose_yaw_ = tf::getYaw(transform.getRotation());
            ROS_INFO("x:%lf, y:%lf, yaw:%lf", pose_x_, pose_y_, pose_yaw_);
            calculateTwistCommand();
        }
        ros::spinOnce();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pure_pursuit;
    pure_pursuit.runPurePursuit();
    return 0;
}
