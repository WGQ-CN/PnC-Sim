#ifndef _B_SPLINE_SMOOTHER_H
#define _B_SPLINE_SMOOTHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <cmath>

namespace rrt_global_planner {

class BSplineSmoother {
    public:

        BSplineSmoother();
        BSplineSmoother(std::string frame_id);

        void initialize(std::string frame_id);
        void uQuasiUniform(std::vector<double>& node_vector, unsigned int n, unsigned int k);
        double baseFunction(unsigned int i, unsigned int k, double u, const std::vector<double>& node_vector);
        void makeSmooth(std::vector<geometry_msgs::PoseStamped>& plan);

    protected:

    private:
        std::string frame_id_;

    };
};

#endif
