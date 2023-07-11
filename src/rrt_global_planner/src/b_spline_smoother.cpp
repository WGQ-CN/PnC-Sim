#include <b_spline_smoother.h>

using namespace std;

namespace rrt_global_planner {

    BSplineSmoother::BSplineSmoother() {
    }

    BSplineSmoother::BSplineSmoother(std::string frame_id) {
        initialize(frame_id);
    }

    void BSplineSmoother::initialize(std::string frame_id) {
        frame_id_ = frame_id;
    }

    void BSplineSmoother::uQuasiUniform(std::vector<double>& node_vector, unsigned int n, unsigned int k) {
        node_vector.resize(n + k + 2, 0.0);
        unsigned int piecewise = n - k + 1;
        if (piecewise == 1) {
            for (int i = k + 1; i <= n + k + 1; ++i) {
                node_vector[i] = 1.0;
            }
        } else {
            unsigned int flag = 0;
            while (flag != piecewise - 1) {
                node_vector[k + flag + 1] = node_vector[k + flag] + 1.0 / piecewise;
                flag = flag + 1;
            }
            for (int i = n + 1; i <= n + k + 1; ++i) {
                node_vector[i] = 1.0;
            }
        }
    }

    double BSplineSmoother::baseFunction(unsigned int i, unsigned int k, double u, const std::vector<double>& node_vector) {
        if (k == 0) {
            if (u >= node_vector[i] && u < node_vector[i + 1]) {
                return 1.0;
            } else {
                return 0.0;
            }
        } else {
            double length1 = node_vector[i + k] - node_vector[i];
            double length2 = node_vector[i + k + 1] - node_vector[i + 1];
            if (length1 == 0) length1 = 1;  // 注意: double类型判0
            if (length2 == 0) length2 = 1;
            return (u - node_vector[i]) / length1 * baseFunction(i, k-1, u, node_vector)
             + (node_vector[i+k+1] - u) / length2 * baseFunction(i+1, k-1, u, node_vector);
        }
    }

    void BSplineSmoother::makeSmooth(std::vector<geometry_msgs::PoseStamped>& plan) {
        std::vector<geometry_msgs::PoseStamped> path;
        std::vector<double> node_vector;
        unsigned int k = 4;
        unsigned int n = plan.size() - 1;

        if (n < 2) {
            return ;
        } 
        if (n == 2) {
            k = 3;
        }

        std::vector<double> Bik(n + 1, 1);

        uQuasiUniform(node_vector, n, k - 1);
        double u = 0.0;
        for (int cnt = 0; cnt < 200; ++cnt) {
            for (int i = 0; i <= n; ++i) {
                Bik[i] = baseFunction(i, k - 1, u, node_vector);
            }
            u += 0.005;
            double x = 0.0, y = 0.0;
            for (int i = 0; i < n + 1; ++i) {
                x += plan[i].pose.position.x * Bik[i];
                y += plan[i].pose.position.y * Bik[i];
            }

            geometry_msgs::PoseStamped tmp_pose;
            tmp_pose.header.frame_id = frame_id_;
            tmp_pose.pose.position.x = x;
            tmp_pose.pose.position.y = y;
            
            tmp_pose.pose.position.z = 0.0;
            tmp_pose.pose.orientation.w = 1.0;
            tmp_pose.pose.orientation.x = 0.0;
            tmp_pose.pose.orientation.y = 0.0;
            tmp_pose.pose.orientation.z = 0.0;

            path.push_back(tmp_pose);
        }
        plan = path;
    }

}