#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/block_solver.h>
int main(int argc, char **argv)
{
    Eigen::Matrix<float, 2, 3> matrix;
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3d SO3_R(R);
    cv::Mat image;
    ceres::Solver::Options options;
    g2o::SparseOptimizer opt;
    return 0;
}