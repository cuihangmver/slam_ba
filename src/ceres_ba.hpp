#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/block_solver.h>
#include "ceres_function.hpp"
namespace CERES_BA
{
class OptimizeBA
{
    CMapPointsBA *m_objCMapPointsBA;
    OptimizeBA()
    {

    }
    ~OptimizeBA()
    {

    }
    
   // 优化地图点，仅优化一帧的一个地图点
    void OptimizeMapPointsOnePointOneFrame(Eigen::Vector2d uv, double *xyz, const Sophus::SE3d pose， cv::Mat K)
    {
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;

        ceres::CostFunction *costFunction = CMapPointsBA::Create(vecUv[i], pose, K);
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(1.0);
        
        problem.AddResidualBlock(costFunction, lossFunction, xyz);
        Solver::Summary summary;
        Solve(options, &problem, &summary); 
    }
    // 优化位姿
    void OptimizePoseMulPointsOneFrame(std::vector<Eigen::Vector2d> vecUv, std::vector<Eigen::Vector3d> vecXyz, double *pose， cv::Mat K)
    {
        ceres::Problem problem;
        ceres::Solver::Options options;
        
        options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        for(int i = 0; i < vecUv.size(); i++)
        {
            ceres::CostFunction *costFunction = CPosesBA::Create(vecUv[i], vecXyz[i], K);
            ceres::LossFunction *lossFunction = new ceres::HuberLoss(1.0);
            CUpdatePlus objCUpdatePlus = new objCUpdatePlus();
            problem.AddResidualBlock(costFunction, lossFunction, pose);
            problem.AddParameterBlock(pose, 6, objCUpdatePlus);
        }
        Solver::Summary summary;
        Solve(options, &problem, &summary); 

    }
    // 优化地图点及位姿
    void OptimizePoseMulPointsOneFrame(std::vector<Eigen::Vector2d> vecUv, double **xyz, double *pose， cv::Mat K)
    {
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;

        for(int i = 0; i < vecUv.size(); i++)
        {
            ceres::CostFunction *costFunction = CPosesMapPointsBA::Create(vecUv[i], K);
            ceres::LossFunction *lossFunction = new ceres::HuberLoss(1.0);
            CUpdatePlus objCUpdatePlus = new objCUpdatePlus();
            problem.AddResidualBlock(costFunction, lossFunction, xyz[i], pose);
            problem.AddParameterBlock(pose, 6, objCUpdatePlus);
        }
        Solver::Summary summary;
        Solve(options, &problem, &summary);
    }
};
}
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