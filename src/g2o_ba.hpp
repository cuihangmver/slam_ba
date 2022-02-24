#ifndef G2O_BA_HPP_
#define G2O_BA_HPP_

#include "g2o_functions.hpp"
namespace G2O_BA
{
class OptimizeBA
{
    
    OptimizeBA()
    {

    }
    ~OptimizeBA()
    {

    }
    void OptimizePoseMulPointsOneFrame(std::vector<Eigen::Vector3d> &vecLandmark, std::vector<Eigen::Vector3d> &vecUv, std::vector<Sophus::SE3d> &vecPose, cv::Mat K)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 加位姿顶点
        for(int i = 0; i < vecPose.size(); i++)
        {
                PoseVertex *vertex_pose = new PoseVertex();
                vertex_pose->setId(i);
                vertex_pose->setEstimate(vecPose[i]);
                optimizer.addVertex(vertex_pose);
        }
        // 加路标顶点
        for(int i = 0; i < vecLandmark.size(); i++)
        {
                PointVertex *vertex_point = new PointVertex();
                vertex_pose->setId(i + vecPose.size());     // 序号不重复
                vertex_pose->setEstimate(vecLandmark[i]);  
                vertex_pose->setMarginalized(true);         // 设置边缘化
                optimizer.addVertex(vertex_point);
        }
        // 加优化边


    }

};
}
#endif