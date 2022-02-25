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

    int consitute(int i, int j)
    {
        int num = 10;
        while(1)
        {
            if(j / num == 0)
            {
                break;
            }
            num = num * 10;
        }
        int cons = i * num + j;
        return cons;
    }

    void OptimizePoseMulPointsOneFrame(std::vector<Eigen::Vector3d> &vecLandmark, std::map<int, Eigen::Vector2d> &mapUv, std::vector<Sophus::SE3d> &vecPose, cv::Mat K)
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
        for(int i = 0; i < vecLandmark.size(); i++)
        {
            for(int j = 0; j < vecPose.size(); j++)
            {
                PosePointEdge *posepointEdge = new PosePointEdge(K);
                posepointEdge->setId(consitute(i, j));
                posepointEdge->setVertex(0, vecPose[j]);         // pose
                posepointEdge->setVertex(1, vecLandmark[i]);     // point
                posepointEdge->Measurement(mapUv[consitute(i, j)]);  // 像素
                posepointEdge->setInformation(Mat22::Identity());          // 信息矩阵
                auto rk = new g2o::RobustKernelHuber();          // 鲁棒核
                rk->setDelta(chi2_th);
                posepointEdge->setRobustKernel(rk);
                optimizer.addEdge(posepointEdge);
            }
        }
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    }

};
}
#endif