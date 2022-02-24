#ifndef CERES_FUNCTION_HPP_
#define CERES_FUNCTION_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

// 优化地图点
class CMapPointsBA : public SizedCostFunction<2/*number of residuals*/, 3/*size of first parameter*/>
{
private:
CMapPointsBA(Eigen::Vector2d uv, Sophus::SE3d pose， cv::Mat K):_pose(pose), _uv(uv), _K(K)
{

}
public:
virtual bool Evaluate(double const* const* parameters,  // parameters是待优化变量
                        double* residuals,              // 残差，输出
                        double** jacobians) const {     // 雅可比，输出
    double fx = _K.at<double>(0, 0);
    double fy = _K.at<double>(1, 1);
    double cx = _K.at<double>(0, 2);
    double cy = _K.at<double>(1, 2);

    Eigen::Vector3d pointWorld(parameters[0][0], parameters[0][1], parameters[0][2]);  // 待优化的地图点
    Eigen::Vector3d pointCamera = _pose * pointWorld;
    Eigen::Matrix<float, 2, 3> matrix23, matrix23Jacobians;    // matrix23是对相机坐标系点的导数，matrix23Jacobians是对世界坐标系点（地图点）的导数
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Matrix<float, 3, 4> R34;

    residuals[0] = _uv(0) - pointCamera(0) * fx / pointCamera(2);   // 残差
    residuals[1] = _uv(1) - pointCamera(1) * fy / pointCamera(2); 
    // 对相机坐标系的导数
    matrix23(0, 0) = fx / pointCamera(2);
    matrix23(0, 1) = 0;
    matrix23(0, 2) = (-1) * fx * pointCamera(0) / (pointCamera(2) * pointCamera(2));
    matrix23(1, 0) = 0;
    matrix23(1, 1) = fy / pointCamera(2);
    matrix23(1, 2) = (-1) * fy * pointCamera(1) / (pointCamera(2) * pointCamera(2));  
    R34 = _pose.matrix();                                     // 将旋转矩阵取出
    R(0, 0) = R34(0, 0);
    R(0, 1) = R34(0, 1);
    R(0, 2) = R34(0, 2);
    R(1, 0) = R34(1, 0);
    R(1, 1) = R34(1, 1);
    R(1, 2) = R34(1, 2);
    R(2, 0) = R34(2, 0);
    R(2, 1) = R34(2, 1);
    R(2, 2) = R34(2, 2);
    // 对世界坐标系的导数
    matrix23Jacobians = matrix23 * R * (-1);
    jacobians[0] = matrix23Jacobians(0, 0);
    jacobians[1] = matrix23Jacobians(0, 1);
    jacobians[2] = matrix23Jacobians(0, 2);
    jacobians[3] = matrix23Jacobians(1, 0);
    jacobians[4] = matrix23Jacobians(1, 1);
    jacobians[5] = matrix23Jacobians(1, 2);
    }
    static ceres::SizedCostFunction<2, 3>* Create(Eigen::Vector2d uv, Sophus::SE3d pose， cv::Mat K)
    {
        return new CMapPointsBA(uv, pose, K);
    }
private:
    Eigen::Vector2d _uv;
    Sophus::SE3d _pose;
    cv::Mat _K;

};
// 优化位姿（世界->相机）
class CPosesBA : public SizedCostFunction<2/*number of residuals*/, 6/*size of first parameter*/>
{
private:
CPosesBA(Eigen::Vector2d uv, Eigen::Vector3d xyz, cv::Mat K):_uv(uv), _xyz(xyz), _K(K)
{

}
public:
virtual bool Evaluate(double const* const* parameters,  
                        double* residuals,
                        double** jacobians) const {
    double fx = _K.at<double>(0, 0);
    double fy = _K.at<double>(1, 1);
    double cx = _K.at<double>(0, 2);
    double cy = _K.at<double>(1, 2);

    Eigen::Vector3d angle(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d t(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Matrix3d R = angle.matrix();
    Sophus::SE3d SE3_Rt(R, t);                          // 待优化变量位姿转换为李群
    Eigen::Vector3d pointCamera = SE3_Rt * _xyz;        // 世界转相机

    residuals[0] = _uv(0) - pointCamera(0) * fx / pointCamera(2);    // 残差
    residuals[1] = _uv(1) - pointCamera(1) * fy / pointCamera(2); 
    // 对姿态的雅可比
    jacobians[0] = (-1) * fx / pointCamera(2);
    jacobians[1] = 0;
    jacobians[2] = fx * pointCamera(0) / (pointCamera(2) * pointCamera(2));
    jacobians[3] = fx * pointCamera(0) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[4] = (-1) * (fx + fx * pointCamera(0) * pointCamera(0) / (pointCamera(2) * pointCamera(2)));
    jacobians[5] = fx * pointCamera(1) / (pointCamera(2) * pointCamera(2));  
    jacobians[6] = 0;
    jacobians[7] = (-1) * fy / pointCamera(2);
    jacobians[8] = fy * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[9] = fy + fy * pointCamera(1) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[10] = (-1) * fy * pointCamera(0) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[11] = (-1) * fy * pointCamera(0) / pointCamera(2);  
    }
    static ceres::SizedCostFunction<2, 6>* Create(Eigen::Vector2d uv, Eigen::Vector3d xyz, cv::Mat K)
    {
        return new CPosesBA(uv, xyz, K);
    }
private:
    Eigen::Vector2d _uv;
    Eigen::Vector3d _xyz;
    cv::Mat _K;
};
// 优化位姿和地图点
class CPosesMapPointsBA : public SizedCostFunction<2/*number of residuals*/, 3， 6/*size of first parameter*/>
{
private:
CPosesMapPointsBA(Eigen::Vector2d uv, cv::Mat K):_uv(uv), _K(K)
{

}
public:
virtual bool Evaluate(double const* const* parameters,  // parameters[0][0] = fx,parameters[0][1] = fy, parameters[0][2] = R, parameters[0][3] = t
                        double* residuals,
                        double** jacobians) const {
    double fx = _K.at<double>(0, 0);
    double fy = _K.at<double>(1, 1);
    double cx = _K.at<double>(0, 2);
    double cy = _K.at<double>(1, 2);

    Eigen::Vector3d pointWorld(parameters[0][0], parameters[0][1], parameters[0][2]);  // 待优化的地图点
    // 
    Eigen::Vector3d angle(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d t(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Matrix3d R = angle.matrix();
    Sophus::SE3d SE3_Rt(R, t);                                      // 待优化变量位姿

    Eigen::Vector3d pointCamera = SE3_Rt * pointWorld;
    Eigen::Matrix<float, 2, 3> matrix23, matrix23Jacobians;         // matrix23是对相机坐标系点的导数，matrix23Jacobians是对世界坐标系点（地图点）的导数
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Matrix<float, 3, 4> R34;
    
    residuals[0] = _uv(0) - pointCamera(0) * fx / pointCamera(2);   // 残差
    residuals[1] = _uv(1) - pointCamera(1) * fy / pointCamera(2); 
    // 相机坐标系点的雅可比
    matrix23(0, 0) = fx / pointCamera(2);
    matrix23(0, 1) = 0;
    matrix23(0, 2) = (-1) * fx * pointCamera(0) / (pointCamera(2) * pointCamera(2));
    matrix23(1, 0) = 0;
    matrix23(1, 1) = fy / pointCamera(2);
    matrix23(1, 2) = (-1) * fy * pointCamera(1) / (pointCamera(2) * pointCamera(2));  
    R34 = SE3_Rt.matrix();                                     // 将旋转矩阵取出
    R(0, 0) = R34(0, 0);
    R(0, 1) = R34(0, 1);
    R(0, 2) = R34(0, 2);
    R(1, 0) = R34(1, 0);
    R(1, 1) = R34(1, 1);
    R(1, 2) = R34(1, 2);
    R(2, 0) = R34(2, 0);
    R(2, 1) = R34(2, 1);
    R(2, 2) = R34(2, 2);
    // 世界坐标系点的雅可比
    matrix23Jacobians = matrix23 * R * (-1);
    jacobians[1][0] = matrix23Jacobians(0, 0);
    jacobians[1][1] = matrix23Jacobians(0, 1);
    jacobians[1][2] = matrix23Jacobians(0, 2);
    jacobians[1][3] = matrix23Jacobians(1, 0);
    jacobians[1][4] = matrix23Jacobians(1, 1);
    jacobians[1][5] = matrix23Jacobians(1, 2);
    // 相机姿态雅可比
    jacobians[0] = (-1) * fx / pointCamera(2);
    jacobians[1] = 0;
    jacobians[2] = fx * pointCamera(0) / (pointCamera(2) * pointCamera(2));
    jacobians[3] = fx * pointCamera(0) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[4] = (-1) * (fx + fx * pointCamera(0) * pointCamera(0) / (pointCamera(2) * pointCamera(2)));
    jacobians[5] = fx * pointCamera(1) / (pointCamera(2) * pointCamera(2));  
    jacobians[6] = 0;
    jacobians[7] = (-1) * fy / pointCamera(2);
    jacobians[8] = fy * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[9] = fy + fy * pointCamera(1) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[10] = (-1) * fy * pointCamera(0) * pointCamera(1) / (pointCamera(2) * pointCamera(2));
    jacobians[11] = (-1) * fy * pointCamera(0) / pointCamera(2);  
    }
    static ceres::SizedCostFunction<2, 3, 6>* Create(Eigen::Vector2d uv, cv::Mat K)
    {
        return new CPosesBA(uv, K);
    }
private:
    Eigen::Vector2d _uv;
    Eigen::Vector3d _xyz;
    cv::Mat _K;
};
// 重写位姿更新
class CUpdatePlus : public LocalParameterization
{
    virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta)const override;  // const表示不能修改其类成员
                    {
                        Eigen::Vector3d angle(x[0], x[1], x[2]);
                        Eigen::Vector3d t(x[3], x[4], x[5]);
                        Eigen::Matrix3d R = angle.matrix();
                        Sophus::SE3d SE3_Rt(R, t);                                   // 当前姿态

                        Eigen::Vector3d angledelta(delta[0], delta[1], delta[2]);
                        Eigen::Vector3d tdelta(delta[3], delta[4], delta[5]);
                        Eigen::Matrix3d Rdelta = angledelta.matrix();
                        Sophus::SE3d SE3_Rtdelta(Rdelta, tdelta);                   // 当前扰动
                        
                        Sophus::SE3d SE3_Rtplusdelta = SE3_Rtdelta * SE3_Rt;        // 扰动更新
                        Eigen::Vector6d se3 = SE3_Rtplusdelta.log();                // 李群转李代数
                        x_plus_delta[0] = se3(0);
                        x_plus_delta[1] = se3(1);
                        x_plus_delta[2] = se3(2);
                        x_plus_delta[3] = se3(3);
                        x_plus_delta[4] = se3(4);
                        x_plus_delta[5] = se3(5);
                    }
    bool ComputeJacobian(const double* x, double* jacobian) const override
    {
        return true;
    }
    virtual int GlobalSize()const override
    {
        return 6;
    }
    virtual int LocalSize()const override
    {
        return 6;
    }
};

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
#endif