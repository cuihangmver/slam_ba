#ifndef G2O_FUNCTION_HPP_
#define G2O_FUNCTION_HPP_
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
// 设置待优化姿态变量（姿态点）
class PoseVertex : public g2o::BaseVertex<6, SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 待优化变量设初值
        virtual void setToOriginImpl() override 
        { 
                _estimate = SE3();     // 对待优化变量初始化
        } 
        // 待优化变量更新
        virtual void oplusImpl(const double *update) override
        {
                Eigen::Vec6 update_eigen;
                update_eigen << update[0], update[1], update[2], update[3], update[4],
                update[5];
                _estimate = SE3::exp(update_eigen) * _estimate;  // 李群更新
        }
        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
};

// 设置待优化路标点变量（路标点）
class PointVertex : public g2o::BaseVertex<3, Eigen::Vec3>  // 定义_estimate的类型
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 待优化变量设初值
        virtual void setToOriginImpl() override 
        { 
                _estimate = Eigen::Vec3::Zero();     // 对待优化变量初始化
        } 
        // 待优化变量更新
        virtual void oplusImpl(const double *update) override
        {
                Eigen::Vector3d update_eigen;
                update_eigen << update[0], update[1], update[2];
                _estimate = update_eigen + _estimate;  // 李群更新
        }
        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
};

// 一元边，优化姿态
class PoseEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PoseVertex>  // 残差维度及类型
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        PoseEdge(cv::Mat K, Eigen::Vector3d point):_K(K), _point(point)
        {

        }
        // 计算残差
        virtual void computeError() override 
        {
               const VertexPose *v = static_cast<VertexPose *>(_vertices[0]); 
               SE3::SE3d T = v->estimate();
               Eigen::Vector3d pointCam3d = _K * T * point;
               Eigen::Vector2d uv = pointCam3d /= pointCam3d[2];
               _error = _measurement - uv;
        }
        virtual void linearizeOplus() override
        {
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]); 
                SE3::SE3d T = v->estimate();
                Eigen::Vector3d pointCam3d = T * point;
                Eigen::Vector3d pos_cam = T * _pos3d;
                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pointCam3d[0];
                double Y = pointCam3d[1];
                double Z = pointCam3d[2];
                double Zinv = 1.0 / (Z + 1e-18);  // 防止Z=0
                double Zinv2 = Zinv * Zinv;
                _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
        }
        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
private:
        cv::Mat _K;
        Eigen::Vector3d _point;
};

// 一元边，优化路标点
class PointEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PointVertex>  // 残差维度及类型
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
};

// 二元边，优化位姿，路标点
class PointEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PoseVertex, PointVertex>  // 残差维度及类型
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};
#endif