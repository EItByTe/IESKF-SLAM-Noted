/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-18 20:00:47
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:18:14
 */
#pragma once 
#ifndef MATH_SO3
#define MATH_SO3
#include <Eigen/Dense>
namespace IESKFSlam
{
    // 反对称矩阵
    static inline Eigen::Matrix3d  skewSymmetric(const Eigen::Vector3d &so3){
        Eigen::Matrix3d so3_skew_sym;
        so3_skew_sym.setZero();
        so3_skew_sym(0,1) = -1*so3(2);
        so3_skew_sym(1,0) = so3(2);
        so3_skew_sym(0,2) = so3(1);
        so3_skew_sym(2,0) = -1*so3(1);
        so3_skew_sym(1,2) = -1*so3(0);
        so3_skew_sym(2,1) = so3(0);
        return so3_skew_sym;
    }

    static Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
        Eigen::Matrix3d  SO3; // 3x3
        double so3_norm = so3.norm(); // 李代数so3的模长就是旋转角度(弧度)。
        if (so3_norm<=0.0000001)
        {
            SO3.setIdentity();
            return SO3;
        }

        Eigen::Matrix3d so3_skew_sym = skewSymmetric(so3);
        // Rodrigues' rotation formula，基于so(3) : w = theta * v的形式，公式如下：
        // SO3 = I + sin(theta)/theta * so3_skew_sym + (1-cos(theta))/theta^2 * so3_skew_sym^2
        // 这个公式可以看作是罗德里格斯公式的变体，可以去看 一篇1995年的 论文 《Proportional Derivative(PD) Control on the Euclidean Group》 p1092，lemma1
        SO3 = Eigen::Matrix3d::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym/(so3_norm*so3_norm))*(1-cos(so3_norm));
        return SO3;
    }

    // 李群转李代数 SO(3) -> so(3)
    // 公式如下：
    // w = theta * u = theta * 0.5 / sin(theta) * (SO3 - SO3^T)
    // 其中 theta = arccos((SO3.trace()-1)/2)
    static Eigen::Vector3d SO3Log(const Eigen::Matrix3d&SO3 ){
        double theta = (SO3.trace()>3-1e6)?0:acos((SO3.trace()-1)/2);   // SO3.trace() = 3 表示是单位矩阵
        // 计算旋转向量u
        Eigen::Vector3d so3(SO3(2,1)-SO3(1,2),SO3(0,2)-SO3(2,0),SO3(1,0)-SO3(0,1));
        // 当theta 很小时，sin(theta)约等于theta，所以这里直接返回0.5*so3即可
        return fabs(theta)<0.001?(0.5*so3):(0.5*theta/sin(theta)*so3);  // theta * u，所以结果的分子上多了theta，
    }

    // 对应论文Fast-lio 中的公式6
    static Eigen::Matrix3d  A_T(const Eigen::Vector3d& v){
        Eigen::Matrix3d res;
        double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        double norm = std::sqrt(squaredNorm);
        if(norm <1e-11){
            res = Eigen::Matrix3d::Identity();
        }
        else{
            // 《Proportional Derivative(PD) Control on the Euclidean Group》 p1092，lemma1 SE(3)中用到的。
            // 右雅可比矩阵 论文中的公式是A(v)^{-1}，这里没有取逆操作，但是这里在实际用的时候加了负号，也就是说A(-v) = A(v)^{-1}，所以这里的v是负的。
            res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) + (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
        }
        return res;
    }
} // namespace SO3
#endif