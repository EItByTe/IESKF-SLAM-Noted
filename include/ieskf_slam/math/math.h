/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:55:35
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:18:11
 */
#pragma once
#include <Eigen/Dense>
namespace IESKFSlam
{
    static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond & q,const Eigen::Vector3d & t){
        Eigen::Matrix4d ans; // 4x4 double matrix
        ans.setIdentity();   //  单位阵
        ans.block<3,3>(0,0) = q.toRotationMatrix(); // 四元数转旋转矩阵
        ans.block<3,1>(0,3) = t;
        return ans;
    }
} // namespace IESKFSlam
