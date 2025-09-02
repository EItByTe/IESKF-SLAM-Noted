/*
 * @Author: Danny 986337252@qq.com
 * @Date: 2023-06-23 17:11:32
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 16:26:11
 * @FilePath: /ieskf_slam/include/ieskf_slam/modules/frontend/lio_zh_model.hpp
 */
#pragma once
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/math/geometry.h"
namespace IESKFSlam
{
    class LIOZHModel :public IESKF::CalcZHInterface
    {
    private:
        const int NEAR_POINTS_NUM = 5;
        //1) point_imu 2)normal_vector 3)distance      
        using loss_type = triple<Eigen::Vector3d,Eigen::Vector3d,double>; // 三元组
        KDTreeConstPtr global_map_kdtree_ptr;
        PCLPointCloudPtr current_cloud_ptr;
        PCLPointCloudConstPtr local_map_ptr;
    public:
    using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(KDTreeConstPtr kd_tree,PCLPointCloudPtr current_cloud,PCLPointCloudConstPtr local_map){
            global_map_kdtree_ptr = kd_tree;
            current_cloud_ptr = current_cloud;
            local_map_ptr = local_map;
        }
        bool calculate(const IESKF::State18&state,Eigen::MatrixXd & Z,Eigen::MatrixXd & H)override{
            std::vector<loss_type> loss_v;
            loss_v.resize(current_cloud_ptr->size());
            std::vector<bool> is_effect_point(current_cloud_ptr->size(),false);
            std::vector<loss_type> loss_real;
            int  vaild_points_num = 0;
            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
                #pragma omp parallel for
            #endif
            /**
             * 有效点的判断
             * 1. 将当前点变换到世界系下
             * 2. 在局部地图上使用kd_tree 临近搜索NEAR_POINTS_NUM个点
             * 3. 判断这些点是否构成平面
             * 4. 判断点离这个平面够不够近(达到阈值)
             * 5. 满足上述条件，设置为有效点。
            */
            for (size_t  i = 0; i < current_cloud_ptr->size(); i++)
            {
                // . 变换到世界系
                Point point_imu = current_cloud_ptr->points[i];
                Point point_world;
                point_world = transformPoint(point_imu,state.rotation,state.position); 
                // . 临近搜索 K近邻搜索
                std::vector<int> point_ind;
                std::vector<float> distance;
                global_map_kdtree_ptr->nearestKSearch(point_world,NEAR_POINTS_NUM,point_ind,distance);
                // . 是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
                if (distance.size()<NEAR_POINTS_NUM||distance[NEAR_POINTS_NUM-1]>5)
                {
                    continue;
                }
                // . 判断这些点够不够成平面
                std::vector<Point> planar_points;
                for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
                {
                    planar_points.push_back(local_map_ptr->at(point_ind[ni]));
                }
                Eigen::Vector4d pabcd;
                // . 如果构成平面
                if (planarCheck(planar_points,pabcd,0.1))    
                {
                    // 法向量： pabcd
                    // . 计算点到平面距离
                    double pd = point_world.x*pabcd(0)+point_world.y*pabcd(1)+point_world.z*pabcd(2)+pabcd(3); // 由于归一化了，分母为1
                    // . 记录残差
                    loss_type loss;
                    loss.third = pd; // 残差
                    loss.first = {point_imu.x,point_imu.y,point_imu.z}; // imu系下点的坐标，用于求H
                    loss.second = pabcd.block<3,1>(0,0);// 平面法向量 用于求H
                    if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
                    // .计算点和平面的夹角，夹角越小S越大。
                    // 平面拟合的另一个判断,这里不是夹角判断，但是可以理解为和平面的距离远近，物理意义上不太好解释
                    // fabs(pd) / sqrt(loss.first.norm() 可以理解为sin(theta) theta为点到平面的夹角，而这个平面也不一定是过imu的原点。
                    double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm()); // 并非合理的物理意义上的夹角
                    // 实际加不加这个判断都行
                    // cc-note 这里可以优化。调大调小效果都差不多
                    if(s > 0.9 ){
                        vaild_points_num++;
                        loss_v[i]=(loss);
                        is_effect_point[i] = true;
                    }
                }
            }
            // cc-note 这里的写法可以优化
            for (size_t i = 0; i <current_cloud_ptr->size() ; i++)
            {
                if(is_effect_point[i])loss_real.push_back(loss_v[i]);
            }
            // 根据有效点的数量分配H Z的大小
            vaild_points_num = loss_real.size();
            H = Eigen::MatrixXd::Zero(vaild_points_num, 18); 
            Z.resize(vaild_points_num,1);
            for (int vi = 0; vi < vaild_points_num; vi++)
            {
                // H 记录导数
                // dr = -n^T * R * [p]x
                Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(loss_real[vi].first);
                // 为什么是前三列呢？因为状态量的定义是这样，要对应起来。
                H.block<1,3>(vi,0) = dr.transpose();    // 对旋转求导
                H.block<1,3>(vi,3) = loss_real[vi].second.transpose(); // second是法向量  对平移求导
                // 打印一下H的大小
                // std::cout<<"H: "<<H.rows()<<" "<<H.cols()<<std::endl;
                // Z记录距离
                Z(vi,0) = loss_real[vi].third;
            }
            return true;
        }
    }; 
} // namespace IESKFSlam
