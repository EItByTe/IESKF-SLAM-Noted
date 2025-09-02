/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-13 17:49:23
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 16:33:53
 */
#include "ieskf_slam/modules/map/rect_map_manager.h"
namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"RectMapManager")
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
        readParam<float>("map_side_length_2",map_side_length_2,500);
        readParam<float>("map_resolution",map_resolution,0.5);
    }
    
    RectMapManager::~RectMapManager()
    {
    }
    
    // 生成局部地图的逻辑，保留再map_side_length_2中的地图点，可以做到比较笨蛋的滑动窗口.
    // cc-note 可以优化的点，对local_map_ptr加入更好的滑动窗口算法
    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        if(local_map_ptr->empty()){
            *local_map_ptr=scan;
        }else{
            // . 添加
            // . 离添加点的最近点大于分辨率就添加
            // 这样写更加通用，叫做万能引用，适配左值和右值，避免拷贝，提高效率
            for (auto &&point : scan)
            {
                std::vector<int> ind;
                std::vector<float> distance;
                kdtree_ptr->nearestKSearch(point,5,ind,distance);   // 查找该点在local_map中最近的5个点，索引和距离都存放在ind和distance中
                // 其中最近邻点的索引在ind[0]，距离在distance[0]
                if(distance[0]>map_resolution)local_map_ptr->push_back(point); // 如果该点与当前地图中的点距离够远，那就加入到局部地图中去。
            }
            // 删除
            // . 把地图中超过 地图范围的挪到后面，用resize删除
            int left = 0,right = local_map_ptr->size()-1;
            // 目前这边的做法是把地图约束在一个正方体内，现在实际上都不会删除任何东西。
            // cc-note 这里可以优化成long-life地图的管理方式
            while(left<right){
                // std::cout << "0000000left: " << left << ", right: " << right << std::endl;
                // 如果右侧超出范围，就右指针左移，找到右侧，第一个在范围内的点。
                while(left<right&&(abs(local_map_ptr->points[right].x-pos_t.x())>map_side_length_2
                                ||abs(local_map_ptr->points[right].y-pos_t.y())>map_side_length_2
                                ||abs(local_map_ptr->points[right].z-pos_t.z())>map_side_length_2))
                    {
                        right--;
                    }
                // std::cout << "11111111left: " << left << ", right: " << right << std::endl;
                // 如果左侧点在范围内，left右移，如果不在范围内，会被swap到最后。
                while(left<right&&abs(local_map_ptr->points[left].x-pos_t.x())<map_side_length_2
                                &&abs(local_map_ptr->points[left].y-pos_t.y())<map_side_length_2
                                &&abs(local_map_ptr->points[left].z-pos_t.z())<map_side_length_2)
                    {
                        left++;
                    }
                // std::cout << "2222222left: " << left << ", right: " << right << std::endl;
                // 发现左边不在范围内了，就把当前左边的点移到右边去。
                std::swap(local_map_ptr->points[left],local_map_ptr->points[right]);
            }
            local_map_ptr->resize(right+1); // 把后面的去掉.
        }
        kdtree_ptr->setInputCloud(local_map_ptr);
        
    }
    void RectMapManager::reset(){
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap(){
        return local_map_ptr;
    }
    KDTreeConstPtr RectMapManager::readKDtree(){
        return kdtree_ptr;
    }
} // namespace IESKFSlam
