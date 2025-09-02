/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:27:35
 */
#include "ieskf_slam/modules/frontend/frontend.h"
namespace IESKFSlam {
    FrontEnd::FrontEnd(const std::string &config_file_path, const std::string &prefix)
        : ModuleBase(config_file_path, prefix, "Front End Module") {
        float leaf_size;
        readParam("filter_leaf_size", leaf_size, 0.5f);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        std::vector<double> extrin_v;
        readParam("extrin_r", extrin_v, std::vector<double>());
        extrin_r.setIdentity();
        extrin_t.setZero();
        if (extrin_v.size() == 9) {
            Eigen::Matrix3d extrin_r33;
            extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4],
                extrin_v[5], extrin_v[6], extrin_v[7], extrin_v[8];
            extrin_r = extrin_r33;
        } else if (extrin_v.size() == 3) {
            extrin_r.x() = extrin_v[0];
            extrin_r.y() = extrin_v[1];
            extrin_r.z() = extrin_v[2];
            extrin_r.w() = extrin_v[3];
        }
        readParam("extrin_t", extrin_v, std::vector<double>());
        if (extrin_v.size() == 3) {
            extrin_t << extrin_v[0], extrin_v[1], extrin_v[2];
        }
        ieskf_ptr = std::make_shared<IESKF>(config_file_path, "ieskf");
        map_ptr = std::make_shared<RectMapManager>(config_file_path, "map");

        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr,
                                  map_ptr->getLocalMap());

        readParam("enable_record", enable_record, false);
        readParam("record_file_name", record_file_name, std::string("default.txt"));
        if (enable_record) {
            record_file.open(RESULT_DIR + record_file_name, std::ios::out | std::ios::app);
        }

        print_table();
    }
    FrontEnd::~FrontEnd() { record_file.close(); }
    // 向前端添加imu数据，这里实现的很简单
    void FrontEnd::addImu(const IMU &imu) { imu_deque.push_back(imu); }
    // 向前端添加点云数据，点云数据会被转换到imu坐标系下
    void FrontEnd::addPointCloud(const PointCloud &pointcloud) {
        pointcloud_deque.push_back(pointcloud);
        // 每次进来一帧，就对这一帧点云进行转换
        pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr,
                                 *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
    }
    
    // 主程序入口函数
    bool FrontEnd::track() {
        MeasureGroup mg;
        if (syncMeasureGroup(mg)) {
            // 初始化
            if (!imu_inited) {
                map_ptr->reset();
                // mg.cloud.cloud_ptr 转换到imu坐标系下的点云了
                map_ptr->addScan(mg.cloud.cloud_ptr, Eigen::Quaterniond::Identity(),
                                 Eigen::Vector3d::Zero());
                initState(mg); // 初始化了状态中的imu零偏和重力
                return false;
            }
            // 前向传播
            fbpropagate_ptr->propagate(mg, ieskf_ptr);
            voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);
            ieskf_ptr->update();
            auto state = ieskf_ptr->getX();
            if (enable_record) {
                record_file << std::setprecision(15) << mg.lidar_end_time << " "
                            << state.position.x() << " " << state.position.y() << " "
                            << state.position.z() << " " << state.rotation.x() << " "
                            << state.rotation.y() << " " << state.rotation.z() << " "
                            << state.rotation.w() << std::endl;
            }
            // 转换到map坐标系下
            map_ptr->addScan(filter_point_cloud_ptr, state.rotation, state.position);
            return true;
        }
        return false;
    }
    // 返回当前滤波后的点云数据
    const PCLPointCloud &FrontEnd::readCurrentPointCloud() { return *filter_point_cloud_ptr; }
    const PCLPointCloud &FrontEnd::readCurrentLocalMap() { return *map_ptr->getLocalMap(); }
    
    /// @brief 同步测量组，测量组包含点云和imu数据
    /// @param mg 测量组
    /// @return 如果同步成功返回true，否则返回false
    /// 同步的逻辑是：点云的时间戳必须在imu的时间戳范围内
    /// 如果点云的时间戳在imu的时间戳范围内，则将点云和imu数据添加到测量组中
    /// 如果点云的时间戳不在imu的时间戳范围内，则丢弃点云数据
    bool FrontEnd::syncMeasureGroup(MeasureGroup &mg) {
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if (pointcloud_deque.empty() || imu_deque.empty()) {
            return false;
        }
        ///. wait for imu
        // 当前双端队列里，会有好几帧IMU，取这段IMU时间内的点云。IMU是好几帧，点云是取最近的一帧。
        // 统一比较秒
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time = pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time =
            pointcloud_deque.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;

        // 最好是：imu_start_time < cloud_start_time < cloud_end_time < imu_end_time
        // 但这里只判断了imu_end_time和cloud_end_time的关系，start没有进行比较
        if (imu_end_time < cloud_end_time) {
            // std::cout<<"1111: imu_end_time < cloud_end_time, discard pointcloud."<<std::endl;
            pointcloud_deque.pop_front();
            return false;
        }

        if (cloud_end_time < imu_start_time) {
            // std::cout<<"2222: imu_start_time > cloud_end_time, discard pointcloud."<<std::endl;
            // 完全不在IMU时间范围内，直接丢弃
            pointcloud_deque.pop_front();
            return false;
        }
        // 这个判断更加严格。但是会丢掉很多数据，不做这个处理
        // if (cloud_start_time < imu_start_time) {
        //     std::cout<<"3333: cloud_start_time < imu_start_time, discard pointcloud."<<std::endl;
        //     // pointcloud_deque.pop_front();
        //     return false;
        // }
        // cc-note: 可优化点：现在的情况是lidar_start_time < imu_start_time < lidar_end_time < imu_end_time，应该会导致很多点是找不到很精确的imu数据进行修正的。
        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;
        // 依然，这里会放很多帧imu，用于一帧的点云
        while (!imu_deque.empty()) {
            // 半开环的方式加入imu的测量
            // 放到 lidar_end_time为止
            // cc-note 可优化的点：t表示t时刻，则[lidar_end_time(t-1), lidar_start_time(t)] 之间的imu会参与 t时刻的前向传播。如果严格一些，可以将其中一些imu的消息给丢弃。
            if (imu_deque.front().time_stamp.sec() < mg.lidar_end_time) {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
            } else {
                break;
            }
        }
        if (mg.imus.size() <= 5) {
            // std::cout << "IMU数据太少了!: " << mg.imus.size()<<std::endl;
            return false;
        }
        return true;
    }
    
    // 初始化状态，主要是初始化状态中imu的零偏和重力，亲身经历中，有的imu以g为单位，有的imu以m/s^2为单位，这里的imu的z轴还是1g.
    void FrontEnd::initState(MeasureGroup &mg) {
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0, 0, 0};
        auto &ieskf = *ieskf_ptr;
        if (imu_inited) {
            return;
        }

        for (size_t i = 0; i < mg.imus.size(); i++) {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc += mg.imus[i].acceleration;
            // 认为是静止，所以认为这就是陀螺仪的零偏，实际很小
            x.bg += mg.imus[i].gyroscope;
            // std::cout << "imu: " << i << " bg: " << x.bg.transpose() << std::endl;
            ieskf.setX(x); // 将陀螺仪零偏写入
        }
        // 假设一开始是静止的，收集满5个imu数据就开始初始化。
        if (imu_count >= 5) {
            auto x = ieskf.getX();
            mean_acc /= double(imu_count);

            x.bg /= double(imu_count); // 因为前面已经将陀螺仪零偏写入了，所以这里直接除以imu_count即可
            imu_scale = GRAVITY / mean_acc.norm(); // 所以这里的scale,可以去除imu乘以频率的情况.

            // 重力的符号为负 就和fastlio公式一致
            x.gravity = -mean_acc / mean_acc.norm() * GRAVITY; // 记得要加负号,通常在imu中的z轴加计是正的.
            // 打印初始化这边的gravity
            // std::cout << "imu_scale: " << imu_scale << ", gravity: " << x.gravity.transpose()
            //           << std::endl;
            ieskf.setX(x);
            imu_inited = true;

            // 后向传播的内容 ?
            fbpropagate_ptr->imu_scale = imu_scale;
            fbpropagate_ptr->last_imu = mg.imus.back();
            fbpropagate_ptr->last_lidar_end_time_ = mg.lidar_end_time;
        }
        return;
    }
    IESKF::State18 FrontEnd::readState() { return ieskf_ptr->getX(); }
}  // namespace IESKFSlam
