#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"

#include "ieskf_slam/type/point.h"
namespace IESKFSlam {

    FrontbackPropagate::FrontbackPropagate(/* args */) {}

    FrontbackPropagate::~FrontbackPropagate() {}

    // 前向传播
    void FrontbackPropagate::propagate(MeasureGroup &mg, IESKF::Ptr ieskf_ptr) {
        // 按照时间顺序排序点云,mg中只有一帧点云
        std::sort(mg.cloud.cloud_ptr->begin(), mg.cloud.cloud_ptr->end(),
                  [](Point x, Point y) { return x.offset_time < y.offset_time; });

        std::vector<IMUPose6d> IMUpose;
        auto v_imu = mg.imus;
        v_imu.push_front(last_imu_);  // last_imu_的时间戳是上一帧点云的结束时间左右，用这个时间是比较准的。
        // imu的时间：[imu_beg_time, imu_end_time]
        const double &imu_beg_time = v_imu.front().time_stamp.sec(); // 也就是last_imu_的时间戳
        const double &imu_end_time = v_imu.back().time_stamp.sec();  
        const double &pcl_beg_time = mg.lidar_begin_time;
        const double &pcl_end_time = mg.lidar_end_time;
        auto &pcl_out = *mg.cloud.cloud_ptr; // 是对象，不是指针
        // 当前状态，上一时刻的后验
        auto imu_state = ieskf_ptr->getX();
        IMUpose.clear();    // 重新开始累积imu_pose
        // 上一时刻的加速度、角速度，time为0.0
        IMUpose.emplace_back(0.0, acc_s_last, angvel_last, imu_state.velocity, imu_state.position,
                             imu_state.rotation);
        Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
        Eigen::Matrix3d R_imu;
        double dt = 0;
        IMU in;
        for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
            // 当前处理的两个imu数据，当前和下一个。用于插值
            // [cur_imu, next_imu]
            auto &&head = *(it_imu);
            auto &&tail = *(it_imu + 1);
            if (tail.time_stamp.sec() < last_lidar_end_time_) continue; // 一直到这一帧的lidar时间开始再开始处理
            // 线性插值。
            angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
            acc_avr = 0.5 * (head.acceleration + tail.acceleration);
            // std::cout << imu_scale << std::endl; // m2dgr 0.980929 0 0 9.8 是比较标准的imu读数
            acc_avr = acc_avr * imu_scale; // 缩放到正确的尺度
            // 这里的acc_avr是加速度计的加速度，单位是m/s^2 打印
            // std::cout << "acc_avr: " << acc_avr.transpose() << std::endl;
            // 这里的时间节点是这样的
            if (head.time_stamp.sec() < last_lidar_end_time_) { // [head.time_stamp, last_lidar_end_time, tail.time_stamp]
                dt = tail.time_stamp.sec() - last_lidar_end_time_;
            } else {                                            // [last_lidar_end_time, head.time_stamp, tail.time_stamp]
                dt = tail.time_stamp.sec() - head.time_stamp.sec();
            }
            in.acceleration = acc_avr;
            in.gyroscope = angvel_avr;
            // in的timestamp并没有赋值。插过值的结果放进去进行预测
            ieskf_ptr->predict(in, dt);

            imu_state = ieskf_ptr->getX();
            // 更新了状态，其中有imu的零偏，直接修正当前帧
            angvel_last = angvel_avr - imu_state.bg;
            acc_s_last = imu_state.rotation * (acc_avr - imu_state.ba); // 转换到map系下的加速度
            // 加上重力修正，所以如果有组合导航修正后的imu数据，可以去掉这里的修正。
            for (int i = 0; i < 3; i++) {
                acc_s_last[i] += imu_state.gravity[i];
            }
            // std::cout << "acc_s_last: " << acc_s_last.transpose() << std::endl;
            double &&offs_t = tail.time_stamp.sec() - pcl_beg_time; // 这个是当前imu数据相对于当前点云的时间偏移
            IMUpose.emplace_back(offs_t, acc_s_last, angvel_last, imu_state.velocity,
                                 imu_state.position, imu_state.rotation);
        }

        // 上述做到了lidar_end_time前的，[*v_imu.back()-1, imu_end_time,____ ,lidar_end_time] 当中这一段是空缺的。
        // 补一下这最后一块的后向传播。
        dt = pcl_end_time - imu_end_time;   // 因为计算dt的方式和循环中不一样，所以单独拿出来算
        ieskf_ptr->predict(in, dt);
        imu_state = ieskf_ptr->getX();
        last_imu_ = mg.imus.back(); // 就是 v_imu.back()
        last_lidar_end_time_ = pcl_end_time;
        // 后向传播 - 去畸变
        if (pcl_out.points.begin() == pcl_out.points.end()) return; // 如果点云没东西就不进行更新
        auto it_pcl = pcl_out.points.end() - 1;
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
            // 从后往前，head前一帧 tail当前帧
            auto head = it_kp - 1;
            auto tail = it_kp;
            R_imu = head->rot.toRotationMatrix();
            vel_imu = head->vel;
            pos_imu = head->pos;
            acc_imu = tail->acc;
            angvel_avr = tail->angvel;
            // 处理每个点
            for (; it_pcl->offset_time / 1e9 > head->time; it_pcl--) {
                dt = it_pcl->offset_time / 1e9 - head->time;
                Eigen::Matrix3d R_i(R_imu * so3Exp(angvel_avr * dt));
                Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z); // 当前点
                Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                                     imu_state.position);   // 
                Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei); // 转换到世界坐标系下
                it_pcl->x = P_compensate(0); 
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);
                if (it_pcl == pcl_out.points.begin()) break;
            }
        }
        return;
    }

}  // namespace IESKFSlam
