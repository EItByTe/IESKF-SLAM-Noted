#pragma once 
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
#include <deque>
namespace IESKFSlam
{
    struct MeasureGroup{
        // 时间单位是s
        double lidar_begin_time;
        std::deque<IMU> imus;
        PointCloud cloud;
        double lidar_end_time;
    };
} // namespace IESKFSlam
