/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-08 21:05:55
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:25:59
 */
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
// ros中的一些订阅发布常规写法。
namespace ROSNoetic {
    // 构造函数
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh) {
        std::string config_file_name, lidar_topic, imu_topic;
        // 不抠细节了，这里还是从ros参数服务器中加载了，并没有完全解耦。
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        front_end_ptr =
            std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");  // 这里应该只读了frontend的配置

        // 发布者和订阅者
        cloud_subscriber =
            nh.subscribe(lidar_topic, 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        // 读取雷达类型
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if (lidar_type == AVIA) {
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        } else if (lidar_type == VELO) {
            lidar_process_ptr = std::make_shared<VelodyneProcess>();
        } else {
            std::cout << "unsupport lidar type" << std::endl;
            exit(100);
        }
        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
        path_pub = nh.advertise<nav_msgs::Path>("path", 100);
        local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
        run();
    }
    // 析构函数
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper() {}
    //每到来一帧点云数据，就会调用这个回调函数
    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg) {
        IESKFSlam::PointCloud cloud; // 一帧点云数据
        lidar_process_ptr->process(*msg, cloud);
        front_end_ptr->addPointCloud(cloud);
    }

    // 由于IMU的格式都是统一的，这边就统一用一个回调函数.
    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg) {
        IESKFSlam::IMU imu;
        // 这个时间戳里面存了秒和纳秒两种格式
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                            msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }
    // 程序开始入口
    void IESKFFrontEndWrapper::run() {
        ros::Rate rate(500);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            // true则数据同步成功，即程序正常运行。
            if (front_end_ptr->track()) {
                publishMsg();
            }
        }
    }
    // 从ieskf中读取状态，发布到ROS中
    void IESKFFrontEndWrapper::publishMsg() {
        static nav_msgs::Path path;
        // 当前状态
        auto X = front_end_ptr->readState();
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);
        IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
        // 转到当前位置下
        pcl::transformPointCloud(
            cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());
        // auto cloud =front_end_ptr->readCurrentPointCloud();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        curr_cloud_pub.publish(msg);

        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        local_map_pub.publish(msg);
    }
}  // namespace ROSNoetic
