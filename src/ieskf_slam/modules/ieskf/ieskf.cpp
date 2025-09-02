/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-13 16:43:29
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 16:01:59
 */
#include "ieskf_slam/modules/ieskf/ieskf.h"
namespace IESKFSlam {
    IESKF::IESKF(const std::string &config_path, const std::string &prefix)
        : ModuleBase(config_path, prefix, "IESKF") {
        // 初始化协方差
        P.setIdentity();
        P(9, 9) = P(10, 10) = P(11, 11) = 0.0001;
        P(12, 12) = P(13, 13) = P(14, 14) = 0.001;
        P(15, 15) = P(16, 16) = P(17, 17) = 0.00001;
        double cov_gyroscope, cov_acceleration, cov_bias_acceleration, cov_bias_gyroscope;
        // 直接在类里面读，不需要加前缀。
        readParam("cov_gyroscope", cov_gyroscope, 0.1);
        readParam("cov_acceleration", cov_acceleration, 0.1);
        readParam("cov_bias_acceleration", cov_bias_acceleration, 0.1);
        readParam("cov_bias_gyroscope", cov_bias_gyroscope, 0.1);
        // 初始化噪声协方差矩阵 Q
        Q.block<3, 3>(0, 0).diagonal() =
            Eigen::Vector3d{cov_gyroscope, cov_gyroscope, cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() =
            Eigen::Vector3d{cov_acceleration, cov_acceleration, cov_acceleration};
        Q.block<3, 3>(6, 6).diagonal() =
            Eigen::Vector3d{cov_bias_gyroscope, cov_bias_gyroscope, cov_bias_gyroscope};
        Q.block<3, 3>(9, 9).diagonal() =
            Eigen::Vector3d{cov_bias_acceleration, cov_bias_acceleration, cov_bias_acceleration};
        X.ba.setZero();
        X.bg.setZero();
        X.gravity.setZero();
        X.position.setZero();
        X.rotation.setIdentity();
        X.velocity.setZero();
        print_table();
    }

    IESKF::~IESKF() {}
    // 预测步 fast-lio1公式8，不需要imu信息的时间戳直接拿dt即可
    // 前向传播纯靠imu
    void IESKF::predict(IMU imu, double dt) {
        // 递推状态
        // 重载了+号
        imu.acceleration -= X.ba;
        imu.gyroscope -= X.bg;
        // X.rotation是四元数
        auto rotation = X.rotation.toRotationMatrix();
        // 简单的预测步骤
        X.rotation =
            Eigen::Quaterniond(X.rotation.toRotationMatrix() * so3Exp((imu.gyroscope) * dt)); // R_{t-1} * exp(so3(w_{t}*dt))
        X.rotation.normalize();
        X.position += X.velocity * dt;

        // 重力补偿 acc
        X.velocity += (rotation * (imu.acceleration) + X.gravity) * dt; // 这里的加速度是经过重力补偿的，x.gravity: -9.81左右，是个会优化的值。
        Eigen::Matrix<double, 18, 18> Fx;

        Eigen::Matrix<double, 18, 12> Fw;
        Fw.setZero();
        Fx.setIdentity();   // 对角线全1
        
        Fx.block<3, 3>(0, 0) = so3Exp(-1 * imu.gyroscope * dt); // Exp(-w \Delta t)

        Fx.block<3, 3>(0, 9) = -1 * A_T(-imu.gyroscope * dt) * dt;  // -A(w \Delta t) * \Delta t

        Fx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;   // 第四行第七列开始 I*\delta t
        Fx.block<3, 3>(6, 0) = rotation * skewSymmetric(imu.acceleration) * dt * (-1); // -R * [a]x * \Delta t
        Fx.block<3, 3>(6, 12) = rotation * dt * (-1);   // -R * \Delta t
        Fx.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;   // I \Delta t
        Fw.block<3, 3>(0, 0) = -1 * A_T(-imu.gyroscope * dt) * dt;  // -A(w \Delta t) * \Delta t
        Fw.block<3, 3>(6, 3) = -1 * rotation * dt;
        Fw.block<3, 3>(9, 6) = Fw.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;
        P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose();
    }
    
    // 更新步
    bool IESKF::update() {
        static int cnt_ = 0;
        auto x_k_k = X;
        auto x_k_last = X;
        ///. 开迭
        Eigen::MatrixXd K;  // 自适应维度矩阵
        Eigen::MatrixXd H_k;
        Eigen::Matrix<double, 18, 18> P_in_update;  // 协方差
        bool converge = true;
        // 固定了迭代次数
        for (int i = 0; i < iter_times; i++) {
            ///. 计算误差状态 J
            Eigen::Matrix<double, 18, 1> error_state = getErrorState18(x_k_k, X);   // P_k
            Eigen::Matrix<double, 18, 18> J_inv;
            J_inv.setIdentity();
            J_inv.block<3, 3>(0, 0) = A_T(error_state.block<3, 1>(0, 0));   // 因为求取协方差，直接用到的是J的逆
            // 更新 P
            P_in_update = J_inv * P * J_inv.transpose();

            Eigen::MatrixXd z_k; // 特征点到特征平面的距离
            Eigen::MatrixXd R_inv;
            // 调用接口计算 Z H
            // z_k 就是 m * 1
            calc_zh_ptr->calculate(x_k_k, z_k, H_k);
            std::cout << "z_k: " << z_k.rows() << " " << z_k.cols() << std::endl;
            Eigen::MatrixXd H_kt = H_k.transpose();

            // R 直接写死0.001，是雷达的测量噪声;
            K = (H_kt * H_k + (P_in_update / 0.001).inverse()).inverse() * H_kt; // R = 0.001 * I
            // 使用论文中的公式.20，并没有在状态中加入激光雷达的测量噪声，所以直接写死，原公式如下：
            // K = (H_kt * R_inv * H_k + P_in_update.inverse()).inverse() * H_kt * R_inv;

            //. 计算X 的增量 公式18
            Eigen::MatrixXd left = -1 * K * z_k;
            Eigen::MatrixXd right =
                -1 * (Eigen::Matrix<double, 18, 18>::Identity() - K * H_k) * J_inv * error_state;
            Eigen::MatrixXd update_x = left + right;  // 18x1 李代数形式。
            // 收敛判断
            converge = true;
            for (int idx = 0; idx < 18; idx++) {
                if (update_x(idx, 0) > 0.001) {
                    converge = false;
                    break;
                }
            }

            // 更新X
            x_k_k.rotation = x_k_k.rotation.toRotationMatrix() * so3Exp(update_x.block<3, 1>(0, 0));
            x_k_k.rotation.normalize();
            x_k_k.position = x_k_k.position + update_x.block<3, 1>(3, 0);
            x_k_k.velocity = x_k_k.velocity + update_x.block<3, 1>(6, 0);
            x_k_k.bg = x_k_k.bg + update_x.block<3, 1>(9, 0);
            x_k_k.ba = x_k_k.ba + update_x.block<3, 1>(12, 0);
            x_k_k.gravity = x_k_k.gravity + update_x.block<3, 1>(15, 0);

            if (converge){
                break;
            }
        }
        cnt_++;
        X = x_k_k;
        P = (Eigen::Matrix<double, 18, 18>::Identity() - K * H_k) * P_in_update;
        return converge;
    }
    // 获取误差状态向量
    // s1: last optimal state
    // s2: current state
    // return s1和s2之间的误差状态向量
    Eigen::Matrix<double, 18, 1> IESKF::getErrorState18(const State18 &s1, const State18 &s2) {
        Eigen::Matrix<double, 18, 1> es; // error-state
        es.setZero();
        // 流形中的两个旋转矩阵 
        
        es.block<3, 1>(0, 0) =
            SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix()); // 用李代数存储误差状态
        es.block<3, 1>(3, 0) = s1.position - s2.position;   // delta xyz
        es.block<3, 1>(6, 0) = s1.velocity - s2.velocity;   // delta v_xyz
        es.block<3, 1>(9, 0) = s1.bg - s2.bg;               // delta bg_xyz
        es.block<3, 1>(12, 0) = s1.ba - s2.ba;              // delta ba_xyz
        es.block<3, 1>(15, 0) = s1.gravity - s2.gravity;    // delta gravity_xyz
        return es;
    }
    const IESKF::State18 &IESKF::getX() { return X; }
    void IESKF::setX(const IESKF::State18 &x_in) { X = x_in; }
}  // namespace IESKFSlam
