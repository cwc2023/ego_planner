#include "controller1.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <boost/format.hpp>
#include "std_msgs/Float32.h"

using namespace std;
using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;


//构造函数，初始化参数，调用resetThrustMapping函数重置推力映射。
LinearControl::LinearControl(Parameter_t &param) : param_(param) {

    resetThrustMapping();
}

/*
  Fast_250 low_level_controller 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/

// 主要控制函数，接受期望状态、里程计数据、IMU数据，以及一个用于输出的Controller_Output_t变量。
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u) {

    //compute disired acceleration 计算期望加速度
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp,Kv;
    Kp << param_.normal_gain.Kp0, param_.normal_gain.Kp1, param_.normal_gain.Kp2;
    Kv << param_.normal_gain.Kv0, param_.normal_gain.Kv1, param_.normal_gain.Kv2;
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0,0,param_.gra);
    //计算期望的集体推力信号
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

    //计算角度输出给姿态控制器
    double roll,pitch,yaw,yaw_imu; //计算欧拉角
    double yaw_odom = fromQuaternion2yaw(odom.q);//从四元素获取偏航角
    double sin = std::sin(yaw_odom);//计算偏航角的正弦值
    double cos = std::cos(yaw_odom);//计算偏航角的余弦值

    //根据期望的加速度和偏航角计算期望的滚转角和俯仰角
    roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;

    // yaw = fromQuaternion2yaw(des.q);//从期望的四元素获取偏航角

    yaw_imu = fromQuaternion2yaw(imu.q);//根据四元数计算出欧拉角，ros的odom消息机制为ZYX，无人机的是ZXY  //从IMU的四元素获取偏航角

    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());

    //从构建期望的四元数，采用ZYX的顺序，和无人机保持一致
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    u.q = imu.q * odom.q.inverse() * q;// 将期望的四元素从里程计坐标系转换到飞控坐标系，赋值输出变量u.q

    //将一些重要的期望值和实际值存入debug_msg_，用于调试和记录
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);

    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;
  
    // Used for thrust-accel mapping estimation
    //记录推力-时间对 用于推力-加速度映射估计
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    //如果timed_thrust_的长度超过100，就删除前面的元素，保持队列的大小为100
    while (timed_thrust_.size() > 100) {
        
        timed_thrust_.pop();
    }
    //返回debug_msg_，用于后续的调试和记录
    return debug_msg_;
  
}

/*
  compute throttle percentage 
*/
// 计算期望的集体推力信号的函数
// 这个函数根据期望的加速度和推力到加速度的比例计算期望的推力信号
double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc) {

    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

// 估计无人机推力模型的函数
// 这个函数使用带有消失记忆的递归最小二乘算法来估计推力模型
bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a) {

    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {

        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045) {// 45ms
        
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) {// 35ms
        
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        if (param_.thr_map.print_val) 
            printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        //fflush(stdout);

        debug_msg_.hover_percentage = thr2acc_;
        return true;
    }
    return false;
}


// 重置推力映射的函数
// 这个函数将推力到加速度的比例设置为基于重力加速度和悬停百分比参数的默认值
void LinearControl::resetThrustMapping(void) {

    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;
}
// 标准化向量并计算其导数的函数
// 这个函数根据重力方向对给定的向量进行标准化，并计算标准化向量的导数
void LinearControl::normalizeWithGrad(const Eigen::Vector3d &x,
    const Eigen::Vector3d &xd,
    Eigen::Vector3d &xNor,
    Eigen::Vector3d &xNord) const {

    const double xSqrNorm = x.squaredNorm();
    const double xNorm = sqrt(xSqrNorm);
    xNor = x / xNorm;
    xNord = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
    return;
}
// 根据四元数计算偏航角的函数
// 这个函数根据四元数计算并返回无人机的偏航角
double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {

    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}