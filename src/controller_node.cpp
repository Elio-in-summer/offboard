#include "controller/commonUsage.h"
#include "controller/pid.h"
#include "controller/rosRelated.h"
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <PosVelAcc.h>
#include <DebugPid.h>
#include "Eigen/Eigen"
#include <ctime>

#include "controller/FILTER.h"

using namespace Eigen;

#define LOOPRATE 50
#define GRAVITATIONAL_ACC 9.794 // TODO： The gravity of Shanghai

/**************************** Global Variable *******************************/
mavros_msgs::State droneState; // 飞机状态
geometry_msgs::PoseStamped pos_cur;
geometry_msgs::TwistStamped vel_cur;
nav_msgs::Odometry odom_cur;
offboard::PosVelAcc droneTargetPVA;
mavros_msgs::State uav_cur_state;
Eigen::Vector3d imu_acc;
Eigen::Vector3d imu_body_rate;
Eigen::Quaterniond imu_quat;
Eigen::Vector3d acc_in_w;
Eigen::Quaterniond odom_quat;


double targetPoseLastTimeStamp = 0.0; // 上一次目标点时间戳

geometry_msgs::Vector3 droneTargetEuler; // 飞行控制： 期望欧拉角与油门值，仅供自己输出参考
geometry_msgs::PoseStamped msgDroneTargetEulerThrust;


// TODO : TEST
ros::Publisher pubPID;
offboard::DebugPid msgDebugPid;

///========= 参数文件 =========///
float rosParamBaseThrust = 0.4;                          // 飞机基础油门值。各架飞机不同
vector<float> rosParamThrustLimit = vector<float>(2, 0); // 油门上下限
// PID 控制参数
vector<float> rosParamPosKP = vector<float>(3, 0);
vector<float> rosParamPosKI = vector<float>(3, 0);
vector<float> rosParamPosKD = vector<float>(3, 0);
vector<float> rosParamVelKP = vector<float>(3, 0);
vector<float> rosParamVelKI = vector<float>(3, 0);
vector<float> rosParamVelKD = vector<float>(3, 0);

///========= 飞行控制变量 =========///
float baseThrust = 0.4;
PID pidPX, pidPY, pidPZ;
PID pidVX, pidVY, pidVZ;
double pidZVelLast = 0;                      // 上一次z轴速度环输出的结果
geometry_msgs::Vector3 angleTarget;          // 期望欧拉角
geometry_msgs::Quaternion orientationTarget; // 发给无人机的姿态指令  期望四元数
double thrustTarget;                         // 期望油门
mavros_msgs::AttitudeTarget msgTargetAttitudeThrust;

//加速度滤波
FILTER ax_sgFilter(31,11,2),ay_sgFilter(21,11,2),az_sgFilter(31,11,2);
double ax_sg = 0;
double ay_sg = 0;
double az_sg = 0;
ros::Publisher acc_pub;
geometry_msgs::TwistStamped acc_sg;

//thrust model estimation
std::queue<std::pair<ros::Time, double>> timed_thrust_;
double rho2_ = 0.998;
double P_;
double thr2acc_;
double thrust_model_error;
double est_a_norm;
double thr_norm;
double var;

double K1 = 27.06;
double K2 = 0.74;
//double K1 = 18.16;
//double K2 = 0;
//get drone state
//acc = K1 * (K2 * thr**2 + (1 - K2) * thr), K1 will be estimated

void resetThrustMapping(void)
{
  thr2acc_ = GRAVITATIONAL_ACC / rosParamBaseThrust;
  P_ = 1e6;
}

bool estimateThrustModel(const Eigen::Vector3d &est_a)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
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
    double thr_pre = (K2 * thr * thr + (1 - K2) * thr);
    double gamma = 1 / (rho2_ + thr_pre * P_ * thr_pre);
    double K = gamma * P_ * thr_pre;
    thr_norm = thr;
    est_a_norm = est_a(2);
    thrust_model_error = (est_a_norm - (K1 * thr_pre));
    // ! est_a_norm is actually acc measured
    // ! est_a_norm - thrust_model_error is the calculated acc
    var = gamma;


    K1 = K1 + K * (est_a_norm - thr_pre * K1);
    P_ = (1 - K * thr_pre) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

double computeDesiredCollectiveThrustSignal( const Eigen::Vector3d &des_acc )
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
    //throttle_percentage = des_acc.norm() / thr2acc_;
    //here we use a new model
    // acc = K1 * (K2 * thr**2 + (1 - K2) * thr)
    // use implicit expression: thr = 0.5 * (sqrt(K2**2 + 4 * K1 * acc) - K2) / K1
    double a = K1 * K2;
    double b = K1 * (1 - K2);
    double c = -des_acc.norm();
    double discriminant = b*b - 4*a*c;
    if(discriminant < 0){
        ROS_WARN("discriminant < 0");
        return baseThrust;
    }
    throttle_percentage = 0.5 * (sqrt(discriminant) - b) / a;

  return throttle_percentage;
}
/**************************** Function Declaration *******************************/

// 回调函数
void cb_state(const mavros_msgs::State::ConstPtr &msg)
{
    droneState = *msg;
}

// void cb_pos(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     pos_cur = *msg;
// }

// void cb_vel(const geometry_msgs::TwistStamped::ConstPtr &msg)
// {
//     vel_cur = *msg;
// }

void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_cur = *msg;
    odom_quat.w() = msg->pose.pose.orientation.w;
    odom_quat.x() = msg->pose.pose.orientation.x;
    odom_quat.y() = msg->pose.pose.orientation.y;
    odom_quat.z() = msg->pose.pose.orientation.z;
}

void cb_target_pva(const offboard::PosVelAcc::ConstPtr &msg)
{
    droneTargetPVA = *msg;
    targetPoseLastTimeStamp = ros::Time::now().toSec();
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu_body_rate << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_quat.w() = msg->orientation.w;
    imu_quat.x() = msg->orientation.x;
    imu_quat.y() = msg->orientation.y;
    imu_quat.z() = msg->orientation.z;

    ax_sg = ax_sgFilter.sgfilter(imu_acc(0));
    ay_sg = ay_sgFilter.sgfilter(imu_acc(1));
    az_sg = az_sgFilter.sgfilter(imu_acc(2));

    imu_acc << ax_sg, ay_sg, az_sg;

    if(uav_cur_state.armed && uav_cur_state.mode == "OFFBOARD"){
        if (estimateThrustModel(imu_acc))
        {
            ROS_INFO("Thr2Acc: %f", thr2acc_);
        }
    }
    // imu_quat is quat from world to body, imu_acc is in body frame
    //! basically, imu_q = odom_q
    acc_in_w = imu_quat * imu_acc - Eigen::Vector3d(0, 0, GRAVITATIONAL_ACC);
    //! acc_in_w is filtered acc in world frame
    //! can be compared with set_pva or total_out_a,so we can see if DOB is needed

}

void uav_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_cur_state = *msg;
}

/**
 * 将欧拉角转化为四元数, 欧拉角的顺序为Yaw(z)-Pitch(y)-Roll(x)
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    temp.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return temp;
}

geometry_msgs::Quaternion euler2quaternion_ros(geometry_msgs::Vector3 v_)
{
    return euler2quaternion(v_.x, v_.y, v_.z);
}

Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond temp;
    temp.w() = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.x() = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    temp.y() = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    temp.z() = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式，输入的顺序为x-y-z-w,欧拉角的顺序为Yaw(z)-Pitch(y)-Roll(x)
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (w * y - z * x));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

geometry_msgs::Vector3 quaternion2Euler_ros(geometry_msgs::Quaternion q)
{
    return quaternion2euler(q.x, q.y, q.z, q.w);
}

tf::StampedTransform get_stamped_transform_from_pose(float x, float y, float z, geometry_msgs::Quaternion &q)
{
    // publish our transform on TF
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    trans.setRotation(quat);
    tf::Vector3 orig(x, y, z);
    trans.setOrigin(orig);
    return trans;
}




/**
 * @brief 从rosparam中的参数赋给PID
 */
bool px4AttitudeCtlInitRosParam()
{
    /// 使用 rosParam 读取参数
    /// 设置基础油门值
    baseThrust = rosParamBaseThrust;
    /// 设置位置环 PID
    pidPX.setPID(rosParamPosKP[0], rosParamPosKI[0], rosParamPosKD[0]);
    pidPY.setPID(rosParamPosKP[1], rosParamPosKI[1], rosParamPosKD[1]);
    pidPZ.setPID(rosParamPosKP[2], rosParamPosKI[2], rosParamPosKD[2]);
    pidPX.set_sat(1.0, 2, 0);
    pidPY.set_sat(0.6, 2, 0);
    pidPZ.set_sat(1.5, 3, 0); // TODO: Z轴位置积分上限
    pidPX.start_intergrate_flag = false;
    pidPY.start_intergrate_flag = false;
    pidPZ.start_intergrate_flag = false;
    /// 设置速度环 PID
    pidVX.setPID(rosParamVelKP[0], rosParamVelKI[0], rosParamVelKD[0]);
    pidVY.setPID(rosParamVelKP[1], rosParamVelKI[1], rosParamVelKD[1]);
    pidVZ.setPID(rosParamVelKP[2], rosParamVelKI[2], rosParamVelKD[2]);
    pidVX.set_sat(0.2, 2, 0);
    pidVY.set_sat(0.2, 2, 0);
    pidVZ.set_sat(0.3, 3, 0);
    pidVX.start_intergrate_flag = false;
    pidVY.start_intergrate_flag = false;
    pidVZ.start_intergrate_flag = false;
    return true;
}

/**
 * @brief 最重要的计算函数
 *
 * @param _currTime 当前时间
 * @param CurrPos   当前位置
 * @param CurrVel   当前速度
 * @param TargetPos 目标位置
 * @param TargetVel 目标速度
 * @param TargetAcc 目标加速度
 * @param TargetYaw 目标yaw角度
 * @param state     无人机此时的状态，是否解锁，是否进入OFFBOARD
 */
void px4AttitudeCtlPVA(double _currTime,
                       Eigen::Vector3d CurrPos,
                       Eigen::Vector3d CurrVel,
                       Eigen::Vector3d TargetPos,
                       Eigen::Vector3d TargetVel,
                       Eigen::Vector3d TargetAcc,
                       double TargetYaw,
                       mavros_msgs::State &state)
{
    /// 如果处于Offboard状态再用积分环节
    if (state.mode == "OFFBOARD")
    {
        /// 位于较好的初始位置，再启用积分环节
        if ((CurrPos - TargetPos).norm() < 1)
        {
            ROS_WARN_ONCE("[Controller] Start intergrate in PID");
            pidPX.start_intergrate_flag = true;
            pidPY.start_intergrate_flag = true;
            pidPZ.start_intergrate_flag = true;
            pidVX.start_intergrate_flag = true;
            pidVY.start_intergrate_flag = true;
            pidVZ.start_intergrate_flag = true;
        }
        // else{  // TODO
        //     ROS_WARN_ONCE("[Controller] Stop intergrate in PID");
        //     pidPX.start_intergrate_flag = false;
        //     pidPY.start_intergrate_flag = false;
        //     pidPZ.start_intergrate_flag = false;
        //     pidVX.start_intergrate_flag = false;
        //     pidVY.start_intergrate_flag = false;
        //     pidVZ.start_intergrate_flag = false;
        // }
    }

    // 位置环误差计算
    Eigen::Vector3d posError, velError;
    posError = TargetPos - CurrPos;
    velError = TargetVel - CurrVel;
    pidPX.add_error(posError.x(), _currTime);
    pidPY.add_error(posError.y(), _currTime);
    pidPZ.add_error(posError.z(), _currTime);
    pidVX.add_error(velError.x(), _currTime);
    pidVY.add_error(velError.y(), _currTime);
    pidVZ.add_error(velError.z(), _currTime);

    pidPX.pid_output();
    pidPY.pid_output();
    pidPZ.pid_output();
    pidVX.pid_output();
    pidVY.pid_output();
    pidVZ.pid_output();

    Eigen::Vector3d accExcept;
    Eigen::Vector3d accExcept_g;
    accExcept << pidPX.Output + pidVX.Output + TargetAcc[0],
        pidPY.Output + pidVY.Output + TargetAcc[1],
        pidPZ.Output + pidVZ.Output + TargetAcc[2];

    // TODO:限制Z方向加速度
    accExcept[2] = min(7.0, max(accExcept[2], -7.0));

    accExcept_g = accExcept + Eigen::Vector3d(0, 0, 1) * GRAVITATIONAL_ACC;

    // TODO:Limit control angle to 15 degree
    // double theta_ = M_PI * 15.0 / 180.0;
    // double cos_ = cos(theta_);
    // if (Eigen::Vector3d(0, 0, 1).dot(accExcept_g.normalized()) < cos_)
    // {
    //     double nf_ = accExcept.norm();
    //     double A_ = cos_ * cos_ * nf_ * nf_ - accExcept(2) * accExcept(2);
    //     double B_ = 2 * (cos_ * cos_ - 1) * accExcept(2) * GRAVITATIONAL_ACC;
    //     double C_ = (cos_ * cos_ - 1) * GRAVITATIONAL_ACC * GRAVITATIONAL_ACC;
    //     double s = (-B_ + sqrt(B_ * B_ - 4 * A_ * C_)) / (2 * A_);
    //     accExcept_g.noalias() = s * accExcept + GRAVITATIONAL_ACC * Eigen::Vector3d(0, 0, 1);
    //     ROS_WARN("The control reaches the limitation");
    // }
    // Limit control angle to 10 degree

    // Add control for yaw.
    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b2d(-sin(TargetYaw), cos(TargetYaw), 0);
    if (accExcept_g.norm() > 1e-6)
    {
        b3c.noalias() = accExcept_g.normalized();
    }
    else
    {
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);
    }
    b1c.noalias() = b2d.cross(b3c).normalized();
    b2c.noalias() = b3c.cross(b1c).normalized();
    Eigen::Matrix3d R_;
    Eigen::Quaterniond qua_;
    R_ << b1c, b2c, b3c;
    qua_ = Eigen::Quaterniond(R_);
    orientationTarget.w = qua_.w();
    orientationTarget.x = qua_.x();
    orientationTarget.y = qua_.y();
    orientationTarget.z = qua_.z();

    // ! TODO:限制总推力
    // thrustTarget = accExcept_g.norm() / GRAVITATIONAL_ACC * (baseThrust); // 目标推力值
    thrustTarget = computeDesiredCollectiveThrustSignal(accExcept_g);
    // thrustTarget = min(max(thrustTarget, (double)rosParamThrustLimit[0]), (double)rosParamThrustLimit[1]);

    msgTargetAttitudeThrust.orientation = orientationTarget;
    msgTargetAttitudeThrust.thrust = thrustTarget;
    msgTargetAttitudeThrust.header.stamp = ros::Time::now();
    angleTarget = quaternion2Euler_ros(orientationTarget);
    ROS_INFO("target Yaw = %.3f |Pitch = %.3f | Roll = %.3f | thrust = %.3f", angleTarget.z, angleTarget.y, angleTarget.x, thrustTarget);

    // TODO: TEST
    // ROS_INFO_STREAM("CurrPos  :" << CurrPos[0] << " " << CurrPos[1] << " " << CurrPos[2]);
    // ROS_INFO_STREAM("TargetPos:" << TargetPos[0] << " " << TargetPos[1] << " " << TargetPos[2]);
    // ROS_INFO_STREAM("posError :" << posError[0] << " " << posError[1] << " " << posError[2]);
    // ROS_INFO_STREAM("OUT pos:" << pidPX.Output << " " << pidPY.Output << " " << pidPZ.Output);
    // ROS_INFO_STREAM("CurrVel  :" << CurrVel[0] << " " << CurrVel[1] << " " << CurrVel[2]);
    // ROS_INFO_STREAM("TargetVel:" << TargetVel[0] << " " << TargetVel[1] << " " << TargetVel[2]);
    // ROS_INFO_STREAM("velError :" << velError[0] << " " << velError[1] << " " << velError[2]);
    // ROS_INFO_STREAM("OUT vel:" << pidVX.Output << " " << pidVY.Output << " " << pidVZ.Output);
    // ROS_INFO_STREAM("TargetAcc:" << TargetAcc[0] << " " << TargetAcc[1] << " " << TargetAcc[2]);
    // ROS_INFO_STREAM("accExcept:" << accExcept[0] << " " << accExcept[1] << " " << accExcept[2]);
    msgDebugPid.header.stamp = ros::Time::now();
    msgDebugPid.PError.x = posError[0];
    msgDebugPid.PError.y = posError[1];
    msgDebugPid.PError.z = posError[2];
    msgDebugPid.POut.x = pidPX.Output;
    msgDebugPid.POut.y = pidPY.Output;
    msgDebugPid.POut.z = pidPZ.Output;
    msgDebugPid.VError.x = velError[0];
    msgDebugPid.VError.y = velError[1];
    msgDebugPid.VError.z = velError[2];
    msgDebugPid.VOut.x = pidVX.Output;
    msgDebugPid.VOut.y = pidVY.Output;
    msgDebugPid.VOut.z = pidVZ.Output;
    msgDebugPid.AOut.x = TargetAcc[0];
    msgDebugPid.AOut.y = TargetAcc[1];
    msgDebugPid.AOut.z = TargetAcc[2];
    msgDebugPid.TotalOut.x = accExcept[0];
    msgDebugPid.TotalOut.y = accExcept[1];
    msgDebugPid.TotalOut.z = accExcept[2];
    msgDebugPid.P_.data = P_;
    msgDebugPid.model_error.data = thrust_model_error;
    msgDebugPid.base_thrust.data = K1;
    msgDebugPid.est_a_norm.data = est_a_norm;
    msgDebugPid.thr_norm.data = thr_norm;
    msgDebugPid.var.data = P_;
    msgDebugPid.acc_in_w.x = acc_in_w(0);
    msgDebugPid.acc_in_w.y = acc_in_w(1);
    msgDebugPid.acc_in_w.z = acc_in_w(2);
    msgDebugPid.imu_q.w = imu_quat.w();
    msgDebugPid.imu_q.x = imu_quat.x();
    msgDebugPid.imu_q.y = imu_quat.y();
    msgDebugPid.imu_q.z = imu_quat.z();
    msgDebugPid.odom_q.w = odom_quat.w();
    msgDebugPid.odom_q.x = odom_quat.x();
    msgDebugPid.odom_q.y = odom_quat.y();
    msgDebugPid.odom_q.z = odom_quat.z();
    pubPID.publish(msgDebugPid);
}

int main(int argc, char **argv)
{
    // node init
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Rate rate(LOOPRATE);

    // load parameter file
    const string &node_name = ros::this_node::getName();
    nh.getParam(node_name + "/" + "BaseThrust", rosParamBaseThrust);
    nh.getParam(node_name + "/" + "ThrustLimit", rosParamThrustLimit);
    nh.getParam(node_name + "/" + "posKP", rosParamPosKP);
    nh.getParam(node_name + "/" + "posKI", rosParamPosKI);
    nh.getParam(node_name + "/" + "posKD", rosParamPosKD);
    nh.getParam(node_name + "/" + "velKP", rosParamVelKP);
    nh.getParam(node_name + "/" + "velKI", rosParamVelKI);
    nh.getParam(node_name + "/" + "velKD", rosParamVelKD);
    std::cout << "BaseThrust: " << rosParamBaseThrust << endl
              << "rosParamThrustLimit: " << rosParamThrustLimit[0] << "  ~  " << rosParamThrustLimit[1] << endl
              << "posKP: " << rosParamPosKP[0] << "    " << rosParamPosKP[1] << "    " << rosParamPosKP[2] << endl
              << "velKP: " << rosParamVelKP[0] << "    " << rosParamVelKP[1] << "    " << rosParamVelKP[2] << endl
              << "velKI: " << rosParamVelKI[0] << "    " << rosParamVelKI[1] << "    " << rosParamVelKI[2] << endl
              << "velKD: " << rosParamVelKD[0] << "    " << rosParamVelKD[1] << "    " << rosParamVelKD[2] << endl;

    // Check thrust range
    if ((rosParamThrustLimit[1] - rosParamBaseThrust) < 0.05 || (rosParamThrustLimit[0] - rosParamBaseThrust) > -0.05)
    {
        ROS_WARN_STREAM("The param `ThrustLimit` is unreasonable, EXIT !!!!!!!!");
        return 0;
    }
    // reset thrust mapping
    resetThrustMapping();

    /// subscriber
    ros::Subscriber subState = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &cb_state);
    // ros::Subscriber subLocalPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &cb_pos);
    // ros::Subscriber subLocalVel = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &cb_vel);
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("mavros/vision_pose/odom", 1, &cb_odom);
    ros::Subscriber subTargetPVA = nh.subscribe<offboard::PosVelAcc>("setpoint_pva", 1, &cb_target_pva);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
    ros::Subscriber uav_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, uav_state_cb);
    
    /// publisher
    ros::Publisher pubPx4Attitude = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    ros::Publisher pubDroneTargetEulerThrust = nh.advertise<geometry_msgs::PoseStamped>("controller/drone_target_euler_thrust", 1);
    acc_pub = nh.advertise<geometry_msgs::TwistStamped>("/hxl_uav/mocap/acc_sg", 10);
    pubPID = nh.advertise<offboard::DebugPid>("controller/pid", 1);

    ROS_INFO("Subscriber: %s", subState.getTopic().c_str());
    // ROS_INFO("Subscriber: %s", subLocalPos.getTopic().c_str());
    // ROS_INFO("Subscriber: %s", subLocalVel.getTopic().c_str());
    // ROS_INFO("Subscriber: %s", subOdom.getTopic().c_str());
    ROS_INFO("Subscriber: %s", subTargetPVA.getTopic().c_str());
    ROS_INFO("Publisher: %s", pubPx4Attitude.getTopic().c_str());
    ROS_INFO("Publisher: %s", pubDroneTargetEulerThrust.getTopic().c_str());

    if (!px4AttitudeCtlInitRosParam())
    {
        ROS_ERROR("px4AttitudeCtlInitRosParam failed , exit !!!");
        return 0;
    }
    ros::Duration(2).sleep();

    // Wait for connection with fcu
    while (ros::ok() && droneState.connected == 0)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected !!");

    // Get the current aircraft attitude angle for initializing the yaw angle
    int count_yaw_init = 0;
    double yaw_init = 0.0;
    double yaw_last = 0.0;
    double yaw_thre = M_PI / (4.0 * LOOPRATE); // TODO: 最大yaw角变化量(角速度)
    geometry_msgs::Vector3 preCurrEuler;
    while (ros::ok() && count_yaw_init < 30)
    {
        ros::spinOnce();
        preCurrEuler = quaternion2Euler_ros(odom_cur.pose.pose.orientation);
        // preCurrEuler = quaternion2Euler_ros(pos_cur.pose.orientation);
        if (abs(preCurrEuler.z) > 0.000001)
        {
            yaw_init += preCurrEuler.z;
            count_yaw_init++;
        }
        rate.sleep();
    }
    yaw_init = yaw_init / 30.0;
    ROS_INFO("Yaw initialized !! Yaw is %f", yaw_init);
    yaw_last = yaw_init;

    // 判断当前有无目标位置传入，若没有，则等待，不需要计算期望姿态
    while (ros::ok() && droneState.connected && abs(targetPoseLastTimeStamp - ros::Time::now().toSec()) > 0.1)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "not target position given, targetPoseLastTimeStamp: %f, droneState: %s",
                          targetPoseLastTimeStamp, droneState.mode.c_str());
        rate.sleep();
    }

    // 临时变量，用于传入当前状态与目标点
    Eigen::Vector3d odomCurrPos;
    Eigen::Vector3d odomCurrVel;
    Eigen::Vector3d TargetPos;
    Eigen::Vector3d TargetVel;
    Eigen::Vector3d TargetAcc;

    double TargetYaw;
    ros::Time startTime = ros::Time::now();
    double relativeTime;

    while (ros::ok() && droneState.connected)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "droneState: %s", droneState.mode.c_str());

        // 赋予当前位置速度，期望位置和yaw角度
        relativeTime = (odom_cur.header.stamp - startTime).toSec();
        odomCurrPos << odom_cur.pose.pose.position.x, odom_cur.pose.pose.position.y, odom_cur.pose.pose.position.z;
        odomCurrVel << odom_cur.twist.twist.linear.x, odom_cur.twist.twist.linear.y, odom_cur.twist.twist.linear.z;
        // relativeTime = (pos_cur.header.stamp - startTime).toSec();
        // odomCurrPos << pos_cur.pose.position.x, pos_cur.pose.position.y, pos_cur.pose.position.z;
        // odomCurrVel << vel_cur.twist.linear.x, vel_cur.twist.linear.y, vel_cur.twist.linear.z;
        TargetPos << droneTargetPVA.px, droneTargetPVA.py, droneTargetPVA.pz;
        TargetVel << droneTargetPVA.vx, droneTargetPVA.vy, droneTargetPVA.vz;
        TargetAcc << droneTargetPVA.ax, droneTargetPVA.ay, droneTargetPVA.az;
        TargetYaw = droneTargetPVA.yaw;

        // double yaw_diff = fmod(abs(TargetYaw - yaw_last), M_PI_2);
        // if (yaw_diff > yaw_thre)
        // {
        //     ROS_WARN_STREAM_THROTTLE(1, "The difference between expected yaw(" << TargetYaw << ")and last yaw(" << yaw_last << ")is too large");
        //     ROS_WARN_STREAM_THROTTLE(1, "it will cause serious drift in the T265 localization data, so the previous yaw will be used");
        //     TargetYaw = yaw_last;
        // }
        // else
        // {
        //     yaw_last = TargetYaw;
        // }

        // 传入PX4串级PID程序解算，并发布控制话题
        px4AttitudeCtlPVA(relativeTime, odomCurrPos, odomCurrVel, TargetPos, TargetVel, TargetAcc, TargetYaw, droneState);
        pubPx4Attitude.publish(msgTargetAttitudeThrust);
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), msgTargetAttitudeThrust.thrust));
        while (timed_thrust_.size() > 100)
        {
            timed_thrust_.pop();
        }

        // 可视化欧拉角姿态与油门值
        msgDroneTargetEulerThrust.header.stamp = ros::Time::now();
        msgDroneTargetEulerThrust.pose.position.x = msgTargetAttitudeThrust.thrust;
        msgDroneTargetEulerThrust.pose.orientation = msgTargetAttitudeThrust.orientation;
        pubDroneTargetEulerThrust.publish(msgDroneTargetEulerThrust);

        rate.sleep();
    }

    return 0;
}
