// Created by zph on 2021/1/12.
// Modified by wzy on 2022/12/28.
// Modified by mzy on 2023/5/4.

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include "Eigen/Eigen"
#include "math.h"
#include <PosVelAcc.h>
#include <boost/filesystem.hpp>

/**************************** Global Variable *******************************/
mavros_msgs::State uav_cur_state;
geometry_msgs::PoseStamped uav_cur_pose;
geometry_msgs::PoseStamped takeoff_pose;
geometry_msgs::PoseStamped offb_hover_pose; // pose to hover in offboard mode

float hover_height = 1.0;
double r_circle = 1.0;
double t_total = 30.0;
double omege = 2 * M_PI / 30.0;
int count_circle = 0;
int count_total = 25 * 30;

int ctrl_rate = 25;
bool repeat_path = false;

/**************************** Function Declaration and Definition *******************************/
// Tool Function
template <typename T>
void readParam(ros::NodeHandle &nh, std::string param_name, T &loaded_param)
{
    // template to read param from roslaunch
    const std::string &node_name = ros::this_node::getName();
    param_name = node_name + "/" + param_name;
    if (!nh.getParam(param_name, loaded_param))
    {
        ROS_WARN_STREAM("Fail to load " << param_name << ", use default value:" << loaded_param);
    }
    else
    {
        ROS_INFO_STREAM("Load successfully " << param_name << ": " << loaded_param);
    }
}

void pose2pva(geometry_msgs::PoseStamped pose, offboard::PosVelAcc &pva)
{
    pva.px = pose.pose.position.x;
    pva.py = pose.pose.position.y;
    pva.pz = pose.pose.position.z;
    pva.vx = 0.0;
    pva.vy = 0.0;
    pva.vz = 0.0;
    pva.ax = 0.0;
    pva.ay = 0.0;
    pva.az = 0.0;
    pva.yaw = 0.0;
}

void loadRosParams(ros::NodeHandle &nh)
{
    readParam<int>(nh, "ctrl_rate", ctrl_rate);
    readParam<bool>(nh, "repeat_path", repeat_path);
    readParam<float>(nh, "hover_height", hover_height);
    readParam<double>(nh, "r_circle", r_circle);
    readParam<double>(nh, "t_total", t_total);
}

// callback function
void uav_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_cur_state = *msg;
}

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_cur_pose = *msg;
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "offboard_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // load param
    loadRosParams(nh);
    // ros pub and sub
    ros::Subscriber uav_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, uav_state_cb);
    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 2, uav_pose_cb);
    ros::Publisher offb_setpva_pub = nh.advertise<offboard::PosVelAcc>("/setpoint_pva", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate ctrl_loop(ctrl_rate);

    // circle param
    omege = 2 * M_PI / t_total;
    count_circle = 0;
    count_total = int(ctrl_rate * t_total);

    // wait for FCU connection
    while (ros::ok() && !uav_cur_state.connected)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "wait for FCU connection");
        ctrl_loop.sleep();
    }

    /**
     * Pre Step :
     * Send a few setpoints before starting
     * Before entering Offboard mode,
     * you must have already started streaming setpoints.
     * Otherwise the mode switch will be rejected.
     */
    offboard::PosVelAcc pva_before_offb;
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        pose2pva(uav_cur_pose, pva_before_offb);
        offb_setpva_pub.publish(pva_before_offb);
        ctrl_loop.sleep();
    }

    // takeoff: Record whether the uav has taken off
    // just_takeoff: check whether the uav change into offboard mode from other mode
    bool takeoff = false;
    bool just_takeoff = true;

    offboard::PosVelAcc pva_offb_hover;
    pose2pva(uav_cur_pose, pva_offb_hover);

    /**************************** main ctrl loop *******************************/
    while (ros::ok())
    {
        ros::spinOnce();
        if (uav_cur_state.armed)
        {
            if (uav_cur_state.mode == "OFFBOARD")
            {
                ROS_INFO_STREAM_THROTTLE(1, "\033[33m OFFBOARD Mode \033[0m");
                ROS_INFO( "\033[33m ========================= \033[0m");

                // Take off to some height
                if (!takeoff)
                {
                    offboard::PosVelAcc pva_climb;
                    geometry_msgs::PoseStamped climb_pose; // 预计爬升的高度
                    takeoff_pose.pose.position.x = uav_cur_pose.pose.position.x;
                    takeoff_pose.pose.position.y = uav_cur_pose.pose.position.y;
                    takeoff_pose.pose.position.z = hover_height;
                    climb_pose.pose.position.z = uav_cur_pose.pose.position.z;

                    double err = 0.1; // TODO: 高度阈值
                    while (fabs(takeoff_pose.pose.position.z - uav_cur_pose.pose.position.z) > err)
                    {
                        ros::spinOnce();
                        climb_pose.pose.position.x = takeoff_pose.pose.position.x;
                        climb_pose.pose.position.y = takeoff_pose.pose.position.y;
                        climb_pose.pose.position.z = climb_pose.pose.position.z + 0.2 / double(ctrl_rate); // TODO:上升速度 0.2m/s
                        climb_pose.pose.position.z = std::min(climb_pose.pose.position.z, takeoff_pose.pose.position.z);
                        pose2pva(climb_pose, pva_climb);
                        offb_setpva_pub.publish(pva_climb);
                        ROS_INFO("[Takeoff]: Target xyz: %.3f, %.3f, %.3f | Curr xyz: %.3f, %.3f, %.3f",
                                 climb_pose.pose.position.x, climb_pose.pose.position.y, climb_pose.pose.position.z,
                                 uav_cur_pose.pose.position.x, uav_cur_pose.pose.position.y, uav_cur_pose.pose.position.z);
                        ctrl_loop.sleep();
                    }
                    ROS_INFO_STREAM("\033[33m Take off done \033[0m");
                    takeoff = true;
                    offb_hover_pose = takeoff_pose;
                }

                // TODO: Switch into offboard mode just now, stay still for a while, about several seconds
                if (just_takeoff)
                {
                    for (int i = 150; ros::ok() && i > 0; --i)
                    {
                        ros::spinOnce();
                        pose2pva(takeoff_pose, pva_before_offb);
                        offb_setpva_pub.publish(pva_before_offb);
                        ctrl_loop.sleep();
                        ROS_INFO("Before trajectory begin, wait for several seconds ...");
                    }
                    just_takeoff = false;
                    offb_hover_pose = takeoff_pose;
                }

                /**
                 * x = r*sin(w*t)
                 * y = r*(1-cos(w*t))
                 * vx = r*w*cos(w*t)
                 * vy = r*w*sin(w*t)
                 * ax = -r*w^2*sin(w*t)
                 * ay = r*w^2*cos(w*t)
                 *
                 * t = n/k, k为控制频率，n为计数点
                 */
                ROS_INFO_STREAM_ONCE("\033[33m In Offboard mode, start circling! \033[0m");
                if (count_circle < count_total + 1)
                {
                    offboard::PosVelAcc pva_traj;
                    pva_traj.header.frame_id = "global";
                    pva_traj.header.stamp = ros::Time::now();
                    double phi = double(count_circle) / double(ctrl_rate) * omege;

                    /**
                     *  Here is the code for testing.
                     *  When running a specific case,
                     *  please comment out the other cases.
                     */

                    /**
                     *  case 0 hovering
                     */
                    pva_traj.px = 0 + takeoff_pose.pose.position.x;
                    pva_traj.py = 0 + takeoff_pose.pose.position.y;
                    pva_traj.pz = hover_height;
                    pva_traj.vx = 0;
                    pva_traj.vy = 0;
                    pva_traj.vz = 0;
                    pva_traj.ax = 0;
                    pva_traj.ay = 0;
                    pva_traj.az = 0;
                    pva_traj.yaw = 0.0;

                    /**
                     *  case 1 circling WITHOUT yaw rotating
                     */
                    // pva_traj.px = r_circle * sin(phi) + takeoff_pose.pose.position.x;
                    // pva_traj.py = r_circle * (1 - cos(phi)) + takeoff_pose.pose.position.y;
                    // pva_traj.pz = hover_height;
                    // pva_traj.vx = 0;
                    // pva_traj.vy = 0;
                    // pva_traj.vz = 0;
                    // pva_traj.ax = 0;
                    // pva_traj.ay = 0;
                    // pva_traj.az = 0;
                    // pva_traj.yaw = 0.0;

                    /**
                     *  case 2  yaw rotating WITHOUT moving
                     */
                    // pva_traj.px = 0 + takeoff_pose.pose.position.x;
                    // pva_traj.py = 0 + takeoff_pose.pose.position.y;
                    // pva_traj.pz = hover_height;
                    // pva_traj.vx = 0;
                    // pva_traj.vy = 0;
                    // pva_traj.vz = 0;
                    // pva_traj.ax = 0;
                    // pva_traj.ay = 0;
                    // pva_traj.az = 0;
                    // pva_traj.yaw = phi;

                    /**
                     *  case 3 circling WITH yaw rotating
                     */
                    // pva_traj.px = r_circle * sin(phi) + takeoff_pose.pose.position.x;
                    // pva_traj.py = r_circle * (1 - cos(phi)) + takeoff_pose.pose.position.y;
                    // pva_traj.pz = hover_height;
                    // pva_traj.vx = 0;
                    // pva_traj.vy = 0;
                    // pva_traj.vz = 0;
                    // pva_traj.ax = 0;
                    // pva_traj.ay = 0;
                    // pva_traj.az = 0;
                    // pva_traj.yaw = phi;

                    /**
                     *  case 4 circling WITH yaw rotating,
                     *         specify velocity and accleration
                     */
                    // pva_traj.px = r_circle * sin(phi) + takeoff_pose.pose.position.x;
                    // pva_traj.py = r_circle * (1 - cos(phi)) + takeoff_pose.pose.position.y;
                    // pva_traj.pz = hover_height;
                    // pva_traj.vx = r_circle * omege * cos(phi);
                    // pva_traj.vy = r_circle * omege * sin(phi);
                    // pva_traj.vz = 0;
                    // pva_traj.ax = -r_circle * pow(omege, 2) * sin(phi);
                    // pva_traj.ay = r_circle * pow(omege, 2) * cos(phi);
                    // pva_traj.az = 0;
                    // pva_traj.yaw = phi;

                    offb_setpva_pub.publish(pva_traj);
                    offb_hover_pose = uav_cur_pose; // TODO: the pose for hovering can be the last target pose
                    count_circle++;
                    ROS_INFO("[trajectory] xyz: %.3f, %.3f, %.3f", pva_traj.px, pva_traj.py, pva_traj.pz);
                }
                else
                {
                    if (repeat_path)
                    {
                        ROS_WARN("=============== Repeat the trajectory ===============");
                        count_circle = 0; // 从头开始 pva模式不要重复路径
                    }
                    else
                    {
                        ROS_WARN_THROTTLE(1, "=============== Trajectory finished, Hover ... ===============");
                        pose2pva(offb_hover_pose, pva_offb_hover);
                        offb_setpva_pub.publish(pva_offb_hover);
                    }
                }
            }
            else
            { // If not offb
                ROS_INFO_STREAM_THROTTLE(1, " waiting for Offboard cmd");
                offboard::PosVelAcc pva_curr;
                pose2pva(uav_cur_pose, pva_curr);
                offb_setpva_pub.publish(pva_curr);
                just_takeoff = true;
            }
        }
        else
        { // If not arm
            ROS_INFO_STREAM_THROTTLE(1, " waiting for Vehicle arm");
            offboard::PosVelAcc pva_curr;
            pose2pva(uav_cur_pose, pva_curr);
            offb_setpva_pub.publish(pva_curr);
            takeoff = false;
            just_takeoff = true;
        }

        ctrl_loop.sleep();
    }

    return 0;
}
