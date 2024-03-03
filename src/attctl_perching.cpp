// Created by zph on 2021/1/12.
// Modified by wzy on 2022/12/28.
// Modified by mzy on 2023/5/4.

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include "Eigen/Eigen"
#include "math.h"
#include <PosVelAcc.h>
#include <boost/filesystem.hpp>

/**************************** Global Variable *******************************/
mavros_msgs::State uav_cur_state;
geometry_msgs::PoseStamped uav_cur_pose;
geometry_msgs::PoseStamped hover_pose;
geometry_msgs::PoseStamped takeoff_pose;

int ctrl_rate = 50;
bool repeat_path = false;

bool has_goal = false;

// execute_flag = 0: TAKEOFF
// execute_flag = 1: TRAJ
// execute_flag = 2: HOVER
int execute_flag = 0;

/**************************** Function Declaration and Definition *******************************/
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

// callback function
void uav_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_cur_state = *msg;
}

void uav_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_cur_pose.pose = msg->pose.pose;
}

void execute_flag_cb(const std_msgs::Int8ConstPtr &msg)
{
    execute_flag = msg->data;
}

void goal_init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    takeoff_pose = *msg;
    has_goal = true;
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "offboard_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // ros pub and sub
    ros::Subscriber uav_state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, uav_state_cb);
    ros::Subscriber uav_pose_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/vision_pose/odom", 2, uav_pose_cb);
    ros::Subscriber execute_flag_sub = nh.subscribe<std_msgs::Int8>(
        "/palnner_execute_flag", 1, execute_flag_cb);
    ros::Subscriber goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/goal_init_pose", 1, goal_init_pose_cb);
    ros::Publisher offb_setpva_pub = nh.advertise<offboard::PosVelAcc>(
        "/setpoint_pva", 10);
    ros::Publisher is_stable_pub = nh.advertise<std_msgs::Bool>(
        "/is_init_stable", 1);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate ctrl_loop(ctrl_rate);

    // wait for FCU connection
    while (ros::ok() && !uav_cur_state.connected)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "wait for FCU connection");
        ctrl_loop.sleep();
    }

    // wait for init goal
    while (ros::ok() && !has_goal)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "wait for init goal");
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
    bool takeoff = false;
    int stable_count = 0;

    offboard::PosVelAcc pva_offb_hover;
    pose2pva(uav_cur_pose, pva_offb_hover);

    /**************************** main ctrl loop *******************************/
    while (ros::ok())
    {
        ros::spinOnce();
        if (uav_cur_state.armed && uav_cur_state.mode == "OFFBOARD")
        {
            if (execute_flag == 0)
            {
                if (!takeoff)
                {
                    offboard::PosVelAcc pva_climb;
                    geometry_msgs::PoseStamped climb_pose; // 预计爬升的高度
                    climb_pose.pose.position.z = uav_cur_pose.pose.position.z;

                    double err = 0.1; // TODO: 高度阈值
                    while (fabs(takeoff_pose.pose.position.z - uav_cur_pose.pose.position.z) > err)
                    {
                        ros::spinOnce();
                        climb_pose.pose.position.x = takeoff_pose.pose.position.x;
                        climb_pose.pose.position.y = takeoff_pose.pose.position.y;
                        // TODO:上升速度 0.3m/s
                        climb_pose.pose.position.z = climb_pose.pose.position.z +
                                                     0.3 / double(ctrl_rate);
                        climb_pose.pose.position.z = std::min(climb_pose.pose.position.z,
                                                              takeoff_pose.pose.position.z);
                        pose2pva(climb_pose, pva_climb);
                        offb_setpva_pub.publish(pva_climb);
                        ROS_INFO_THROTTLE(1, "Takeoff]: Target: %.2f, %.2f, %.2f | Cur: %.2f, %.2f, %.2f",
                                          climb_pose.pose.position.x,
                                          climb_pose.pose.position.y,
                                          climb_pose.pose.position.z,
                                          uav_cur_pose.pose.position.x,
                                          uav_cur_pose.pose.position.y,
                                          uav_cur_pose.pose.position.z);
                        ctrl_loop.sleep();
                    }
                    ROS_INFO_STREAM("\033[33m Take off done \033[0m");
                    takeoff = true;
                }

                offboard::PosVelAcc pva_hover;
                pose2pva(takeoff_pose, pva_hover);
                offb_setpva_pub.publish(pva_hover);

                // calculate the distance between current pose and goal pose
                double distance = sqrt(
                    pow(uav_cur_pose.pose.position.x - takeoff_pose.pose.position.x, 2) +
                    pow(uav_cur_pose.pose.position.y - takeoff_pose.pose.position.y, 2) +
                    pow(uav_cur_pose.pose.position.z - takeoff_pose.pose.position.z, 2));

                if (distance < 0.1) // TODO: 调整阈值
                {
                    stable_count++;
                }
                else
                {
                    stable_count = 0;
                    ROS_INFO_THROTTLE(1, "[HOVER]: Target: %.2f, %.2f, %.2f | Cur: %.2f, %.2f, %.2f",
                                      takeoff_pose.pose.position.x,
                                      takeoff_pose.pose.position.y,
                                      takeoff_pose.pose.position.z,
                                      uav_cur_pose.pose.position.x,
                                      uav_cur_pose.pose.position.y,
                                      uav_cur_pose.pose.position.z);
                }

                if (stable_count > ctrl_rate * 2)
                {
                    std_msgs::Bool is_stable;
                    is_stable.data = true;
                    is_stable_pub.publish(is_stable);
                    stable_count = 0;
                }
            }
            else if (execute_flag == 1 or execute_flag == 4)
            {   
                ROS_INFO_STREAM_ONCE("\033[33m TRAJ! \033[0m");
                hover_pose = uav_cur_pose;
            }
            else
            {
                // hover
                ROS_INFO_STREAM_ONCE("\033[33m HOVERING! \033[0m");
                offboard::PosVelAcc pva_hover;
                pose2pva(hover_pose, pva_hover);
                offb_setpva_pub.publish(pva_hover);
            }
        }
        else
        { // If not arm
            ROS_INFO_STREAM_THROTTLE(1, " Waiting for Vehicle arm and Offboard cmd");
            offboard::PosVelAcc pva_curr;
            pose2pva(uav_cur_pose, pva_curr);
            offb_setpva_pub.publish(pva_curr);
            takeoff = false;
        }
        ctrl_loop.sleep();
    }

    return 0;
}
