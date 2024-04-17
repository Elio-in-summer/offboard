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
    bool ready_to_land = false;
    int stable_count = 0;

    offboard::PosVelAcc pva_offb_hover;
    pose2pva(uav_cur_pose, pva_offb_hover);

    /**************************** main ctrl loop *******************************/
    while (ros::ok())
    {
        ros::spinOnce();
        if (uav_cur_state.armed && uav_cur_state.mode == "OFFBOARD")
        {   
            // ! execute_flag = 0: Before traj tracking, the uav need to take off to the initial height and hover
            if (execute_flag == 0)
            {   
                // std::cout << "takeoff_pose_z: " << takeoff_pose.pose.position.z << std::endl;
                if (!takeoff)
                {
                    offboard::PosVelAcc pva_climb;
                    geometry_msgs::PoseStamped climb_pose; // 预计爬升的高度
                    climb_pose.pose.position.z = uav_cur_pose.pose.position.z;
                    // ! Take off until it reaches the target height
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

                // ! After taking off to approximately the initial height, adjust to hover state
                offboard::PosVelAcc pva_hover;
                pose2pva(takeoff_pose, pva_hover);
                offb_setpva_pub.publish(pva_hover);

                // calculate the distance between current pose and goal pose
                double distance = sqrt(
                    pow(uav_cur_pose.pose.position.x - takeoff_pose.pose.position.x, 2) +
                    pow(uav_cur_pose.pose.position.y - takeoff_pose.pose.position.y, 2) +
                    pow(uav_cur_pose.pose.position.z - takeoff_pose.pose.position.z, 2));

                // ! if the hover state is stable for 2s, then publish the is_stable flag
                if (distance < 0.1) // TODO: 调整阈值
                {
                    stable_count++;
                    std::cout << "stable_count: " << stable_count << std::endl;
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
                    execute_flag = 9;
                    stable_count = 0;
                }

                // ! hover pose is updated to the current pose
                hover_pose = uav_cur_pose;
            }
            else
            {   
                std::cout << "hover_pose_z: " << hover_pose.pose.position.z << std::endl;
                // ! After the trajectory tracking, the uav will hover
                if (!ready_to_land){
                    ROS_INFO_STREAM_ONCE("\033[33m HOVERING Before Landing! \033[0m");
                    offboard::PosVelAcc pva_hover;
                    pose2pva(hover_pose, pva_hover);
                    offb_setpva_pub.publish(pva_hover);

                    double distance = sqrt(
                        pow(uav_cur_pose.pose.position.x - hover_pose.pose.position.x, 2) +
                        pow(uav_cur_pose.pose.position.y - hover_pose.pose.position.y, 2) +
                        pow(uav_cur_pose.pose.position.z - hover_pose.pose.position.z, 2));
                    
                    if (distance < 0.1)
                    {
                        stable_count++;
                        std::cout << "stable_count: " << stable_count << std::endl;
                    }
                    else
                    {
                        stable_count = 0;
                        ROS_INFO_THROTTLE(1, "[HOVER]: Target: %.2f, %.2f, %.2f | Cur: %.2f, %.2f, %.2f",
                                        hover_pose.pose.position.x,
                                        hover_pose.pose.position.y,
                                        hover_pose.pose.position.z,
                                        uav_cur_pose.pose.position.x,
                                        uav_cur_pose.pose.position.y,
                                        uav_cur_pose.pose.position.z);
                    }

                    if(stable_count > ctrl_rate * 2)
                    {
                        ready_to_land = true;
                        stable_count = 0;
                    }
                }
                else
                {
                    // ! After hovering for 2s, the uav will land
                    offboard::PosVelAcc pva_land;
                    geometry_msgs::PoseStamped land_pose;
                    land_pose.pose.position.z = uav_cur_pose.pose.position.z;
                    land_pose.pose.position.x = uav_cur_pose.pose.position.x;
                    land_pose.pose.position.y = uav_cur_pose.pose.position.y;
                    int stop_count = 0;                    
                    while (uav_cur_pose.pose.position.z > 0.2)
                    {
                        ros::spinOnce();
                        // TODO:下降速度 0.1m/s
                        land_pose.pose.position.z = land_pose.pose.position.z -
                                                     0.1 / double(ctrl_rate);                      // land_pose.pose.position.z = std::max(land_pose.pose.position.z, 0.0);
                        pose2pva(land_pose, pva_land);
                        offb_setpva_pub.publish(pva_land);
                        ROS_INFO_THROTTLE(1, "Landing]: Target: %.2f, %.2f, %.2f | Cur: %.2f, %.2f, %.2f",
                                          land_pose.pose.position.x,
                                          land_pose.pose.position.y,
                                          land_pose.pose.position.z,
                                          uav_cur_pose.pose.position.x,
                                          uav_cur_pose.pose.position.y,
                                          uav_cur_pose.pose.position.z);
                        ctrl_loop.sleep();
                    }
                    // ! Actually, at this time uav have reached the ground, so let it disarmed as soon as possible
                    int temp_count = 0;
                    while (uav_cur_state.armed){
                        if(temp_count < ctrl_rate * 2){
                            land_pose.pose.position.z = land_pose.pose.position.z - 1.0 / double(ctrl_rate);
                            pose2pva(land_pose, pva_land);
                            offb_setpva_pub.publish(pva_land);
                            ROS_INFO_STREAM("\033[33m Land done \033[0m");
                            temp_count++;
                            ctrl_loop.sleep();
                        }
                        else{
                            ROS_INFO_STREAM("\033[33m Stop All \033[0m");
                            offboard::PosVelAcc pva_stop;
                            // ! give an impossible position pz as the stop signal
                            pva_stop.pz = -100.0; 
                            offb_setpva_pub.publish(pva_stop);
                            ctrl_loop.sleep();
                        }
                    }
                }
                
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
