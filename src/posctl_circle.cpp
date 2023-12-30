/**
 * @file posctl_circle.cpp
 * @brief Offboard control example node in POSITION control mode
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/State.h>
#include <string>
#include "math.h"

/**************************** Global Variable *******************************/
mavros_msgs::State uav_cur_state;
geometry_msgs::PoseStamped uav_cur_pose;
geometry_msgs::PoseStamped takeoff_pose;
geometry_msgs::PoseStamped offb_hover_pose; // pose to hover in offboard mode
geometry_msgs::PoseStamped target_pose;     // pose to fly in offboard mode

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
geometry_msgs::Quaternion euler2quaternion(double roll, double pitch, double yaw)
{
    geometry_msgs::Quaternion q;
    q.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    return q;
}

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
    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 2, uav_pose_cb);
    ros::Publisher offb_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

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
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        offb_setpoint_pub.publish(uav_cur_pose);
        ctrl_loop.sleep();
    }

    // takeoff：Record whether the uav has taken off
    // just_takeoff：check whether the uav change into offboard mode from other mode
    bool takeoff = false;
    bool just_takeoff = true;

    /**************************** main ctrl loop *******************************/
    while (ros::ok())
    {
        ros::spinOnce();
        if (uav_cur_state.armed)
        {
            if (uav_cur_state.mode == "OFFBOARD")
            {
                ROS_INFO_STREAM_THROTTLE(1, "\033[33m OFFBOARD Mode \033[0m");

                // Take off to some height
                if (!takeoff)
                {
                    geometry_msgs::PoseStamped climb_pose;
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
                        offb_setpoint_pub.publish(climb_pose);
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
                    for (int i = 150; ros::ok() && i > 0; --i) // TODO: 先悬停
                    {
                        ros::spinOnce();
                        offb_setpoint_pub.publish(takeoff_pose);
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

                    target_pose.header.stamp = ros::Time::now();
                    double phi = double(count_circle) / double(ctrl_rate) * omege;

                    /**
                     *  Here is the code for testing.
                     *  When running a specific case,
                     *  please comment out the other cases.
                     */

                    /**
                     *  case 1 hovering
                     */
                    // target_pose.pose.position.x = 0 + takeoff_pose.pose.position.x;
                    // target_pose.pose.position.y = 0 + takeoff_pose.pose.position.y;
                    // target_pose.pose.position.z = hover_height;

                    /**
                     *  case 1 circling WITHOUT yaw rotating
                     */
                    target_pose.pose.position.x = r_circle * sin(phi) + takeoff_pose.pose.position.x;
                    target_pose.pose.position.y = r_circle * (1 - cos(phi)) + takeoff_pose.pose.position.y;
                    target_pose.pose.position.z = hover_height;

                    /**
                     *  case 2  yaw rotating WITHOUT moving
                     */
                    // target_pose.pose.position.x = takeoff_pose.pose.position.x;
                    // target_pose.pose.position.y = takeoff_pose.pose.position.y;
                    // target_pose.pose.position.z = hover_height;
                    // target_pose.pose.orientation = euler2quaternion(0, 0, phi);

                    /**
                     *  case 3 circling WITH yaw rotating
                     */
                    // target_pose.pose.position.x = r_circle * sin(phi) + takeoff_pose.pose.position.x;
                    // target_pose.pose.position.y = r_circle * (1 - cos(phi)) + takeoff_pose.pose.position.y;
                    // target_pose.pose.position.z = hover_height;
                    // target_pose.pose.orientation = euler2quaternion(0,0,phi);

                    offb_setpoint_pub.publish(target_pose);

                    offb_hover_pose = uav_cur_pose;
                    count_circle++;
                    ROS_INFO("[trajectory] Target xyz: %.3f, %.3f, %.3f | Curr xyz: %.3f, %.3f, %.3f",
                             target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                             uav_cur_pose.pose.position.x, uav_cur_pose.pose.position.y, uav_cur_pose.pose.position.z);
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
                        offb_setpoint_pub.publish(offb_hover_pose);
                    }
                }
            }
            else
            { // If not offb
                ROS_INFO_STREAM_THROTTLE(1, " waiting for Offboard cmd");
                offb_setpoint_pub.publish(uav_cur_pose);
                just_takeoff = true;
            }
        }
        else
        { // If not arm
            ROS_INFO_STREAM_THROTTLE(1, " waiting for Vehicle arm");
            offb_setpoint_pub.publish(uav_cur_pose);
            takeoff = false;
            just_takeoff = true;
        }

        ctrl_loop.sleep();
    }

    return 0;
}
