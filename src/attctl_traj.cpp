/**
 * @file attctl_traj.cpp
 * @brief px4 offboard node for reading from csv file and continuously publish target position
 */


// #define OFFLINE_PREVIEW
#define ONLINE_PREVIEW

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

std::vector<geometry_msgs::PoseStamped> uav_pose_preview_vec;
std::vector<geometry_msgs::PoseStamped> uav_pose_curr_vec;
geometry_msgs::PoseStamped pose_preview; // 发布的目标姿态
geometry_msgs::PoseStamped pose_curr;    // 发布的当前姿态
float hover_height = 1.0;
float tf_scale = 1.0;
float tf_theta = 0;
float tf_x = 0.0;
float tf_y = 0.0;
float tf_z = 0.0;

int ctrl_rate = 25;
std::string traj_csv_name = "/cfg/circle_r1m.csv";
bool repeat_path = false;

ros::Publisher traj_curr_pose_pub;
ros::Publisher traj_curr_path_pub;

/**************************** Function Declaration and Definition *******************************/
// Declaration Function
void path_visual(std::vector<geometry_msgs::PoseStamped> &pose_msg_vec, geometry_msgs::PoseStamped &pose_msg, ros::Publisher &pub);

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

std::vector<std::vector<double>> read_csv_traj(const std::string &file, double tf_scale, double tf_theta, Eigen::Vector3d tf_xyz)
{
    std::vector<std::vector<double>> csv_traj;

    // 对轨迹缩放，旋转和平移
    Eigen::Matrix4d AffineMatrix;
    AffineMatrix << tf_scale * cos(tf_theta), -tf_scale * sin(tf_theta), 0, tf_xyz[0],
        tf_scale * sin(tf_theta), tf_scale * cos(tf_theta), 1, tf_xyz[1],
        0, 0, 1, tf_xyz[2],
        0, 0, 1, 1;

    // 读取文件
    boost::filesystem::path waypoint_file = file;
    if (boost::filesystem::exists(waypoint_file))
    {
        std::ifstream File(waypoint_file.c_str());
        if (File.is_open())
        {
            double px, py, pz, vx, vy, vz, ax, ay, az;
            char eater; // eats commas
            while (File >> px >> eater >> py >> eater >> pz >> eater >> vx >> eater >> vy >> eater >> vz >> eater >> ax >> eater >> ay >> eater >> az)
            {
                Eigen::Matrix<double, 4, 1> traj_point_eigen;
                std::vector<double> traj_point(9);
                traj_point[0] = px;
                traj_point[1] = py;
                traj_point[2] = pz;
                traj_point[3] = vx;
                traj_point[4] = vy;
                traj_point[5] = vz;
                traj_point[6] = ax;
                traj_point[7] = ay;
                traj_point[8] = az;
                // 对位置,速度，加速度仿射变换
                traj_point_eigen << traj_point[0], traj_point[1], traj_point[2], 1.0;
                Eigen::Vector4d tf_traj_point = AffineMatrix * traj_point_eigen;
                traj_point[0] = tf_traj_point[0];
                traj_point[1] = tf_traj_point[1];
                traj_point[2] = tf_traj_point[2];
                // 速度
                // 加速度
                csv_traj.push_back(traj_point);
                if (File.eof())
                {
                    break;
                }
            }
            if (csv_traj.size() > 0)
            {
                ROS_INFO_STREAM("Successfully load trajectory, waypoints number are " << csv_traj.size());
            }
            else
            {
                ROS_ERROR_STREAM("" << waypoint_file.c_str() << " file does not have any waypoint. ");
            }
        }
        else
        {
            ROS_ERROR_STREAM("" << waypoint_file.c_str() << " file could not be opened");
        }
    }
    else
    {
        ROS_ERROR_STREAM("the project folder does not have a waypoints.txt file :" << waypoint_file.c_str());
    }

    return csv_traj;
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
    readParam<std::string>(nh, "traj_csv_name", traj_csv_name);
    readParam<bool>(nh, "repeat_path", repeat_path);
    readParam<float>(nh, "hover_height", hover_height);
    readParam<float>(nh, "tf_scale", tf_scale);
    readParam<float>(nh, "tf_theta", tf_theta);
    readParam<float>(nh, "tf_x", tf_x);
    readParam<float>(nh, "tf_y", tf_y);
    readParam<float>(nh, "tf_z", tf_z);
}

// callback function
void uav_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_cur_state = *msg;
}

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_cur_pose = *msg;
    // 实时查看当前飞机位姿与轨迹
    pose_curr = uav_cur_pose;
    pose_curr.header.frame_id = "global";
    pose_curr.header.stamp = ros::Time::now();
    traj_curr_pose_pub.publish(pose_curr);                         // pub pose
    path_visual(uav_pose_curr_vec, pose_curr, traj_curr_path_pub); // pub path
}

void path_visual(std::vector<geometry_msgs::PoseStamped> &pose_msg_vec, geometry_msgs::PoseStamped &pose_msg, ros::Publisher &pub)
{
    // 填充消息
    pose_msg_vec.emplace_back(pose_msg);

    nav_msgs::Path path_msg;
    path_msg.header = pose_msg.header;
    for (auto &i : pose_msg_vec)
    {
        path_msg.poses.push_back(i);
    }
    // 发布数据
    pub.publish(path_msg);
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "offboard_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // load param
    loadRosParams(nh);
    std::string pkg_path = ros::package::getPath("offboard");
    std::vector<std::vector<double>> csv_traj = read_csv_traj(pkg_path + traj_csv_name, tf_scale, tf_theta, Eigen::Vector3d(tf_x, tf_y, tf_z));
    // ros pub and sub
    ros::Subscriber uav_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, uav_state_cb);
    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 2, uav_pose_cb);
    ros::Publisher offb_setpva_pub = nh.advertise<offboard::PosVelAcc>("/setpoint_pva", 10);

    ros::Publisher traj_offline_preview_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_preview/offline/pose", 10);
    ros::Publisher traj_offline_preview_path_pub = nh.advertise<nav_msgs::Path>("/traj_preview/offline/path", 10);
    ros::Publisher traj_online_preview_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_preview/online/pose", 10);
    ros::Publisher traj_online_preview_path_pub = nh.advertise<nav_msgs::Path>("/traj_preview/online/path", 10);

    traj_curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_curr/online/pose", 10);
    traj_curr_path_pub = nh.advertise<nav_msgs::Path>("/traj_curr/online/path", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate ctrl_loop(ctrl_rate);

    int traj_index = 0;

#ifdef OFFLINE_PREVIEW
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_ONCE("OFFLINE_PREVIEW, please open RVIZ and monitor " / traj_preview / offline / path "");
        if (traj_index + 1 < csv_traj.size())
        {
            geometry_msgs::PoseStamped traj_pose;
            traj_pose.pose.position.x = csv_traj[traj_index][0];
            traj_pose.pose.position.y = csv_traj[traj_index][1];
            traj_pose.pose.position.z = csv_traj[traj_index][2];
            traj_pose.header.frame_id = "global";
            traj_pose.header.stamp = ros::Time::now();
            traj_offline_preview_pose_pub.publish(traj_pose);                            // pub pose
            path_visual(uav_pose_preview_vec, traj_pose, traj_offline_preview_path_pub); // pub path

            traj_index++;
        }
        else
        {
            if (repeat_path)
            {
                ROS_INFO_ONCE("Traj Repeat !!!");
                traj_index = 0; // 从头开始
            }
            else
            {
                ROS_INFO_THROTTLE(1, "OFFLINE_PREVIEW Closed");
            }
        }
        ctrl_loop.sleep();
    }
#endif

#ifdef ONLINE_PREVIEW
    ROS_INFO_ONCE("ONLINE_PREVIEW");

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

    bool takeoff = false;  // Record whether the uav has taken off
    bool just_takeoff = true; // check whether plane change into offboard mode from other mode

    geometry_msgs::PoseStamped pose_offb_hover; // pose to hover in offboard mode
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

                ROS_INFO_ONCE("In Offboard mode, start trajectory moving !");
                if (traj_index + 1 < csv_traj.size())
                {
                    offboard::PosVelAcc pva_traj;
                    pva_traj.header.frame_id = "global";
                    pva_traj.header.stamp = ros::Time::now();
                    pva_traj.px = csv_traj[traj_index][0] + takeoff_pose.pose.position.x;
                    pva_traj.py = csv_traj[traj_index][1] + takeoff_pose.pose.position.y;
                    pva_traj.pz = csv_traj[traj_index][2] + hover_height;
                    pva_traj.vx = csv_traj[traj_index][3];
                    pva_traj.vy = csv_traj[traj_index][4];
                    pva_traj.vz = csv_traj[traj_index][5];
                    pva_traj.ax = csv_traj[traj_index][6];
                    pva_traj.ay = csv_traj[traj_index][7];
                    pva_traj.az = csv_traj[traj_index][8];
                    offb_setpva_pub.publish(pva_traj);

                    geometry_msgs::PoseStamped traj_pose;
                    traj_pose.pose.position.x = csv_traj[traj_index][0] + takeoff_pose.pose.position.x;
                    traj_pose.pose.position.y = csv_traj[traj_index][1] + takeoff_pose.pose.position.y;
                    traj_pose.pose.position.z = csv_traj[traj_index][2] + hover_height;
                    traj_pose.header.frame_id = "global";
                    traj_pose.header.stamp = ros::Time::now();
                    pose_preview = traj_pose;
                    pose_offb_hover = uav_cur_pose;
                    traj_index++;
                    ROS_INFO("[trajectory] xyz: %.3f, %.3f, %.3f", traj_pose.pose.position.x, traj_pose.pose.position.y, traj_pose.pose.position.z);
                }
                else
                {
                    if (repeat_path)
                    {
                        ROS_WARN("=============== Repeat the trajectory ===============");
                        traj_index = 0; // 从头开始 pva模式不要重复路径
                    }
                    else
                    {
                        ROS_WARN_THROTTLE(1, "=============== Trajectory finished, Hover ... ===============");
                        pose2pva(pose_offb_hover, pva_offb_hover);
                        offb_setpva_pub.publish(pva_offb_hover);
                        pose_preview = pose_offb_hover;
                    }
                }
            }
            else
            { // If not offb
                ROS_INFO_STREAM_THROTTLE(1, " waiting for Offboard cmd");
                offboard::PosVelAcc pva_curr;
                pose2pva(uav_cur_pose, pva_curr);
                offb_setpva_pub.publish(pva_curr);
                pose_preview = uav_cur_pose;
                just_takeoff = true;
            }
        }
        else
        { // If not arm
            ROS_INFO_STREAM_THROTTLE(1, " waiting for Vehicle arm");
            offboard::PosVelAcc pva_curr;
            pose2pva(uav_cur_pose, pva_curr);
            offb_setpva_pub.publish(pva_curr);
            pose_preview = uav_cur_pose;
            takeoff = false;
            just_takeoff = true;
        }

        // 实时预览当前发布的目标位姿与轨迹
        pose_preview.header.frame_id = "global";
        pose_preview.header.stamp = ros::Time::now();
        traj_online_preview_pose_pub.publish(pose_preview);                            // pub pose
        path_visual(uav_pose_preview_vec, pose_preview, traj_online_preview_path_pub); // pub path

        ctrl_loop.sleep();
    }
#endif

    return 0;
}
