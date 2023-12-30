/**
 * @file posctl_hover.cpp
 * @brief Offboard control example node in POSITION control mode
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#define LOOPRATE 25

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(LOOPRATE);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("wait for FCU connection");
    }
    ROS_INFO_STREAM("\033[33m FCU Connected!! \033[0m");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.7;

    geometry_msgs::PoseStamped pose_land;
    pose_land.pose.position.x = 0;
    pose_land.pose.position.y = 0;
    pose_land.pose.position.z = 0;

    /**
     * Pre Step :
     * Send a few setpoints before starting
     * Before entering Offboard mode,
     * you must have already started streaming setpoints.
     * Otherwise the mode switch will be rejected. 
    */
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose_land);
        ros::spinOnce();
        rate.sleep();
    }

    // Flying time setting
    int fly_time = 20;
    int count = 0;
    int count_total = fly_time * LOOPRATE;

    while (ros::ok())
    {
        ros::spinOnce();
        if (current_state.armed){
            if (current_state.mode == "OFFBOARD"){
                ROS_INFO_STREAM_ONCE("\033[33m In OFFBOARD Mode \033[0m");
                if(count < count_total){
                    local_pos_pub.publish(pose);
                    ROS_INFO_STREAM_THROTTLE(1,"\033[33m Flying \033[0m");
                    count++;
                }
                else{
                    local_pos_pub.publish(pose_land);
                    ROS_INFO_STREAM_THROTTLE(1,"\033[33m Landing \033[0m");
                }
                
            }
            else{
                ROS_INFO_STREAM_ONCE(" waiting for Offboard cmd");
                local_pos_pub.publish(pose_land);
            }
        }
        else{
            ROS_INFO_STREAM_ONCE(" waiting for Vehicle arm");
            local_pos_pub.publish(pose_land);
        }

        ros::spinOnce(); // TODO
        rate.sleep();
    }

    return 0;
}
