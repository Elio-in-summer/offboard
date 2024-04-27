#include "controller/commonUsage.h"
#include "controller/pid.h"
#include "controller/rosRelated.h"
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <PosVelAcc.h>
#include <DebugPid.h>
#include "Eigen/Eigen"
#include <ctime>

#include "controller/FILTER.h"

ros::Subscriber PID_sub;
double model_error;
double model_error_t;
FILTER model_error_filter(20);
ros::Publisher model_error_filtered_pub;
ros::Publisher model_error_pub;

void pid_callback(const offboard::DebugPid::ConstPtr& msg)
{
    model_error_t = msg->model_error.data;
    model_error = model_error_filter.filter(model_error_t);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_test");
    ros::NodeHandle nh;
    PID_sub = nh.subscribe<offboard::DebugPid>("controller/pid", 1, pid_callback);
    model_error_filtered_pub = nh.advertise<std_msgs::Float64>("/model_error_filtered", 1);
    model_error_pub = nh.advertise<std_msgs::Float64>("/model_error", 1);
    ros::Rate rate(100);
    while (ros::ok())
    {   
        ros::spinOnce();
        std_msgs::Float64 model_error_filtered;
        model_error_filtered.data = model_error;
        model_error_filtered_pub.publish(model_error_filtered);
        std_msgs::Float64 model_error_msg;
        model_error_msg.data = model_error_t;
        model_error_pub.publish(model_error_msg);
        rate.sleep();
    }
    ros::spin();
    return 0;
}