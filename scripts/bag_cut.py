#!/usr/bin/env python
import rosbag
from std_msgs.msg import Int8
bag_id = '2292'
input_bag_path = '/home/up/exper_data' + bag_id + '.bag'  # 输入bag文件的路径
output_bag_path = '/home/up/exper_data' + bag_id + '_filtered.bag'  # 输出bag文件的路径

def find_time_segment(input_bag_path):
    start_time = None
    end_time = None
    for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
        if topic == "/palnner_execute_flag" and msg.data == 1:
            if start_time is None:
                start_time = t  # 记录第一次满足条件的时间
            end_time = t  # 更新最后一次满足条件的时间
    return start_time, end_time

def filter_bag(input_bag_path, output_bag_path, start_time, end_time):
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            if start_time <= t <= end_time:
                outbag.write(topic, msg, t)

if __name__ == '__main__':
    start_time, end_time = find_time_segment(input_bag_path)
    print("start_time: ", start_time)
    print("end_time: ", end_time)
    if start_time and end_time:
        filter_bag(input_bag_path, output_bag_path, start_time, end_time)
    else:
        print("No segment found where '/my_topic' has data == 1.")
