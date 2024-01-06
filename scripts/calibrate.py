import rosbag
import numpy as np
import matplotlib.pyplot as plt
# import std_msgs.msg

# 路径替换为你的ROS bag文件路径
bag_path = '/home/hxl/Documents/model_cut.bag'

# 初始化存储数据的列表
thr_norm_data = []
est_a_norm_data = []
timestamps = []

# 从ROS bag中提取数据

with rosbag.Bag(bag_path) as bag:
    # print(bag)
    for topic, msg, t in bag.read_messages(topics=['/controller/pid']):
        thr_norm_data.append(msg.thr_norm.data)
        est_a_norm_data.append(msg.est_a_norm.data)
        timestamps.append(t.to_sec())
print(len(thr_norm_data))
print(len(est_a_norm_data))
print(len(timestamps))

thr_norm_data = np.array(thr_norm_data)
est_a_norm_data = np.array(est_a_norm_data)
timestamps = np.array(timestamps)
# stack thr_norm_data and est_a_norm_data and timestamps
data = np.column_stack((timestamps, thr_norm_data, est_a_norm_data))
print(data.shape)

# delete those rows which thr_norm_data do not changed between two timestamps
data = data[np.where(np.diff(data[:, 1]) != 0)]
print(data.shape)
timestamps = data[:, 0]
normalized_time = (timestamps - timestamps[0])/(timestamps[-1] - timestamps[0])



fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

# 第一个子图
ax1.plot(normalized_time, 15 * data[:, 1], label='thr_norm_data')
ax1.plot(normalized_time, data[:, 2], label='est_a_norm_data')
line, = ax1.plot([0, 0], [0, 16], color="k", linestyle="--")  # 初始垂直线
ax1.set_xlabel('Normalized Time')
ax1.set_ylabel('Data')
ax1.legend()
ax1.grid(True)

# 第二个子图
sc = ax2.scatter(data[:, 1], data[:, 2], c=normalized_time, cmap='coolwarm', alpha=0.5)
ax2.set_xlabel('thr_norm_data')
ax2.set_ylabel('est_a_norm_data')
ax2.legend(['thr_norm_data vs est_a_norm_data'])
ax2.grid(True)

def onclick(event):
    if event.inaxes == ax2:
        # 计算点击位置与所有点的距离
        dist = np.linalg.norm(data[:, 1:3] - [event.xdata, event.ydata], axis=1)
        # 找到最近点的索引
        index = np.argmin(dist)
        # 找到对应的时间点
        time_point = normalized_time[index]
        # 更新第一个子图的垂直线
        line.set_xdata([time_point, time_point])
        fig.canvas.draw()

# 绑定点击事件
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()