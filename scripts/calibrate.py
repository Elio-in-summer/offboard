import rosbag
import numpy as np
import matplotlib.pyplot as plt
# from ipywidgets import interact
# import std_msgs.msg

# 路径替换为你的ROS bag文件路径
# bag_path = '/home/hxl/Downloads/agress_traj_cut.bag'
bag_path = '/home/up/2021.bag'

# 初始化存储数据的列表
thr_norm_data = []
est_a_norm_data = []
K1_data = []
timestamps = []

# 从ROS bag中提取数据

with rosbag.Bag(bag_path) as bag:
    # print(bag)
    for topic, msg, t in bag.read_messages(topics=['/controller/pid']):
        thr_norm_data.append(msg.thr_norm.data)
        est_a_norm_data.append(msg.est_a_norm.data)
        K1_data.append(msg.base_thrust.data)
        timestamps.append(t.to_sec())
print(len(thr_norm_data))
print(len(est_a_norm_data))
print(len(K1_data))
print(len(timestamps))

thr_norm_data = np.array(thr_norm_data)
est_a_norm_data = np.array(est_a_norm_data)
K1_data = np.array(K1_data)
timestamps = np.array(timestamps)
# stack thr_norm_data and est_a_norm_data and timestamps
data = np.column_stack((timestamps, thr_norm_data, est_a_norm_data, K1_data))
print(data.shape)

# delete those rows which thr_norm_data do not changed between two timestamps
data = data[np.where(np.diff(data[:, 1]) != 0)]
print(data.shape)
timestamps = data[:, 0]
normalized_time = (timestamps - timestamps[0])/(timestamps[-1] - timestamps[0])


from scipy.optimize import curve_fit

# 假设你已经有了thr和acc数据
thr = data[:, 1]
acc = data[:, 2]
K1 = data[:, 3]
acc_dive_K1 = acc/K1

# 定义模型函数
def model1(thr, K2):
    return  (K2 * thr**2 + (1 - K2) * thr)
def model2(thr, K):
    return K * thr

# 使用curve_fit来估计参数
popt1, pcov1 = curve_fit(model1, thr, acc_dive_K1)
popt2, pcov2 = curve_fit(model2, thr, acc)

print("pcov1: ", pcov1)
print("pcov2: ", pcov2)
print("popt1: ", popt1)
print("popt2: ", popt2)
# 生成预测曲线的推力值
thr_pred = np.linspace(np.min(thr), np.max(thr), 500)
# 计算预测曲线的加速度值
acc_pred1 = model1(thr_pred, *popt1)
acc_pred2 = model2(thr_pred, *popt2)

# 绘制原始数据的散点图
plt.scatter(thr, acc_dive_K1, label='Original Data')

# 绘制模型预测的曲线
plt.plot(thr_pred, acc_pred1, color='red', label='Fitted Model')
# plt.plot(thr_pred, acc_pred2, color='green', label='Fitted Model')

plt.xlabel('Thrust')
plt.ylabel('Acceleration_dive_K1')
plt.title('Thrust vs. Acceleration with Fitted Model')
plt.legend()
plt.show()