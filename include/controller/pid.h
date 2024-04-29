//
// Created by wzy on 2020/12/24.
//

#ifndef TUNNELFLIGHT_PID_H
#define TUNNELFLIGHT_PID_H

#include "commonUsage.h"
using namespace std;
class PID
{

public:
    // 构造函数
    PID();
    double Kp; // 参数P
    double Ki; // 参数I
    double Kd; // 参数D

    double error;       // 误差量 = 实际值 - 期望值
    double delta_time; // 时间间隔dt

    std::vector<std::pair<double, double>> error_list;      // 误差表,用作计算微分项 平滑窗口 [2nd data, 1st time]
    std::vector<std::pair<double, double>> difference_list; // 误差表,用作计算微分项 平滑窗口 [2nd data, 1st time]

    double P_Out;  // P环节输出 Kp*error
    double I_Out;  // I环节输出
    double D_Out;  // D环节输出
    double Output; // 输出

    bool start_intergrate_flag; // 是否积分标志[进入offboard(启控)后,才开始积分] 1 or 0
    double Imax;                 // 积分上限
    double Output_max;           // 输出最大值
    double errThres;             // 误差死区(if error<errThres, error<0)

    // 设置PID参数函数[Kp Ki Kd]
    void setPID(double p_value, double i_value, double d_value);
    // 设置积分上限 控制量最大值 误差死区
    void set_sat(double i_max, double con_max, double thres);
    // 输入 误差 和 当前时间
    bool add_error(double input_error, double curtime);
    void pid_output(void);

    // 饱和函数
    double satfunc(double data, double Max, double Thres);
};

// PID()函数，用于初始化
PID::PID()
{
    error_list.push_back(make_pair(0.0f, 0.0f));
    error = 0;
    P_Out = 0;
    I_Out = 0;
    D_Out = 0;
    Output = 0;
    start_intergrate_flag = false;
}

// 饱和函数 I环节限幅Thres<= I_out<=Max
double PID::satfunc(double data, double Max, double Thres)
{
    if (fabs(data) < Thres)
        return 0;
    else if (fabs(data) > Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

// 设置PID参数函数[Kp Ki Kd]
void PID::setPID(double p_value, double i_value, double d_value)
{
    Kp = p_value;
    Ki = i_value;
    Kd = d_value;
}
// 设置控制量最大值 积分上限 误差死区
void PID::set_sat(double i_max, double con_max, double thres)
{
    Imax = i_max;
    Output_max = con_max;
    errThres = thres;
}

// 输入误差 和 当前时间
bool PID::add_error(double input_error, double curtime)
{
    error = input_error;
    // delta_time = 0.05;
    if (error_list.size() == 1)
    {
        delta_time = 0.05;
    }
    else
    {
        delta_time = curtime - error_list.rbegin()->first; // error_list 逆向队列的第一个元素
    }

    if (error_list.size() < 10)
    {
        error_list.push_back(make_pair(curtime, error)); // errorlist只存放10个pair
    }
    else
    {
        vector<pair<double, double>>::iterator k_beg = error_list.begin(); // 定义一个可以迭代pair类型的迭代器k_beg，指向error_list的首位
        error_list.erase(k_beg);                                         // 删除第k_beg个元素
        std::pair<double, double> p1(curtime, error);                      // 定义pair类型的p1，并用(curtime, error)初始化
        error_list.push_back(p1);                                        // 新增p1
    }

    return true;
}

void PID::pid_output(void)
{   
    delta_time = 0.02;
    P_Out = Kp * error;                      // P环节输出值
    I_Out = I_Out + Ki * error * delta_time; // I环节输出值

    // std::cout << "pid" << Ki << " " << error << " " << delta_time << " " << endl; // TODO TEST DELETE
    I_Out = satfunc(I_Out, Imax, 0);                                              // I环节限幅[I_Out<=Imax]

    if (start_intergrate_flag == 0)
    {
        I_Out = 0;
    }

    D_Out = 0;

    if (error_list.size() < 3 || Kd == 0)
    {
        D_Out = 0; // initiral process
    }
    else
    {
        vector<pair<double, double>>::reverse_iterator error_k = error_list.rbegin(); // 传回一个逆向队列的第一个数据。
        vector<pair<double, double>>::reverse_iterator error_k_1 = error_k + 1;
        D_Out = (error_k->second - error_k_1->second) / delta_time * Kd;
    }

    Output = P_Out + I_Out + D_Out;
    Output = satfunc(Output, Output_max, errThres);
}

#endif // TUNNELFLIGHT_PID_H
