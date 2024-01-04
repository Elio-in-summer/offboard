//
// Created by zm on 19-3-11.
//

#ifndef OFFB_POSCTL_FILTER_H
#define OFFB_POSCTL_FILTER_H

#include <vector>
#include <list>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/QR>

using namespace Eigen;
using namespace std;

class FILTER{

//    float Filter_in;
//    float Filter_out;
//    float aa;

protected:
    MatrixXf B_matrix;
public:
    FILTER(int filterlength);
    FILTER(int sg_listlen,int sg_winlen, int sg_order);
    float filter_data;                 //待滤波的数据
    float Output_filter;               //积分滤波的输出
    float delta_time;                  //时间间隔dt
    bool start_filter_flag;        //是否积分标志[进入offboard(启控)后,才开始积分]
    int filter_length=10;
    int derivation_legngth=3;
    int sg_order=3;
    int sg_window=15;
    int sg_listlength=101;
    std::list<double> filterlist;
    std::vector<double> sglist;

    std::vector <std::pair<float, float> > filter_list; //积分平均表
    std::vector <std::pair<double, double> > derivation_list; //积分平均表

    float satfunc(float data, float Max, float Thres);
    //积分平均滤波器
    bool filter_input(float data2fliter, float curtime);
    void filter_output();
    double filter(double data);
    double derivation(double data2derivation,double curtime);

    Eigen::MatrixXi vander(const int F);
    Eigen::MatrixXf sgdiff(int k, int F, double Fd);
    Eigen::RowVectorXf savgolfilt(Eigen::VectorXf  x, int k, int F);
    double sgfilter(double data);

};

#endif //OFFB_POSCTL_FILTER_H
