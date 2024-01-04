//
// Created by zm on 19-3-11.
//

#include "FILTER.h"
#include <time.h>
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
#include <list>


FILTER::FILTER(int filterlength){
    filter_list.push_back(std::make_pair(0.0f, 0.0f));
    filter_data = 0;
    Output_filter = 0;
    start_filter_flag = false;
    filter_length=filterlength;
    derivation_legngth=filterlength;

}

FILTER::FILTER(int sg_listlen,int sg_winlen, int sg_order)//sg_listlen and sg_winlen must be odd and sg_oder must be less than sg_winlen
{
    this->sg_order=sg_order;
    this->sg_window=sg_winlen;
    this->sg_listlength=sg_listlen;
    //Find leading singleton dimensions
    auto Fd = static_cast<double>(sg_window);        //sets the frame size for the savgol differentiation coefficients. This must be odd
    this->B_matrix = sgdiff(sg_order, sg_window, Fd);       //retrieve matrix B
}
float FILTER::satfunc(float data, float Max, float Thres)
{
    if (fabs(data)<Thres)
        return 0;
    else if(fabs(data)>Max){
        return (data>0)?Max:-Max;
    }
    else{
        return data;
    }
}

bool FILTER::filter_input(float data2fliter, float curtime)
{
    if(filter_list.size() == 1)
    {
        delta_time = curtime;
    }
    else{
        delta_time = curtime - filter_list.rbegin()->first;
//        std::cout<<delta_time<<endl;
    }

    filter_data = data2fliter;
    if(filter_list.size() < filter_length){
        filter_list.push_back(std::make_pair(curtime, filter_data));
    }
    else{
        std::vector<std::pair<float, float > > ::iterator fil_iter = filter_list.begin();
        filter_list.erase(fil_iter);
        std::pair<float, float > temp_iter(curtime, filter_data);
        filter_list.push_back(temp_iter);
    }
    return true;
}

void FILTER::filter_output()
{
    if(filter_list.size() < filter_length || ! start_filter_flag){
        Output_filter = 0;
    }
    else{
        std::vector<std::pair<float, float> >::iterator filter_k;
        float filter_sum = 0;
        for(filter_k = filter_list.begin(); filter_k != filter_list.end(); ++ filter_k){
            filter_sum = filter_sum + filter_k->second;
        }
        Output_filter = filter_sum * delta_time/(filter_list.back().first - filter_list.front().first);

    }
}

/**
 * filter by average a specific length
 * @param data to be filted
 */
double FILTER::filter(double data)
{
    double tempsum=0.0;
    if(filterlist.empty())
    {
        filterlist.push_back(data);
        return data;
    } else
    {
        filterlist.push_back(data);
        if(filterlist.size()>filter_length)
        {
            filterlist.pop_front();
        }
        std::list<double > ::iterator itor=filterlist.begin();
        while(itor!=filterlist.end())
        {
//            cout<<"*itor~~~~~~----:"<<*itor<<endl;
            tempsum=tempsum+(*itor++);
        }
        return tempsum/filterlist.size();
    }
}

double FILTER::derivation(double data2derivation, double curtime)
{
    if(derivation_list.size() < derivation_legngth)
    {
        derivation_list.push_back(std::make_pair(curtime, data2derivation));
        if(derivation_list.size()>=derivation_legngth)
        {
            return (derivation_list.back().second-derivation_list.front().second)/(derivation_list.back().first-derivation_list.front().first);
        }
        return 0.0;
    }
    else{
        std::vector<std::pair<double, double> > ::iterator derivation_iter = derivation_list.begin();
        derivation_list.erase(derivation_iter);
        derivation_list.push_back(std::make_pair(curtime, data2derivation));
        //    std::cout<<"(derivation_list.back().second-derivation_list.front().second): "<<(derivation_list.back().second-derivation_list.front().second)
        //    <<"  (derivation_list.back().first-derivation_list.front().first): "<<(derivation_list.back().first-derivation_list.front().first)<< std::endl;
        return (derivation_list.back().second-derivation_list.front().second)/(derivation_list.back().first-derivation_list.front().first);
    }
}

/*Compute the polynomial basis vectors s_0, s_1, s_2 ... s_n using the vandermonde matrix.*/
MatrixXi FILTER::vander(const int F)
{
    auto v = VectorXi::LinSpaced(F,(-(F-1)/2),((F-1)/2)).transpose().eval();

    MatrixXi A(F, F+1);     //We basically compute an F X F+1 matrix;

    for(auto i = 0; i < F; ++ i)
    {
        for(auto j=1; j < F+1; ++j)
        {
            A(i,j) = pow(v(i), (j-1) );
        }
    }

    A = A.block(0, 1, F, F );   //and retrieve the right F X F matrix block, excluding the first column block to find the vandermonde matrix.

    return A;
}

/*brief Compute the S-Golay Matrix of differentiators
*
*/
MatrixXf FILTER::sgdiff(int k, int F, double Fd)
{
    //We set the weighting matrix to an identity matrix if no weighting matrix is supplied
    auto W = MatrixXf::Identity(Fd, Fd);

    //Compute Projection Matrix B
    auto s = vander(F);

    //Retrieve the rank deficient matrix from the projection matrix
    auto S = s.block(0, 0, s.rows(), (k+1) ) ;

    //Compute sqrt(W)*S
    auto Sd = S.cast<float> ();    //cast S to float
    auto inter = W * Sd;              //W is assumed to be identity. Change this if you have reasons to.

    //Compute the QR Decomposition
    HouseholderQR<MatrixXf> qr(inter);
    qr.compute(inter);

    FullPivLU<MatrixXf>lu_decomp(inter);      //retrieve rank of matrix

    auto Rank = lu_decomp.rank() ;

    //For rank deficient matrices. The S matrix block will always be rank deficient.
    // MatrixXf Q = qr.householderQ();  //unsused
    MatrixXf R = qr.matrixQR().topLeftCorner(Rank, Rank).template triangularView<Upper>();

    //Compute Matrix of Differentiators
    auto Rinv = R.inverse();
    MatrixXf RinvT = Rinv.transpose();

    MatrixXf G = Sd * Rinv * RinvT;           /*G = S(S'S)^(-1)   -- eqn 8.3.90 (matrix of differentiation filters)*/

    MatrixXf SdT = Sd.transpose().eval();

    MatrixXf B = G * SdT * W;   //SG-Smoothing filters of length F and polynomial order k

    return B;
}

// lss reference:https://blog.csdn.net/ASDF1712/article/details/82383537 and https://www.cnblogs.com/zwjun/p/13745059.html
RowVectorXf FILTER::savgolfilt(VectorXf  x, int k, int F)
{
    /*Transient On*/
    auto id_size = (F+1)/2 - 1;
    auto Bon=B_matrix.topLeftCorner(id_size,B_matrix.cols());
    VectorXf y_on = Bon * x.segment(0,F);  //Now compute the transient on

/***********************************/
    /*Compute the steady state output*/
    // size_t idzeroth = floor(B.cols()/2);
    // auto Bzeroth = B.col(idzeroth);
    // auto Bzerothf = Bzeroth.cast<float>();
    // auto y_ss = Bzerothf.transpose().eval() * x;     //This is the steady-state smoothed value. Here, we should use coveloution--lss.but there is no cov function can be direcetly exploited
/***********************************/

    auto Boff = B_matrix.bottomLeftCorner(id_size, B_matrix.cols());
    auto y_off = Boff * x.segment(x.rows()-F-1,F);   //This is the transient off

    /*Make Y into the shape of X and retuen the smoothed values!*/
    RowVectorXf y(F-1);
    // y << y_off.transpose().eval(), y_ss, y_on.transpose().eval();
    y <<y_on.transpose().eval(), y_off.transpose().eval();

    // return y;

    for(int i=0;i<x.rows()-F+1;i++)
    {
        // cout << "\n auto x.segment(i,F): \n" << x.segment(i,F) << endl;
        // cout << "\n B * x.segment(i,F): \n" << B * x.segment(i,F) << endl;
        x.segment(i,F)= B_matrix * x.segment(i,F);
    }
    return x;
}

double FILTER::sgfilter(double data)
{
    sglist.push_back(data);
    if(sglist.size()>sg_listlength)
    {
        VectorXf data_vec(sg_listlength);
        std::vector<double> ::iterator sglist_iter = sglist.begin();
        sglist.erase(sglist_iter);
        for(int i=0;i<sg_listlength;i++)
        {
            data_vec(i)=sglist[i];
        }
        auto Filter=savgolfilt(data_vec, sg_order, sg_window);
        return Filter(Filter.size()-1);
    } else
    {
        if(sglist.size()==sg_listlength)
        {
            VectorXf data_vec(sg_listlength);
            for(int i=0;i<sg_listlength;i++)
            {
                data_vec(i)=sglist[i];
            }
            auto Filter=savgolfilt(data_vec, sg_order, sg_window);
            return Filter(Filter.size()-1);
        }
        return data;
    }
}