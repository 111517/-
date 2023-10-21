//头文件
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<iostream>
using namespace cv;
/// @brief 
class use_kalman_filter
{
public:
	Mat x_state;
	Mat z;						//真实值
	int stateNum;					//状态值个数
	int measureNum;					//测量值个数
	void set_Q(float x);				
	void set_R(float y);			
	void set_P(float z);
	void get_F(Mat FF);
	void init();					//卡尔曼滤波器的初始化

	void correct();					//用于对预测值的更新

private:
	Mat Q;			//系统噪声方差矩阵Q
	Mat R;			//测量噪声方差矩阵R
	Mat F;			//状态转移矩阵，物理方程
	Mat H;          //观测矩阵
	Mat K;			//卡尔曼系数

	Mat P;			//预测的协方差矩阵
	Mat P_predict;  //最终的协方差矩阵
	Mat x_hat_prect;
	Mat temp;
};