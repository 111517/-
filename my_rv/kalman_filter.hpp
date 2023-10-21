#include"kalman_filter.h"
void::use_kalman_filter::set_Q(float x)
{
	Q = x*Mat::eye(stateNum, stateNum, CV_32F);
}

void::use_kalman_filter::set_R(float y)
{

	R = y*Mat::eye(measureNum, measureNum, CV_32F);

}


void::use_kalman_filter::set_P(float z)
{

	P_predict = z*Mat::eye(stateNum, stateNum, CV_32F);

}

void::use_kalman_filter::get_F(Mat FF)
{
	F = FF.clone();
}


void use_kalman_filter::init()
{
	K = Mat::zeros(stateNum, stateNum, CV_32F);
	H = Mat::zeros(measureNum, stateNum, CV_32F);
	temp = Mat::zeros(stateNum, stateNum, CV_32F);

	for (int i = 0; i < measureNum; i++)
	{
		H.at<float>(i, i) = 1;
	}

}

void::use_kalman_filter::correct()
{
	//公式均为blog中的公式

	//predict
	x_hat_prect = F*x_state;
	P = F*P_predict*F.t() + Q;

	//Update
	temp = H*P*H.t() + R;
	temp = temp.inv();
	K = P*H.t() *temp;

	x_state = x_hat_prect+ K*(z - H *x_hat_prect);     				//预测值
	P_predict = (Mat::eye(stateNum, stateNum, CV_32F) - K*H)*P;		//预测值协方差

}