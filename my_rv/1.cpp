

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include<sstream>


int main(int argc, char* argv[])
{
    cv::VideoCapture cap;   //声明相机捕获对象
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')); 
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); //图像的宽，需要相机支持此宽
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); //图像的高，需要相机支持此高
    //图像分辨率640×480
    int deviceID = 0; //相机设备号
    cap.open(deviceID); //打开相机
    if (!cap.isOpened()) //判断相机是否打开
    {
        std::cerr << "ERROR!!Unable to open camera\n";
        return -1;
    }
    cv::Mat img;
    while (true)
    {
        cap >> img; //以流形式捕获图像

        cv::namedWindow("example", 1); //创建一个窗口用于显示图像，1代表窗口适应图像的分辨率进行拉伸。
        if (img.empty() == false) //图像不为空则显示图像
        {
            cv::imshow("example", img);
        }
        
        int  key = cv::waitKey(30); //等待30ms
        {
            break;
        }

    }
    cap.release(); //释放相机捕获对象
    cv::destroyAllWindows(); //关闭所有窗口
        
    return 0;

}

