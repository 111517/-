#include"commen.h"

//为了防止有其他装甲板干扰，使得算法可以追踪，加入限幅滤波
filter_type filter(filter_type effective_value, filter_type new_value, filter_type delat_max)
{
    if ( ( new_value - effective_value > delat_max ) || ( effective_value - new_value > delat_max ))
    {
        new_value=effective_value;
        return effective_value;
    }
    else
    {
        new_value=effective_value;
        return new_value;
    }
}
// 为辅助筛选装甲板，提高算法运行速度，做一次筛选预处理
RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
   {
       using std::swap;

       float& width = rec.size.width;
       float& height = rec.size.height;
       float& angle = rec.angle;

       if (mode == WIDTH_GREATER_THAN_HEIGHT)
       {
           if (width < height)
           {
               swap(width, height);
               angle += 90.0;
           }
       }

       while (angle >= 90.0) angle -= 180.0;
       while (angle < -90.0) angle += 180.0;

       if (mode == ANGLE_TO_UP)
       {
           if (angle >= 45.0)
           {
               swap(width, height);
               angle -= 90.0;
           }
           else if (angle < -45.0)
           {
               swap(width, height);
               angle += 90.0;
           }
       }
   return rec;
}//筛去竖着的轮廓
//输出ROI
void ROIimg(Mat imgOriginal, int x, int y)
{
    //选取ROI
        //创造一个空白图框
        Mat backGround = Mat::zeros(imgOriginal.size(),CV_8UC3);
        //设置选取位置
        vector<Rect> rectROI;
        Rect rect1(x,y, 100, 60);
        //截取
        Mat ROI1 = imgOriginal(rect1);
        rectROI.push_back(rect1);
        for(int i = 0; i < rectROI.size(); i++)
        {
            imgOriginal(rectROI[i]).copyTo(backGround(rectROI[i]));
        }
        //展示ROI
        imshow("ROI", backGround);
}

void calAngle(Mat cam,Mat dis,int x,int y)
{
    double fx=cam.at<double>(0,0);
    double fy=cam.at<double>(1,1);
    double cx=cam.at<double>(0,2);
    double cy=cam.at<double>(1,2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(Point2f(x,y));
    //对像素点去畸变
    undistortPoints(in,out,cam,dis,noArray(),cam);
    pnt=out.front();
    //没有去畸变时的比值
    double rx=(x-cx)/fx;
    double ry=(y-cy)/fy;
    //去畸变后的比值
    double rxNew=(pnt.x-cx)/fx;
    double ryNew=(pnt.y-cy)/fy;
    //输出原来点的坐标和去畸变后点的坐标
    cout<< "x: "<<x<<" xNew:"<<pnt.x<<endl;
    cout<< "y: "<<y<<" yNew:"<<pnt.y<<endl;
    //输出未去畸变时测得的角度和去畸变后测得的角度
    cout<< "angx: "<<atan(rx)/CV_PI*180<<" angleNew:"<<atan(rxNew)/CV_PI*180<<endl;
    cout<< "angy: "<<atan(ry)/CV_PI*180<<" angleNew:"<<atan(ryNew)/CV_PI*180<<endl;
}

void distance(Mat cam,Mat dis,vector<Point2d>pnts)
{
#define HALF_LENGTH 67.5
#define HALF_WIDTH 28.1
//自定义的物体世界坐标，单位为mm
vector<Point3f> obj=vector<Point3f>{
    cv::Point3f(-HALF_LENGTH, -HALF_WIDTH, 0),	//tl
    cv::Point3f(HALF_LENGTH, -HALF_WIDTH ,0),	//tr
    cv::Point3f(HALF_LENGTH, HALF_WIDTH, 0),	//br
    cv::Point3f(-HALF_LENGTH, HALF_WIDTH, 0)	//bl
};
cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec
//进行位置解算
solvePnP(obj,pnts,cam,dis,rVec,tVec,false,SOLVEPNP_ITERATIVE);
//输出平移向量
//pnts.clear()
cout <<"tvec: "<<tVec<<endl;
}

