#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>             // 将ROS下的sensor_msgs/Image消息类型转化为cv::Mat数据类型
#include <sensor_msgs/image_encodings.h>     // ROS下对图像进行处理
#include <image_transport/image_transport.h> // 用来发布和订阅图像信息

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;

Mat mask, dst1, dst2, dst3, src, src1, src2; //设置二维点集变

Mat result;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imageGet_node");                                 // ros初始化，定义节点名为imageGet_node
    ros::NodeHandle nh;                                                     // 定义ros句柄
    image_transport::ImageTransport it(nh);                                 //  类似ROS句柄
    image_transport::Publisher image_pub = it.advertise("/cameraImage", 1); // 发布话题名/cameraImage

    ros::Rate loop_rate(200); // 设置刷新频率，Hz

    // cv::Mat imageRaw;  // 原始图像保存
    // Mat image = imread("/home/ikun/桌面/1.png");
    // cv::VideoCapture capture(0);   // 创建摄像头捕获，并打开摄像头

    namedWindow("output", WINDOW_AUTOSIZE);             //创建窗口
    namedWindow("output1", WINDOW_AUTOSIZE);            //创建窗口
    src = imread("/home/ikun/桌面/1.png");      //读入图像
    resize(src, src, Size(src.cols / 2, src.rows / 2)); //裁剪图像
    src.copyTo(result);
    dst1 = Mat::zeros(Size(result.cols, result.rows * 1.054), CV_8UC3);
    Point center1;                     //俯视中心点
    Point p;                           //切面小球中心点
    Point p1;                          //俯面小球中心点
    double radius_1 = 0, radius_2 = 0; //切面半径中和
    vector<Vec3f> circles;             //获得圆的点

    cvtColor(src, src, COLOR_BGR2GRAY); //灰度化

    GaussianBlur(src, src1, Size(3, 3), 2, 2); //高斯滤波平滑去噪

    HoughCircles(src1, circles, HOUGH_GRADIENT, 1.5, 40, 85, 40, 10, 35); //霍夫圆变换找圆

    for (size_t i = 0; i < circles.size(); i++) //依次在图中绘制出圆
    {
        //参数定义
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cout << center << " << 圆点" << endl;
        cout << radius << " << 半径" << endl;
        //绘制圆心
        circle(result, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //绘制圆轮廓
        circle(result, center, radius, Scalar(255, 50, 255), 3, 8, 0);

        if (radius >= 20)
        {
            radius_1 += radius; //大球半径总和
        }
        else if (radius < 15)
        {
            radius_2 += radius; //小球半径总和
        }
    }

    threshold(src, dst2, 142, 255, 0);                              //二值化
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));    //定义内核
    morphologyEx(dst2, dst2, MORPH_ERODE, element, Point(1, 1), 1); //腐蚀操作
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(dst2, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) //找靠近黄色大球的小球
    {
        RotatedRect s = minAreaRect(contours[i]);
        if (s.boundingRect().area() > 400 && s.boundingRect().area() < 600 || s.boundingRect().area() > 200 && s.boundingRect().area() < 250)
        {

            p.x = s.boundingRect().tl().x + (s.boundingRect().width / 2.0);

            p.y = s.boundingRect().tl().y + (s.boundingRect().height / 2.0);

            int radius_2_1 = s.boundingRect().width / 2.0;
            cout << p << " << 圆点" << endl;
            cout << radius_2_1 << " << 半径" << endl;
            //绘制圆心
            circle(result, p, 2, Scalar(0, 255, 0), -1, 8, 0);
            //绘制圆轮廓
            circle(result, p, radius_2_1, Scalar(155, 50, 255), 3, 8, 0);

            radius_2 += radius_2_1;
        }
    }
    radius_1 = radius_1 / 2 / 1.054; //转换成俯视半径
    radius_2 = radius_2 / 3 / 1.054; //转换成俯视半径
    for (auto i = 0; i < circles.size(); i++)
    {
        int radius1 = cvRound(circles[i][2]);
        center1.x = cvRound(circles[i][0]);
        center1.y = cvRound(circles[i][1]) / 1.054;
        if (radius1 >= 20)
        {
            circle(dst1, center1, radius_1, Scalar(155, 50, 255), 3, 8, 0);
        }
        else if (radius1 < 15)
        {
            circle(dst1, center1, radius_2, Scalar(155, 50, 255), 3, 8, 0);
        }
    }

    p1.x = p.x;
    p1.y = p.y / 1.054;
    circle(dst1, p1, radius_2, Scalar(155, 50, 255), 2, 8, 0);
    imshow("output", result); //输出目标图
    imshow("output1", dst1);  //输出目标图

    waitKey(0);
    while (nh.ok())
    {
        //capture.read(image);          // 读取当前图像到imageRaw
        //cv::imshow("veiwer", image);                                                                  // 将图像输出到窗口
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg(); // 图像格式转换
        image_pub.publish(msg);                                                                       // 发布图像信息
        ros::spinOnce();                                                                              // 回调
        loop_rate.sleep();                                                                            // 照应上面设置的频率
        if (waitKey(2) >= 0)                                                                          // 延时ms,按下任何键退出(必须要有waitKey，不然是看不到图像的)
            break;
    }
}
