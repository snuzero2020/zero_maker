#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>

cv::Mat birdeye(double M[][3], cv::Mat img)
{
    double newrow;
    double newcol;
    double base;

    cv::Mat out_img = cv::Mat::zeros(1200, 1040, CV_8UC3);

    for(int i = 0; i<640; i++){
      for(int j = 0; j<480; j++){
        newrow = i*M[0][0]+j*M[0][1]+M[0][2];
        // std::cout << newrow;
        newcol = i*M[1][0]+j*M[1][1]+M[1][2];
        base = i*M[2][0]+j*M[2][1]+M[2][2];
        if (int(newcol/base)>=0 && int(newcol/base)<=1200 && int(newrow/base)>=0 && int(newrow/base)<=1040)
          out_img.at<cv::Vec3b>(int(newcol/base), int(newrow/base)) = img.at<cv::Vec3b>(j, i);
      }
    }

    return out_img;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_map_publisher");
    ros::NodeHandle nh;

    // code for middle image
    cv::Mat img_middle = cv::imread("/home/ayoung/Downloads/point_label_img_00005.jpg",CV_RGB2GRAY);

    double M_middle[3][3];

    std::ifstream readfile_middle("/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_middle.txt");
    for(int i = 0; i<3; i++)
      for(int j=0; j<3; j++){
        readfile_middle >> M_middle[i][j];
      }

    cv::Mat out_img_middle;

    out_img_middle = birdeye(M_middle, img_middle);

    // cv::imshow("img", out_img_middle);
    // cv::waitKey(0);

    // code for right image
    cv::Mat img_right = cv::imread("/home/ayoung/Downloads/point_label_img_00006.jpg",CV_RGB2GRAY);

    double M_right[3][3];

    std::ifstream readfile_right("/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_right.txt");
    for(int i = 0; i<3; i++)
      for(int j=0; j<3; j++){
        readfile_right >> M_right[i][j];
      }

    cv::Mat out_img_right;

    out_img_right = birdeye(M_right, img_right);

    // cv::imshow("img", out_img_right);
    // cv::waitKey(0);

    // code for left image
    cv::Mat img_left = cv::imread("/home/ayoung/Downloads/point_label_img_00005(2).jpg",CV_RGB2GRAY);

    double M_left[3][3];

    std::ifstream readfile_left("/home/ayoung/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_right.txt");
    for(int i = 0; i<3; i++)
      for(int j=0; j<3; j++){
        readfile_left >> M_left[i][j];
      }

    cv::Mat out_img_left;

    out_img_left = birdeye(M_left, img_left);

    // cv::imshow("img", out_img_middle);
    // cv::waitKey(0);

    cv::Vec3b black(0,0,0);
    cv::Vec3b white(255,255,255);


    cv::Mat out_img = cv::Mat::zeros(1200, 1040, CV_8UC3);
    for(int i = 0; i<1040; i++){
      for(int j = 0; j<1200; j++){
        if((out_img_middle.at<cv::Vec3b>(j, i) != white) || (out_img_right.at<cv::Vec3b>(j, i) != white) || (out_img_left.at<cv::Vec3b>(j, i) != white))
          out_img.at<cv::Vec3b>(j, i) = cv::Vec3b(0,0,0);
        else
          out_img.at<cv::Vec3b>(j, i) = cv::Vec3b(255,255,255);        
      }
    }

    cv::imshow("img", out_img);
    cv::waitKey(0);

    cv_bridge::CvImage cv_out_img;
    cv_out_img.image = out_img;
    // cv_out_img.img = cv::cvtColor(cv_out_img.img, cv_out_img.img, CV_RGB2GRAY) 

    cv_out_img.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_out_img.toImageMsg(ros_image);


    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
    ros::Rate loop_rate(5);

  while (nh.ok())
  {
    pub.publish(ros_image);
    loop_rate.sleep();
  }
}