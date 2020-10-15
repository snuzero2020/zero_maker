#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

using namespace cv;
using namespace std;

std::string middle_matrix_file = "/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_middle.txt";
std::string right_matrix_file = "/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_right.txt";
std::string left_matrix_file = "/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/perspective_matrix_left.txt";

class GlobalMapPublisher
{
    private:
        ros::NodeHandle nh;
        ros::Publisher global_map_pub;
        cv::Mat M_middle = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat M_right = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat M_left = cv::Mat::zeros(3, 3, CV_64FC1);

    public:
        GlobalMapPublisher(){
            global_map_pub = nh.advertise<sensor_msgs::Image>("/global_map", 1);
            ROS_INFO("Glabal Map Publisher Loaded");
        }
        
        void global_map_callback(cv::Mat img_middle, cv::Mat img_right, cv::Mat img_left);
        void load_birdeye_matrix();
        cv::Mat birdeye(cv::Mat M, cv::Mat img);

};

cv::Mat GlobalMapPublisher::birdeye(cv::Mat M, cv::Mat img){
    cv::Mat out_img = cv::Mat::zeros(1200, 1040, CV_8UC1);
    cv::warpPerspective(img, out_img, M, Size(1040, 1200));
    return out_img;
}

void GlobalMapPublisher::load_birdeye_matrix(){

    std::ifstream readfile_middle(middle_matrix_file);
    std::ifstream readfile_right(right_matrix_file);
    std::ifstream readfile_left(left_matrix_file);

    for(int i = 0; i<3; i++){
      for(int j=0; j<3; j++){
          readfile_middle >> M_middle.at<double>(i,j);
          readfile_right >> M_right.at<double>(i,j);
          readfile_left >> M_left.at<double>(i,j);
      }
    }
}

void GlobalMapPublisher::global_map_callback(cv::Mat img_middle, cv::Mat img_right, cv::Mat img_left){
    cv::Mat middle_warped = birdeye(M_middle, img_middle);
    cv::Mat right_warped = birdeye(M_right, img_right);
    cv::Mat left_warped = birdeye(M_left, img_left);

    cv::Mat global_map;

    bitwise_and(middle_warped, right_warped, global_map);
    bitwise_and(left_warped, global_map, global_map);

    imshow("global_map", global_map);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, global_map);
    img_bridge.toImageMsg(img_msg);
    global_map_pub.publish(img_msg);

    cv::waitKey(0);
    cv::destroyAllWindows();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_map_pub");

    cv::Mat img_middle = cv::imread("/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam1/point_label_img_00005.jpg",CV_RGB2GRAY);
    cv::Mat img_right = cv::imread("/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam2/point_label_img_00006.jpg",CV_RGB2GRAY);
    cv::Mat img_left = cv::imread("/home/sunho/catkin_ws/src/zero_maker/computer_vision/global_map_generator/img_sample/cam3/point_label_img_00005.jpg",CV_RGB2GRAY);

    GlobalMapPublisher global_map_publisher;
    global_map_publisher.load_birdeye_matrix();
    global_map_publisher.global_map_callback(img_middle, img_right, img_left);

    return 0;
}