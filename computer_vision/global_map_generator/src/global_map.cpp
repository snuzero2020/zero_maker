#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include "ros/package.h"
#include "std_msgs/Int32MultiArray.h"

using namespace cv;
using namespace std;

int IMAGE_HEIGHT = 1200;
int IMAGE_WIDTH = 1040;

int node_points_x[4] = {180, 407, 634, 861};
int node_points_y[4] = {254, 487, 720, 953};

std::string middle_matrix_file = ros::package::getPath("global_map_generator") + "/perspective_matrix_middle.txt";
std::string right_matrix_file = ros::package::getPath("global_map_generator") + "/perspective_matrix_right.txt";
std::string left_matrix_file = ros::package::getPath("global_map_generator") + "/perspective_matrix_left.txt";

class GlobalMapPublisher
{
    private:
        int node_points[16][2];
        ros::NodeHandle nh;
        ros::Publisher global_map_pub;
        ros::Publisher node_obstacle_info_pub;
        cv::Mat M_middle = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat M_right = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat M_left = cv::Mat::zeros(3, 3, CV_64FC1);

    public:
        GlobalMapPublisher(){
            global_map_pub = nh.advertise<sensor_msgs::Image>("/global_map", 1);
            node_obstacle_info_pub = nh.advertise<std_msgs::Int32MultiArray>("/node_obstacle_info", 1);
            ROS_INFO("Glabal Map Publisher Loaded");

            for(int i=0; i<4; i++){
                for(int j=0; j<4; j++){
                    node_points[4*i + j][0] = node_points_x[i];
                    node_points[4*i + j][1] = node_points_y[j];
                }
            }
        }
        
        void global_map_callback(cv::Mat img_middle, cv::Mat img_right, cv::Mat img_left);
        void load_birdeye_matrix();
        void calc_obstalce_info(cv::Mat global_map);
        cv::Mat birdeye(cv::Mat M, cv::Mat img);

};

void GlobalMapPublisher::calc_obstalce_info(cv::Mat global_map){
    int node_info[16] = {0};
    std_msgs::Int32MultiArray array;

    for(int i=0; i<16; i++){
        for(int s_x = node_points[i][0] - 10; s_x < node_points[i][0] + 10; s_x++){
            for(int s_y = node_points[i][1] - 10; s_y < node_points[i][1] + 10; s_y++){
                if(global_map.at<uchar>(s_y, s_x) == 255){
                    node_info[i] = 1;
                }
            }
        }
    }
    
    array.data.clear();
    for(int i=0; i<16; i++){
        array.data.push_back(node_info[i]);
        //std::cout << node_info[i] << std::endl;
    }
    node_obstacle_info_pub.publish(array);
}

cv::Mat GlobalMapPublisher::birdeye(cv::Mat M, cv::Mat img){
    cv::Mat out_img = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
    cv::warpPerspective(img, out_img, M, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
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

    cv::Mat global_map, global_map_vis;

    bitwise_and(middle_warped, right_warped, global_map);
    bitwise_and(left_warped, global_map, global_map);

    calc_obstalce_info(global_map);

    cv::resize(global_map, global_map_vis, cv::Size(global_map.cols/2, global_map.rows/2 ), 0, 0, CV_INTER_NN );
    
    /*
    for(int i=0; i<16; i++){
        cv::circle(global_map_vis, cv::Point(int(node_points[i][0]/2), int(node_points[i][1]/2)), 2, cv::Scalar(255,255,255));
    }*/

    cv::imwrite(ros::package::getPath("global_map_generator") + "/global_map.png", global_map);
    imshow("global_map", global_map_vis);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, global_map);
    img_bridge.toImageMsg(img_msg);
    global_map_pub.publish(img_msg);

    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_map_pub");
    cv::Mat img_middle = cv::imread(ros::package::getPath("global_map_generator") + "/img_sample/cam1/point_label_img_00005.jpg",CV_RGB2GRAY);
    cv::Mat img_right = cv::imread(ros::package::getPath("global_map_generator") + "/img_sample/cam2/point_label_img_00006.jpg",CV_RGB2GRAY);
    cv::Mat img_left = cv::imread(ros::package::getPath("global_map_generator") + "/img_sample/cam3/point_label_img_00005.jpg",CV_RGB2GRAY);

    GlobalMapPublisher global_map_publisher;
    global_map_publisher.load_birdeye_matrix();
    while(ros::ok())
        global_map_publisher.global_map_callback(img_middle, img_right, img_left);

    return 0;
}