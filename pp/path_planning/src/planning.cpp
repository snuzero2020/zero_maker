#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ctime>
#include <cmath>
#include <random>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/UInt32.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//#define boundary_distance_x 0.9
//#define boundary_distance_y 1.27
//#define global_point_distance 1.18   

#define pixel_distance 0.005
#define random_distance 0.1
#define choose_parent_and_rewire_distance 0.3
#define obstacle_check_distance 0.1
#define goal_point_reach_distance 0.3

using namespace std;

struct Point {

    double x;
    double y;

    int pixel_x;
    int pixel_y;

    int cost = -1;
    double ccost = 0;
    int obstacle = 0;

    Point* next_point[4];
    Point* parrent_point = NULL;

    int parrent_index = -1;
};

class Planning{
    private:
        ros::NodeHandle nh;

        ros::Subscriber start_goal_obstacle_point_sub;
        ros::Publisher global_path_pub;
        //ros::Subscriber map_sub;
        ros::Publisher path_pub;
        ros::Subscriber current_position_sub;
        ros::Publisher tracker_goal_point_pub;

        Point global_points[16];
        int start_point_index;
        int goal_point_index;
        vector<int> global_path;
        vector<Point> total_path;
        vector<Point> total_path_static;
        int check = 0;
        //int x_l = int((global_point_distance * 3 + 2 * boundary_distance_x)/ pixel_distance) ;
        //int y_l = int((global_point_distance * 3 + 2 * boundary_distance_y)/ pixel_distance) ;
        //int map[ int( (global_point_distance * 3 + 2 * boundary_distance_x)/ pixel_distance )][ int( (global_point_distance * 3 + 2 * boundary_distance_y)/ pixel_distance )] ;
        int x_l = 1040;
        int y_l = 1200;
        int map[1040][1200];
        Point current_state;

    public:

        Planning(){
            
            global_path_pub = nh.advertise<nav_msgs::OccupancyGrid>("path",2);
            start_goal_obstacle_point_sub = nh.subscribe("obstacle_point", 2, &Planning::StartGoalObstaclePointCallback, this);
            //map_sub = nh.subscribe("map", 2, &Planning::MapCallback, this);
            path_pub = nh.advertise<nav_msgs::Path>("tracker_path", 2);
            current_position_sub = nh.subscribe("current_position", 2, &Planning::CurrentPositionCallback, this);
            tracker_goal_point_pub = nh.advertise<geometry_msgs::Pose>("tracker_goal_point", 2);
/*
            for (int i = 0; i < 16; ++i){
                global_points[i].x = (i % 4) * global_point_distance + boundary_distance_x;
                global_points[i].y = (i / 4) * global_point_distance + boundary_distance_y;
            }
*/
            global_points[0].x = (180) * pixel_distance; 
            global_points[0].y = (y_l - 953) * pixel_distance; 
            global_points[1].x = (407) * pixel_distance; 
            global_points[1].y = (y_l - 953) * pixel_distance; 
            global_points[2].x = (634) * pixel_distance; 
            global_points[2].y = (y_l - 953) * pixel_distance; 
            global_points[3].x = (861) * pixel_distance; 
            global_points[3].y = (y_l - 953) * pixel_distance; 
            global_points[4].x = (180) * pixel_distance; 
            global_points[4].y = (y_l - 720) * pixel_distance; 
            global_points[5].x = (407) * pixel_distance; 
            global_points[5].y = (y_l - 720) * pixel_distance; 
            global_points[6].x = (634) * pixel_distance; 
            global_points[6].y = (y_l - 720) * pixel_distance; 
            global_points[7].x = (861) * pixel_distance; 
            global_points[7].y = (y_l - 720) * pixel_distance; 
            global_points[8].x = (180) * pixel_distance; 
            global_points[8].y = (y_l - 487) * pixel_distance; 
            global_points[9].x = (407) * pixel_distance; 
            global_points[9].y = (y_l - 487) * pixel_distance; 
            global_points[10].x = (634) * pixel_distance; 
            global_points[10].y = (y_l - 487) * pixel_distance; 
            global_points[11].x = (861) * pixel_distance; 
            global_points[11].y = (y_l - 487) * pixel_distance; 
            global_points[12].x = (180) * pixel_distance; 
            global_points[12].y = (y_l - 254) * pixel_distance; 
            global_points[13].x = (407) * pixel_distance; 
            global_points[13].y = (y_l - 254) * pixel_distance; 
            global_points[14].x = (634) * pixel_distance; 
            global_points[14].y = (y_l - 254) * pixel_distance; 
            global_points[15].x = (861) * pixel_distance; 
            global_points[15].y = (y_l - 254) * pixel_distance; 

            global_points[0].next_point[0] = global_points + 1;
            global_points[0].next_point[1] = global_points + 4;
            global_points[0].next_point[2] = NULL;
            global_points[0].next_point[3] = NULL;

            global_points[1].next_point[0] = global_points;
            global_points[1].next_point[1] = global_points + 5;
            global_points[1].next_point[2] = global_points + 2;
            global_points[1].next_point[3] = NULL;

            global_points[2].next_point[0] = global_points + 1;
            global_points[2].next_point[1] = global_points + 6;
            global_points[2].next_point[2] = global_points + 3;
            global_points[2].next_point[3] = NULL;

            global_points[3].next_point[0] = global_points + 2;
            global_points[3].next_point[1] = global_points + 7;
            global_points[3].next_point[2] = NULL;
            global_points[3].next_point[3] = NULL;

            global_points[4].next_point[0] = global_points + 0;
            global_points[4].next_point[1] = global_points + 5;
            global_points[4].next_point[2] = global_points + 8;
            global_points[4].next_point[3] = NULL;

            global_points[5].next_point[0] = global_points + 1;
            global_points[5].next_point[1] = global_points + 4;
            global_points[5].next_point[2] = global_points + 6;
            global_points[5].next_point[3] = global_points + 9;

            global_points[6].next_point[0] = global_points + 10;
            global_points[6].next_point[1] = global_points + 5;
            global_points[6].next_point[2] = global_points + 2;
            global_points[6].next_point[3] = NULL;

            global_points[8].next_point[0] = global_points + 12;
            global_points[8].next_point[1] = global_points + 9;
            global_points[8].next_point[2] = global_points + 4;
            global_points[8].next_point[3] = NULL;

            global_points[9].next_point[0] = global_points + 13;
            global_points[9].next_point[1] = global_points + 8;
            global_points[9].next_point[2] = global_points + 10;
            global_points[9].next_point[3] = global_points + 5;

            global_points[10].next_point[0] = global_points + 14;
            global_points[10].next_point[1] = global_points + 9;
            global_points[10].next_point[2] = global_points + 11;
            global_points[10].next_point[3] = global_points + 6;

            global_points[11].next_point[0] = global_points + 15;
            global_points[11].next_point[1] = global_points + 10;
            global_points[11].next_point[2] = global_points + 7;
            global_points[11].next_point[3] = NULL;

            global_points[12].next_point[0] = global_points + 13;
            global_points[12].next_point[1] = global_points + 8;
            global_points[12].next_point[2] = NULL;
            global_points[12].next_point[3] = NULL;

            global_points[13].next_point[0] = global_points + 12;
            global_points[13].next_point[1] = global_points + 9;
            global_points[13].next_point[2] = global_points + 14;
            global_points[13].next_point[3] = NULL;

            global_points[14].next_point[0] = global_points + 13;
            global_points[14].next_point[1] = global_points + 10;
            global_points[14].next_point[2] = global_points + 15;
            global_points[14].next_point[3] = NULL;

            global_points[15].next_point[0] = global_points + 14;
            global_points[15].next_point[1] = global_points + 11;
            global_points[15].next_point[2] = NULL;
            global_points[15].next_point[3] = NULL;

            // fill the obstacle
            //global_points[1].obstacle = 1;

            // 12 13 14 15
            // 8 9 10 11
            // 4 5 6 7
            // 0 1 2 3

            start_point_index = 0;
            goal_point_index = 15;

            global_points[start_point_index].cost = 0;

            read_map();

            calculate_global_path();
        }

        void read_map(){
            cv::Mat img = cv::imread("/home/lee/catkin_ws/src/zero_maker/computer_vision/global_map_generator/global_map.png", CV_LOAD_IMAGE_GRAYSCALE);
            if (!img.empty()){
                //cv::imshow("img", img);
                //cv2::waitKey(0);
                //cout << img.cols << " " << img.rows << endl;
                //cout << x_l << " " << y_l << endl;

                //int count1 = 0;
                //int count2 = 0;

                // 0 : can go , 255 : obstacle 
                //cout << img.at<int>(0, 0) << endl;
                for (int i = 0; i < x_l; ++i){
                    for (int j = 0; j < y_l; ++j){
                        if ( int(img.at<uchar>(j, i)) == 0){
                            map[i][j] = 0;
                        }
                        else{
                            map[i][j] = 1;
                        }
                    }
                }

                //img.at<uchar>(254, 180) = 255; // 12
                //img.at<uchar>(254, 407) = 255; // 13
                //img.at<uchar>(254, 634) = 255; // 14
                //img.at<uchar>(254, 861) = 255; // 15
                
                //img.at<uchar>(487, 180) = 255; // 8
                //img.at<uchar>(487, 407) = 255; // 9
                //img.at<uchar>(487, 634) = 255; // 10
                //img.at<uchar>(487, 861) = 255; // 11
                
                //img.at<uchar>(720, 180) = 255; // 4
                //img.at<uchar>(720, 407) = 255; // 5
                //img.at<uchar>(720, 634) = 255; // 6
                //img.at<uchar>(720, 861) = 255; // 7
                
                //img.at<uchar>(953, 180) = 255; // 0
                //img.at<uchar>(953, 407) = 255; // 1
                //img.at<uchar>(953, 634) = 255; // 2
                //img.at<uchar>(953, 861) = 255; // 3
                //int a = y_l - int(global_points[0].y/pixel_distance);
                //int b = int(global_points[0].x/pixel_distance);
                //cout << a << " " << b << endl;
                //img.at<uchar>(a,b) = 255;

                //cv::Mat new_img(y_l, x_l, CV_8UC3);
                //new_img.at<cv::Vec3b>(100, 200)[0] = 255;
                //new_img.at<cv::Vec3b>(100, 200)[1] = 0;
                //new_img.at<cv::Vec3b>(100, 200)[1] = 0;

                cv::imwrite("/home/lee/catkin_ws/src/zero_maker/computer_vision/global_map_generator/new_map.png", img);
                //cv::waitKey(0);
            }

            else{
                cout << "open failed" << endl;
            }            
        }

        void CurrentPositionCallback(const geometry_msgs::Pose& msg){
            current_state.pixel_x = 520 - int(msg.position.y / pixel_distance);
            current_state.pixel_y = int(msg.position.x / pixel_distance) + 600;
            
            int min_index;
            
            for (int i = 0; i < total_path_static.size(); ++i){
                int min_len = 1000000;
                if (distance(total_path_static[i], current_state) < min_len){
                    min_index = i;
                    min_len = distance(total_path_static[i], current_state);
                }
            }

            cout << current_state << endl;

            int goal_plus_index = 2;

            geometry_msgs::Pose mmsg;

            if ( min_index + goal_plus_index < total_path_static.size()){
                mmsg.position.x = (total_path_static[min_index + goal_plus_index].pixel_y - 600) * pixel_distance;
                mmsg.position.y = (520 - total_path_static[min_index + goal_plus_index].pixel_x) * pixel_distance;
                tracker_goal_point_pub.publish(mmsg);
            }
        }
/*
        void MapCallback(const nav_msgs::OccupancyGrid & msg){
            for (int i=0; i < x_l; ++i){
                for (int j = 0; j < y_l; ++j){
                    map[i][j] = msg.data[x_l*i+j];
                }
            }
        }
*/
        void calculate_global_path(){

            int current_cost = 0;

            while (global_points[goal_point_index].cost == -1){
                //global_points[goal_point_index].cost = 0;
                for (int i = 0; i < 16; ++i){
                    if (global_points[i].cost == current_cost){
                        for (int j = 0; j < 4; ++j){
                            if ( (global_points[i].next_point[j]) != NULL){
                                if ( (global_points[i].next_point[j])->obstacle == 0){
                                    if( (global_points[i].next_point[j])->cost == -1 ){
                                        (global_points[i].next_point[j])->cost = current_cost + 1;
                                        (global_points[i].next_point[j])->parrent_point = global_points + i;
                                    }
                                }
                            }
                        }
                    }
                }

                current_cost = current_cost + 1;

            }

            Point* current_point = global_points + goal_point_index;
            while ( current_point -> cost != 0){
                global_path.push_back(current_point - global_points);
                current_point = current_point -> parrent_point;
            }
            global_path.push_back(start_point_index);

            for (int i = 0; i < global_path.size(); ++i){
                cout << global_path[i] << endl;
            }

        }
/*
        void publish_path(){

            nav_msgs::OccupancyGrid msg;

            for (int i = global_path.size() - 1; i > -1; i = i - 1){
                //cout << global_points[global_path[i]].x << " " << global_points[global_path[i]].y << endl;
                msg.data.push_back( (global_points[global_path[i]].x) * 100);
                msg.data.push_back( (global_points[global_path[i]].y) * 100);
            }

            global_path_pub.publish(msg);

        }
*/
        void StartGoalObstaclePointCallback (const nav_msgs::OccupancyGrid & msg){

            for (int i = 0; i < 16; ++i){
                global_points[i].obstacle = msg.data[i]; 
            }
            
        }

        void local_path_planning(){
            total_path.clear();
            srand(time(NULL));
            for( int i = 0; i < global_path.size() - 1; i = i + 1){
                rrt_start( global_path[i+1], global_path[i]);
            }

            nav_msgs::Path msg;

            reverse(total_path.begin(), total_path.end());

            if (check == 0){
                for (int i = 0; i < total_path.size(); ++i){
                    total_path_static.push_back(total_path[i]);
                }
                check = 1;
            }

            cv::Mat img = cv::imread("/home/lee/catkin_ws/src/zero_maker/computer_vision/global_map_generator/global_map.png", CV_LOAD_IMAGE_GRAYSCALE);

            for (int i = 0; i < total_path.size(); ++i){
                cout << total_path[i].pixel_x << " " << total_path[i].pixel_y << " "  << total_path[i].ccost << endl;
                
                int a = y_l - total_path[i].pixel_y;
                int b = total_path[i].pixel_x;
                //cout << a << " " << b << endl;
                img.at<uchar>(a-1,b-1) = 255;
                img.at<uchar>(a-1,b) = 255;
                img.at<uchar>(a-1,b+1) = 255;
                img.at<uchar>(a,b-1) = 255;
                img.at<uchar>(a,b) = 255;
                img.at<uchar>(a,b+1) = 255;
                img.at<uchar>(a+1,b-1) = 255;
                img.at<uchar>(a+1,b) = 255;
                img.at<uchar>(a,b+1) = 255;

                //geometry_msgs::PoseStamped p;
                //p.pose.position.x = total_path[i].pixel_x * pixel_distance;
                //p.pose.position.y = total_path[i].pixel_y * pixel_distance;
                //p.header.seq = i;
                //msg.poses.push_back(p);
            }

            cv::imwrite("/home/lee/catkin_ws/src/zero_maker/computer_vision/global_map_generator/new_map.png", img);

            //path_pub.publish(msg);
/*
            int min_index;
            for (int i = 0; i < total_path_static.size(); ++i){
                int min_len = 1000000;
                if (distance(total_path_static[i], current_state) < min_len){
                    min_index = i;
                    min_len = distance(total_path_static[i], current_state);
                }
            }

            int goal_plus_index = 1;

            geometry_msgs::Pose mmsg;

            if ( min_index + goal_plus_index < total_path_static.size()){
                mmsg.position.x = (total_path_static[min_index + goal_plus_index].pixel_y - 600) * pixel_distance;
                mmsg.position.y = (520 - total_path_static[min_index + goal_plus_index].pixel_x) * pixel_distance;
                tracker_goal_point_pub.publish(mmsg);
            }
*/
            //cout << "-----------------------------------------" << endl;

        }

        void rrt_start (int start, int goal){

            Point start_point, goal_point;
            start_point.pixel_x = int ( global_points[start].x / pixel_distance );
            start_point.pixel_y = int ( global_points[start].y / pixel_distance );
            start_point.ccost = 0;
            goal_point.pixel_x = int ( global_points[goal].x / pixel_distance );
            goal_point.pixel_y = int ( global_points[goal].y / pixel_distance );

            vector<Point> tree;
            tree.push_back(start_point);

            while(goal_reach(tree, goal_point)){
                Point rand_point = random_point();
                Point near_point = tree[nearst_point_index(tree, rand_point)];
                if(obstacle_check(near_point, rand_point)){

                    if (distance(near_point, rand_point) < random_distance/pixel_distance){
                        //cout << distance(near_point, rand_point) << endl;
                        if (distance(near_point, rand_point) > 0.5){
                            //cout << distance(near_point, rand_point) << endl;
                            rand_point.parrent_index = nearst_point_index(tree, rand_point) ;
                            //cout << &(rand_point) << " " << &(near_point) << endl;
                            rand_point.ccost = near_point.ccost + distance(near_point, rand_point);
                            tree.push_back(rand_point);
                        }
                    }

                    else {
                        Point new_point;
                        new_point.pixel_x = int( (rand_point.pixel_x - near_point.pixel_x) * (random_distance/pixel_distance) / distance(near_point, rand_point) + near_point.pixel_x );
                        new_point.pixel_y = int( (rand_point.pixel_y - near_point.pixel_y) * (random_distance/pixel_distance) / distance(near_point, rand_point) + near_point.pixel_y );
                        //cout << distance(near_point, new_point) << endl;
                        if (distance(near_point, new_point) > 0.5){
                            //cout << distance(near_point, rand_point) << endl;
                            //cout << &(new_point) << " " << &(near_point) << endl;
                            new_point.parrent_index = nearst_point_index(tree, rand_point);
                            new_point.ccost = near_point.ccost + distance(near_point, new_point);
                            tree.push_back(new_point);
                        }
                    }
                }

                for (int i = 0; i < tree.size() - 1; ++i){
                    double min_cost = 100000.0;
                    if ( distance(tree[tree.size()-1], tree[i]) < choose_parent_and_rewire_distance ){
                        if (obstacle_check(tree[tree.size()-1], tree[i])){
                            if ( distance(tree[tree.size()-1], tree[i]) + tree[i].ccost < min_cost ){
                                min_cost = distance(tree[tree.size()-1], tree[i]) + tree[i].ccost;
                                tree[tree.size()-1].parrent_index = i;
                                tree[tree.size()-1].ccost = min_cost;
                            }
                        }
                    }
                }

                for (int i = 0; i < tree.size() - 1; ++i){
                    double min_cost = 100000.0;
                    if ( distance(tree[tree.size()-1], tree[i]) < choose_parent_and_rewire_distance ){
                        if (obstacle_check(tree[tree.size()-1], tree[i])){
                            if ( distance(tree[tree.size()-1], tree[i]) + tree[tree.size()-1].ccost < min_cost ){
                                min_cost = distance(tree[tree.size()-1], tree[i]) + tree[tree.size()-1].ccost;
                                tree[i].parrent_index = tree.size()-1;
                                tree[i].ccost = min_cost;
                            }
                        }
                    }
                }
            }

            int num = 0;

            for(int i = 0; i < tree.size(); ++i){
                if(distance(tree[i], goal_point) < goal_point_reach_distance/pixel_distance){
                    num = i;
                }
            } 

            //cout << tree.size() << endl;
            total_path.push_back(goal_point);

            Point current_point = tree[num];

            while (current_point.parrent_index != -1){
                //cout << current_point.pixel_x << " " << current_point.pixel_y << endl;
                total_path.push_back(current_point);
                current_point = tree[current_point.parrent_index];
            }

            total_path.push_back(start_point);

            //cout << start_x << " " << start_y << " " << goal_x << " " << goal_y << endl;
        }

        int obstacle_check (Point p, Point q){
            int check = 1;

            int px = p.pixel_x;
            int py = p.pixel_y;
            int qx = q.pixel_x;
            int qy = q.pixel_y;
            
            int a = py - qy;
            int b = -(px - qx);
            int c = -px*(py-qy) + py*(px-qx);

            for (int i = std::min(px, qx); i < std::max(px, qx) + 1; ++i ){
                for (int j = std::min(py, qy); j < std::max(py, qy) + 1; ++j ){
                    double d = double(std::abs(a*i + b*j + c)) / double(pow(a*a + b*b, 0.5));
                    if ( d < obstacle_check_distance / pixel_distance ){
                        if (map[i][j] == 1 ){
                            check = 0;
                        }
                    }
                }
            }

            return check;        
        }

        int nearst_point_index(vector<Point> tree, Point point){
            double min_l = 100000000000;
            int min_index = 0;

            for (int i = 0; i < tree.size(); ++i){
                if (distance(tree[i], point) < min_l){
                    min_l = distance(tree[i], point);
                    min_index = i;
                }
            }

            return min_index;
        }

        int goal_reach( vector<Point> tree, Point goal_point){
            
            int check = 1;
            
            for(int i = 0; i < tree.size(); ++i){
                if(distance(tree[i], goal_point) < goal_point_reach_distance/pixel_distance){
                    check = 0;
                }
            }

            return check;
        }

        double distance ( Point a, Point b){
            return pow(pow(a.pixel_x-b.pixel_x,2) + pow(a.pixel_y-b.pixel_y, 2), 0.5);
        }

        Point random_point(){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis_x(0, x_l - 1);
            std::uniform_int_distribution<int> dis_y(0, y_l - 1);
            Point rand_point;
            rand_point.pixel_x = dis_x(gen);
            rand_point.pixel_y = dis_y(gen);
            //cout << rand_point.pixel_x << " " << rand_point.pixel_y << endl;
            return rand_point;
        }



};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Planning planning;

    while (ros::ok()){
        //planning.publish_path();
        planning.local_path_planning();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}