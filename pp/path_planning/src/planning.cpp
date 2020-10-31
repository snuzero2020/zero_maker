#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ctime>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/UInt32.h"

#define boundary_distance 1.0
#define global_point_distance 1.0    

using namespace std;

struct Point {
    double x;
    double y;
    int cost = -1;
    int obstacle = 0;

    Point* next_point[4];
    Point* parrent_point = NULL;
};

class Planning{
    private:
        ros::NodeHandle nh;
        ros::Subscriber start_goal_obstacle_point_sub;
        ros::Publisher global_path_pub;

        Point global_points[16];
        int start_point_index;
        int goal_point_index;
        vector<int> global_path;  

    public:
        Planning(){
            global_path_pub = nh.advertise<nav_msgs::OccupancyGrid>("path",2);
            start_goal_obstacle_point_sub = nh.subscribe("obstacle_point", 2, &Planning::StartGoalObstaclePointCallback, this);
            for (int i = 0; i < 16; ++i){
                global_points[i].x = (i % 4) * global_point_distance + boundary_distance;
                global_points[i].y = (i / 4) * global_point_distance + boundary_distance;
            }

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

            calculate_global_path();
        }

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

        }

        void publish_path(){

            nav_msgs::OccupancyGrid msg;

            for (int i = global_path.size() - 1; i > -1; i = i - 1){
                //cout << global_points[global_path[i]].x << " " << global_points[global_path[i]].y << endl;
                msg.data.push_back(global_points[global_path[i]].x);
                msg.data.push_back(global_points[global_path[i]].y);
            }

            global_path_pub.publish(msg);

        }

        void StartGoalObstaclePointCallback (const nav_msgs::OccupancyGrid & msg){

            for (int i = 0; i < 16; ++i){
                global_points[i].obstacle = msg.data[i]; 
            }
            
        }

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Planning planning;

    while (ros::ok()){

        planning.publish_path();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}