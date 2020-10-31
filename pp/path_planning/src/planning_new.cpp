#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ctime>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/UInt32.h"

#include "rrt_ys.h"
#include <nav_msgs/Path.h>
#include <algorithm>

#define boundary_distance 1.0
#define global_point_distance 1.0    
        
#define pixel_distance 0.01
#define random_distance 0.1
#define choose_parent_distance 0.1
#define rewire_distance 0.1
#define obstacle_check_distance 0.01
#define goal_point_arrive_distance 0.2

using namespace std;

struct Point {
    double x;
    double y;
    int cost = -1;
    int obstacle = 0;

    Point* next_point[4];
    Point* parrent_point = NULL;
};

struct point {
    int x = 0;
    int y = 0;
    int cost = 0;
    point* parrent_point = NULL;
    point* children_point = NULL;
};

//2020-10-28 하유섭

class tracker_path{ //이 class는 tracker에 전달할 path를 publish 하는 코드
    private:
        ros::NodeHandle nh;
        ros::Publisher path_pub;

        nav_msgs::Path route;
    public:
        void init()
        {
            path_pub = nh.advertise<nav_msgs::Path>("/tracker_path", 20);
        }
        void add(int x, int y)
        {
            geometry_msgs::PoseStamped p;
            p.pose.position.x = x;
            p.pose.position.y = y;
            route.poses.push_back(p);
        }
        void send()
        {
            reverse(route.poses.begin(), route.poses.end());
            path_pub.publish(route);
            route.poses.erase(route.poses.begin(), route.poses.end());
        }

};

//


class Planning{
    private:
        ros::NodeHandle nh;
        ros::Subscriber global_map_sub;
        ros::Publisher local_path_pub;

        Point global_points[16];
        int start_point_index;
        int goal_point_index;
        vector<int> global_path;  
        vector<point> local_paths;

        int current_map [ int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance ) ][ int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance ) ]; 
        int h = int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance );
        int w = int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance );

    public:
        Planning(){
            srand((unsigned int)time(0));
            global_map_sub = nh.subscribe("global_map", 2, &Planning::GlobalMapCallback, this);
            local_path_pub = nh.advertise<std_msgs::UInt32>("path",2);

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

            for (int i = 0; i < h; i++){
                for (int j = 0; j < w; j++){
                    current_map[i][j] =  0;
                }
            }
        }

        void GlobalMapCallback(const nav_msgs::OccupancyGrid & map){

            for (int i = 0; i < h; i++){
                for (int j = 0; j < w; j++){
                    //current_map[i][j] =  map.data[j*w+i];
                }
            }
            
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

                for (int i = 12; i < 16; ++i){
                    //cout << global_points[i].cost << " ";
                }
                //cout << "\n" << endl;
                for (int i = 8; i < 12; ++i){
                    //cout << global_points[i].cost << " ";
                }
                //cout << "\n" << endl;
                for (int i = 4; i < 8; ++i){
                    //cout << global_points[i].cost << " ";
                }
                //cout << "\n" << endl;
                for (int i = 0; i < 4; ++i){
                    //cout << global_points[i].cost << " ";
                }
                //cout << "\n" << endl;
                current_cost = current_cost + 1;
                //cout << "\n" << endl;
            }

            Point* current_point = global_points + goal_point_index;
            while ( current_point -> cost != 0){
                global_path.push_back(current_point - global_points);
                current_point = current_point -> parrent_point;
            }
            global_path.push_back(start_point_index);

            for (vector<int>::size_type i = 0; i < global_path.size(); ++i){
                //cout << global_path[i] << endl;
            }
        }

        void rrt_star(){
            //cout << 1 << endl;
            for (int i = global_path.size() - 1 ; i > global_path.size() - 2; i = i - 1){
                //cout << i << endl;

                point local_start;
                point local_goal;
                local_start.x = int ( global_points[global_path[i]].x / pixel_distance );
                local_start.y = int ( global_points[global_path[i]].y / pixel_distance );
                local_goal.x = int ( global_points[global_path[i-1]].x / pixel_distance );
                local_goal.y = int ( global_points[global_path[i-1]].y / pixel_distance );

                //cout << local_start.x << " " << local_start.y << " " << local_goal.x << " " << local_goal.y << endl;
/*
                if (obstacle_check(local_start, local_goal) == 0){
                    cout << obstacle_check(local_start, local_goal) << endl;
                    int dx = int (double (local_goal.x - local_start.x) / distance (local_start, local_goal) * random_distance / pixel_distance);
                    int dy = int (double (local_goal.y - local_start.y) / distance (local_start, local_goal) * random_distance / pixel_distance);
                    cout << dx << " " << dy << endl;
                    if (dx != 0){
                        for (int j = 0; j < abs(int ((local_goal.x - local_start.x) / dx) ); ++j){
                        point p;
                        p.x = dx * j + local_start.x;
                        p.y = dy * j + local_start.y;
                        local_paths.push_back(p);
                        }
                    }

                    else {
                        for (int j = 0; j < abs(int ((local_goal.y - local_start.y) / dy) ); ++j){
                            point p;
                            p.x = dx * j + local_start.x;
                            p.y = dy * j + local_start.y;
                            local_paths.push_back(p);
                        }

                    }
                }

                else {
                    point random_point = random();
                }

*/              
                vector<point> local_tree;
                local_tree.push_back(local_start);

                double goal_min_dis = 1000000000;
                double goal_min_index = 0;

                while ( goal_min_dis * pixel_distance > goal_point_arrive_distance){
                    point random_point = random();

                    //cout << random().x << endl;

                    double min_dis = 10000000000;
                    int min_index = 0;

                    for (int j = 0; j < local_tree.size(); ++j){
                        if (min_dis > distance(local_tree[j], random_point)){
                            min_dis = distance(local_tree[j], random_point);
                            min_index = j;
                        }
                    }

                    //cout << min_index << endl;

                    if (distance(local_tree[min_index], random_point) > random_distance){
                        point new_point;
                        new_point.x = local_start.x + int (random_distance / pixel_distance * (random_point.x - local_tree[min_index].x) / distance(local_tree[min_index], random_point) );
                        new_point.y = local_start.y + int (random_distance / pixel_distance * (random_point.y - local_tree[min_index].y) / distance(local_tree[min_index], random_point) );
                        local_tree.push_back(new_point);
                        new_point.parrent_point = &(local_tree[min_index]);
                        local_tree[min_index].children_point = &(new_point);                        
                    }

                    else {
                        local_tree.push_back(random_point);
                        random_point.parrent_point = &(local_tree[min_index]);
                        local_tree[min_index].children_point = &(random_point);
                    }

                    for (int jj = 0; jj < local_tree.size(); ++jj){
                        if (goal_min_dis  > distance(local_tree[jj], local_goal)){
                            goal_min_dis = distance(local_tree[jj], local_goal);
                            goal_min_index = jj;
                        }
                    }

                    if (goal_min_dis != 90){
                        cout << goal_min_dis << endl;
                    }

                }

                local_tree.push_back(local_goal);
                local_tree[goal_min_index].children_point = &(local_goal);
                local_goal.parrent_point = &(local_tree[goal_min_index]);

                point* current_point = &(local_goal);

                while ((current_point->x != local_start.x) && (current_point->y != local_start.y)){
                    local_paths.push_back(*(current_point));
                    current_point = current_point->parrent_point;
                }

                local_paths.push_back(local_start);

                for (int k = 0; k < local_paths.size(); ++ k){
                    // cout << local_paths[k].x << " " << local_paths[k].y << endl;
                }

            }

        }

        point random(){
            point temp;
            temp.x = rand() % h;
            temp.y = rand() % h;
            return temp;
        }

        double distance (point p1, point p2){
            return pow( (pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2)), 0.5 );
        }

        int obstacle_check ( point p1, point p2){
            int dx = int (double (p2.x - p1.x) / distance (p1, p2) * obstacle_check_distance / pixel_distance);
            int dy = int (double (p2.y - p1.y) / distance (p1, p2) * obstacle_check_distance / pixel_distance);

            int obstacle_check = 0;

            if (dx != 0){
                for (int i = 0; i < abs(int ((p2.x - p1.x) / dx)); ++i){
                    if ( current_map[p1.x + dx * i][p1.y + dy * i] == 1 ){
                        obstacle_check = 1;
                    }
                }
            }

            else{
                for (int i = 0; i < abs(int ((p2.y - p1.y) / dy)); ++i){
                    if ( current_map[p1.x + dx * i][p1.y + dy * i] == 1 ){
                        obstacle_check = 1;
                    }
                }                
            }

            return obstacle_check;
        }

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Planning planning;

    rrt_ys::rrt_star rrt;
    rrt.init(pixel_distance, random_distance, choose_parent_distance, rewire_distance, obstacle_check_distance, goal_point_arrive_distance);\
    tracker_path tpath;
    tpath.init();
    while (ros::ok()){

        
        //planning.rrt_star();
        ROS_INFO("dd");
        
        vector<vector<int> > map;

        for(int i=0; i<100; i++)

        {

            vector<int> temp;

            map.push_back(temp);

            for(int j=0; j<100; j++)

            {

                map[i].push_back(0);

            }

        }

        for(int i=0; i<30; i++)

        {

            map[rand()%100][rand()%100]=1;

        }

        //map[30][30] = 1;

        for(int i=0; i<10; i++)

        {

            for(int j=0; j<10; j++)

            {

                map[90+i][90+j] = 0;

            }

        }

        

        map[0][0] = 3;

        ROS_INFO("dd");

        vector<ys::point> route = rrt.rrtstar(map, 0,0,99,99);
        ROS_INFO("dd");
        
        
        for(int i=0;i<route.size();i++)
        {
            tpath.add(route[i].x, route[i].y);
            ROS_INFO("%d, %d \n", route[i].x, route[i].y);
        }
        tpath.send();
ROS_INFO("dd");
        ros::spinOnce();
        //loop_rate.sleep();
        ROS_INFO("dd");
    }
    return 0;
}
