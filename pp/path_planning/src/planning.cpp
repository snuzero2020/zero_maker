#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ctime>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/UInt32.h"

#define boundary_distance 1.0
#define global_point_distance 1.0    
        
#define pixel_distance 0.01
#define random_distance 0.1
#define choose_parent_distance 0.1
#define rewire_distance 0.1

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
    Point* parrent_point = NULL;
    Point* children_point = NULL;
};


class Planning{
    private:
        ros::NodeHandle nh;
        ros::Subscriber global_map_sub;
        ros::Publisher local_path_pub;

        Point global_points[16];
        int start_point_index;
        int goal_point_index;
        vector<int> global_path;  

        int current_map [ int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance ) ][ int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance ) ]; 
        int h = int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance );
        int w = int( (3 * global_point_distance + 2 * boundary_distance)/pixel_distance );

    public:
        Planning(){
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
        }

        void GlobalMapCallback(const nav_msgs::OccupancyGrid & map){

            for (int i = 0; i < h; i++){
                for (int j = 0; j < w; j++){
                    current_map[i][j] =  map.data[j*w+i];
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
/*
                for (int i = 12; i < 16; ++i){
                    cout << global_points[i].cost << " ";
                }
                cout << "\n" << endl;
                for (int i = 8; i < 12; ++i){
                    cout << global_points[i].cost << " ";
                }
                cout << "\n" << endl;
                for (int i = 4; i < 8; ++i){
                    cout << global_points[i].cost << " ";
                }
                cout << "\n" << endl;
                for (int i = 0; i < 4; ++i){
                    cout << global_points[i].cost << " ";
                }
                cout << "\n" << endl;
                current_cost = current_cost + 1;
                cout << "\n" << endl;
*/
            }

            Point* current_point = global_points + goal_point_index;
            while ( current_point -> cost != 0){
                global_path.push_back(current_point - global_points);
                current_point = current_point -> parrent_point;
            }
            global_path.push_back(start_point_index);
/*
            for (vector<int>::size_type i = 0; i < global_path.size(); ++i){
                cout << global_path[i] << endl;
            }
*/ 
        }

        void rrt_star(){
            cout << 1 << endl;
            //for (int i = global_path.size(); i > 0; i = i - 1){

                //cout << i << endl;

                //point local_start;
                //point local_goal;
                //local_start.x = int ( global_points[global_path[i]].x/ pixel_distance );
                //local_start.y = int ( global_points[global_path[i]].y / pixel_distance );
                //local_goal.x = int ( global_points[global_path[i-1]].x / pixel_distance );
                //local_goal.y = int ( global_points[global_path[i-1]].y / pixel_distance );


                //cout << local_start.x << " " << local_start.y << " " << local_goal.x << " " << local_goal.y << endl;
                //vector<point> local_path; 
                //local_path.push_back(local_start); 
                //point rand_point = random();
/*
                for (int i = 0; i < local_path.size(); ++i){
                    double min_dis = 1000000000.0;
                    point* nearest_point;
                    if ( distance(rand_point, local_path[i]) != 0 ){
                        if ( distance(rand_point, local_path[i]) < min_dis){
                            nearest_point = &(local_path[i]);
                        }
                    }

                    cout << nearest_point->x << " " << nearest_point->y << endl;

                }

*/

//            }
            
        }

        point random(){
            srand((unsigned int)time(0));
            point temp;
            temp.x = rand() % h;
            temp.y = rand() % h;
            cout << temp.x << " " << temp.y << endl;
            return temp;
        }

        double distance(point p1, point p2){
            return pow(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) , 0.5) ;
        }

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Planning planning;

    while (ros::ok()){
        planning.rrt_star();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}