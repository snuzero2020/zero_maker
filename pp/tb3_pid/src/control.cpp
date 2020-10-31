#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <math.h>



// global vars
tf::Point Odom_pos;    //odometry position (x, y, z)
double Odom_yaw;    //odometry orientation (yaw)
double Odom_v, Odom_w;    //odometry linear and angular speeds

//
geometry_msgs::Point nodes[10000]; // 경로 point들 
int nodes_length = 0; //경로 점의 개수 
int current_index = 0; //현재 어느 점에 있는가 (다음 점에 도착하기 전까지 유지)
bool neednewpath = false;

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;
ros::Subscriber path_sub;



void odomCallback(const nav_msgs::Odometry odom_msg) {
    tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

    Odom_v = odom_msg.twist.twist.linear.x;
    Odom_w = odom_msg.twist.twist.angular.z;

}

void pathCallback(const nav_msgs::Path path_msg) {

    if(!neednewpath)
        return;
    nodes_length = path_msg.poses.size();

    for(int i=0;i<10000; i++)
    {
        if(i<nodes_length)
        {
            nodes[i].x = path_msg.poses[i].pose.position.x;
            nodes[i].y = path_msg.poses[i].pose.position.y;
            nodes_length = i;
        }
        else{
            nodes[i].x = -10000;
            nodes[i].y = -10000;
        }
    }
    
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "control");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    odom_sub = n.subscribe("/odom", 10, &odomCallback);
    path_sub = n.subscribe("/tracker_path", 10, &pathCallback);

    nodes[0].x = 3;
    nodes[0].y = 2;
    nodes[1].x = -1;
    nodes[1].y = 3;
    nodes[2].x = 1;
    nodes[2].y = 1;
    nodes[3].x = 0;
    nodes[3].y = 0;//이건 테스트 용

    ros::Rate loop_rate(10); // ros spins 10 frames per second

    //we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
    geometry_msgs::Twist tw_msg;
    
    while (ros::ok()) {
       
       
        geometry_msgs::Point current_pos;  // 현재 위치
	    double current_yaw;  // 현재 바라보는 방향
        geometry_msgs::Point goal_pos; //목표 노드
        double goal_yaw; //목표점으로 향하는 방향


        current_pos.x = Odom_pos.x();
        current_pos.y = Odom_pos.y();
	    current_yaw = Odom_yaw;
        goal_pos.x = nodes[current_index].x;
        goal_pos.y = nodes[current_index].y;
        
        if(goal_pos.x - current_pos.x == 0)
        {
            goal_yaw = 0;
        }
        else
            goal_yaw = atan((goal_pos.y - current_pos.y)/(goal_pos.x - current_pos.x));
        if(goal_pos.x - 1 <= current_pos.x && goal_pos.x + 1 >= current_pos.x)
        {
            if(goal_pos.y - 1 <= current_pos.y && goal_pos.y + 1 >= current_pos.y)
            {
                current_index++;
                tw_msg.linear.x = 0;
                tw_msg.angular.z = 0;
                cmd_vel_pub.publish(tw_msg);
                if(nodes[current_index].x == -10000)
                {
                    neednewpath = true;
                }
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
        }

        
        if(fabs(current_yaw - goal_yaw) <= 0.05)
        {
            tw_msg.linear.x = 0.1;
            tw_msg.angular.z = 0;
        }
        else if(fabs(current_yaw - goal_yaw) <= 0.25)
        {
            tw_msg.linear.x = 0.05;
            
            if(current_yaw>=goal_yaw || current_yaw<goal_yaw - 3.141592)
            {
                tw_msg.angular.z = -0.05;
            }
            else{
                tw_msg.angular.z = 0.05;
            }
        }
        else{
            tw_msg.linear.x = 0;
            
            if(current_yaw>=goal_yaw || current_yaw<goal_yaw - 3.141592)
            {
                tw_msg.angular.z = -0.1;
            }
            else{
                tw_msg.angular.z = 0.1;
            }
                
            
        }

        ROS_INFO("destination : (%f, %f)", goal_pos.x, goal_pos.y);

        cmd_vel_pub.publish(tw_msg);
        ROS_INFO("now : (%f, %f) \n", current_pos.x, current_pos.y);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
