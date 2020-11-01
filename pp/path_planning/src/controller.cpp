#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <sstream>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"

#include "ros/ros.h"
#include "ros/time.h"


using namespace std;

typedef pair<double, double> pdd;


double min_abs(double a, double b, double c){
    if(abs(a)<abs(b)){
        if(abs(a)<abs(c)) return a;
        return c;
    }
    else{
        if(abs(b)<abs(c)) return b;
        return c;
    }
}

geometry_msgs::Quaternion q_from_e(double r, double p, double y){
    double cr = cos(0.5*r);
    double sr = sin(0.5*r);
    double cp = cos(0.5*p);
    double sp = sin(0.5*p);
    double cy = cos(0.5*y);
    double sy = sin(0.5*y);

    geometry_msgs::Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;

    return q;
}


class Controller{
    private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub_state;
    ros::Subscriber _sub_goal_point;
    ros::ServiceClient _client;
    string target_name = "turtlebot";
    pdd cur_position;
    double cur_theta;
    double L;
    double R;
    double P_gain;
    double vel;
    clock_t now;
    double threshold_time;

    public:
    Controller(){
        _pub = _nh.advertise<geometry_msgs::Pose>("/current_state", 10);
        _sub_state = _nh.subscribe("/gazebo_model_state", 1, &Controller::callback_state, this);
        _sub_goal_point = _nh.subscribe("/goal_point", 1, &Controller::callback_goal_point, this);
        _client = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
        L = 0.178;
        R = 0.033;
        vel = 1;
        now = clock();
        threshold_time = 1;
    }

    void callback_state(const gazebo_msgs::ModelStates::ConstPtr& msg){
        geometry_msgs::Pose rt;
        int size = msg->name.size();
        int index = -1;
        int i;
        for(i=0;i<size;i++){
            if(msg->name[i].compare(target_name) == 0){
                index = i;
                break;
            }
        }
        
        if(index == -1){
            cout << "No target" << endl;
            return;
        }
        
        cur_position.first = msg->pose[i].position.x;
        cur_position.second = msg->pose[i].position.y;
        
        geometry_msgs::Quaternion q = msg->pose[i].orientation;
        double x = 2*(q.w*q.z+q.x*q.y);
        double y = 1-2*(q.y*q.y+q.z*q.z);
        cur_theta = atan2(x,y);

        rt = msg->pose[i];
        _pub.publish(rt);
    }

    void callback_goal_point(const geometry_msgs::Pose::ConstPtr& msg){
        double dt = (double)(clock() - now)/CLOCKS_PER_SEC;
        now = clock();
        if(dt>threshold_time) return;
        pdd g = {msg->position.x, msg->position.y};
        double d_theta = atan2(g.second-cur_position.second, g.first - cur_position.second); //desired theta
        double e_theta = d_theta - cur_theta;
        e_theta = min_abs(e_theta, e_theta+ 2* M_PI, e_theta-2*M_PI);
        double d_omega = P_gain * e_theta; //desired_omega

        geometry_msgs::Pose next;
        next.position.x = cur_position.first + vel*cos(cur_theta)*dt;
        next.position.y = cur_position.second + vel*sin(cur_theta)*dt;
        next.orientation = q_from_e(0,0,cur_theta+d_omega*dt);


        gazebo_msgs::ModelState turtlebot;
        turtlebot.model_name = target_name;
        turtlebot.pose = next;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = turtlebot;

        if(_client.call(srv)) ROS_INFO("Success to update turtlebot pose");
        else ROS_ERROR("ERROR to update turtlebot pose");
        return;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    Controller controller;
    ros::spin();
}

