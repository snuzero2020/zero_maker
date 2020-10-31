#pragma once

#include <iostream>

#include <string>

#include <vector>

#include <math.h>

#include <stdlib.h>

#include <time.h>

#include <algorithm>

using namespace std;

 

namespace ys{

	class point{

		public:

			int x;

			int y;

			point(int ax, int ay){

				x=ax;

				y=ay;

			}

			point()

			{

				x=-1;

				y=-1;

			}

			vector<point> seek_point(vector<vector<int> > map, double dis, int type)

			{

				vector<point> result;

				for(int i=0; i<map.size(); i++)

				{

					for(int j=0;j<map[i].size(); j++)

					{

						if(sqrt((i-x)*(i-x)+(j-y)*(j-y)) <= dis)

						{

							if(map[i][j] == type)

								result.push_back(point(i, j));

						}

					}

				}

				return result;

			}

			point close_point(vector<vector<int> > map, int type)

			{

				point result= point(-1,-1);

				double temp = 100000;

				for(int i=0; i<map.size(); i++)

				{

					for(int j=0;j<map[i].size(); j++)

					{

						if(sqrt((i-x)*(i-x)+(j-y)*(j-y)) <= temp)

						{

							if(map[i][j] == type)

							{

								temp = sqrt((i-x)*(i-x)+(j-y)*(j-y));

								result.x = i;

								result.y = j;

							}

						}

					}

				}

				

				return result;

			}

			/*vector<point> seek_node(vector<point> route, double dis)

			{

				vector<point> result;

				for(int i=0; i<map.size(); i++)

				{

					

				}

				return result;

			}*/

			

	};

	double get_distance(point p1, point p2);

	double P_Line_distance(point P, point A, point B);

	vector<ys::point> allpoint(vector<vector<int> > map, int type);

	

	class node : public point{

		public:

		node* parent;

		double cost;

		node(int ax, int ay) : point(ax, ay){

			parent = NULL;

		}

		node() : point(){

			parent = NULL;

		}

		node(int ax, int ay, node* aparent) : point(ax, ay)

		{

			parent = aparent;

		}

		double get_cost() // 사용 금지 

		{

			if(parent == NULL)

				return 0;

			return get_distance(*this, *parent) + parent->get_cost();

		}

	};

	

	point close_node(vector<ys::node*> route, point p);

}

 

 

 

namespace rrt_ys{

 

	class rrt_star{

		public:

			void init(double pixel_distance, double random_distance, double choose_parent_distance, double rewire_distance, double obstacle_check_distance, double goal_point_arrive_distance)

			{

				pixel_dis = pixel_distance;

				rand_dis = random_distance;

				parent_dis = choose_parent_distance;

				rewire_dis = rewire_distance;

				obs_dis = obstacle_check_distance;

				goal_dis = goal_point_arrive_distance;

			}

			vector<ys::point> rrtstar(vector<vector<int> > map, int startx, int starty, int goalx, int goaly);

		private:

			double pixel_dis;

			double rand_dis;

			double parent_dis;

			double rewire_dis;

			double obs_dis;

			double goal_dis;

	

	

	};

}
