
#include "rrt_ys.h"

#include <fstream>

 

#define p_empty 0

#define p_obs 1

#define p_goal 2

#define p_route 3

 


#define getdis(x, y, ax, ay) sqrt((x-ax)*(x-ax) + (y-ay)*(y-ay))

namespace ys{

	double get_distance(point p1, point p2)

	{

		return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));

	}

	double P_Line_distance(point P, point A, point B){

		point vPA;

		vPA.x = A.x - P.x;

		vPA.y = A.y - P.y;

		point vAB;

		vAB.x = B.x - A.x;

		vAB.y = B.y - A.y;

		point vBP;

		vBP.x = P.x - B.x;

		vBP.y = P.y - B.y;

		

		if(vPA.x * vAB.x + vPA.y * vAB.y <=0 && vBP.x * vAB.x + vBP.y * vAB.y <=0)

		{

			double area = abs ( (A.x - P.x) * (B.y - P.y) - (A.y - P.y) * (B.x - P.x) );

			double AB = sqrt( (A.x - B.x) *(A.x - B.x)+ (A.y - B.y) *(A.y - B.y));

			return ( area / AB );

		}

		else if(vPA.x * vAB.x + vPA.y * vAB.y >0)

			return get_distance(P, A);

		else

			return get_distance(P, B);

		

		

	}

	vector<point> allpoint(vector<vector<int> > map, int type)

	{

		vector<point> result;

		for(int i=0; i<map.size(); i++)

		{

			for(int j=0;j<map[i].size(); j++)

			{

				

				if(map[i][j] == type)

					result.push_back(point(i, j));

				

			}

		}

		return result;

	}
	point close_node(vector<node*> route, point p)

	{

		int x= p.x;

		int y= p.y;

		point result= point(-1,-1);

		double temp = 100000;

		for(int i=0; i<route.size(); i++)

		{

			if(sqrt((route[i]->x-x)*(route[i]->x-x)+(route[i]->y-y)*(route[i]->y-y)) <= temp)

			{

				temp = sqrt((route[i]->x-x)*(route[i]->x-x)+(route[i]->y-y)*(route[i]->y-y));

				result.x = route[i]->x;

				result.y = route[i]->y;

			}

		}

		return result;

	}
}

namespace rrt_ys{

	

	vector<ys::point> rrt_star::rrtstar(vector<vector<int> > map, int startx, int starty, int goalx, int goaly)

	{

		if(ys::point(0,0).close_point(map, p_obs).x == -1)

		{

			vector<ys::point> route;

			route.push_back(ys::point(0,0));

			route.push_back(ys::point(goalx,goaly));

			return route;

		}

		vector<ys::node*> route;

		ys::node start_node;

		

		start_node.x = startx;

		start_node.y = starty;

		start_node.parent=NULL;

		start_node.cost=0;

		route.push_back(&start_node);

	

		srand(time(NULL));

		

		while(1)

		{

			

			ys::node temp(rand()%(goalx+1), rand()%(goaly+1));

			if(map[temp.x][temp.y] != 0) continue;

			bool check=true;

			

			for(int i=0; i<route.size(); i++)

			{

				if(route[i]->x == temp.x && route[i]->y == temp.y){

				//	cout << route[i]->x <<" "<<temp.x<<endl;

					check = false;

					break;

				} 

			}

			if(!check) continue;

			ys::point temp_close = ys::close_node(route, temp);

 

			if(ys::get_distance(temp_close, temp) <= rand_dis/pixel_dis)

			{

				vector<ys::point> obs = ys::allpoint(map, p_obs);

				bool reachable = false;

				for(int i=0; i<obs.size() ; i++)

				{

					for(int j=0; j<route.size();j++){

					

						if(ys::P_Line_distance(ys::point(obs[i].x, obs[i].y), temp, temp_close) >= obs_dis/pixel_dis)

						{

							reachable = true;

							//cout << ys::P_Line_distance(ys::point(obs[i].x, obs[i].y), temp, temp_close) << endl;

						}

					}

					

				}

				if(!reachable)

					continue;

				

				vector<ys::node*> nearnodes;

				for (int i=0; i<route.size(); i++)

				{

					if(ys::get_distance(*route[i], temp) <= parent_dis/pixel_dis)

					{

						bool reachable = true;

						for(int j=0; j<obs.size() ; j++)

						{

							if(ys::P_Line_distance(ys::point(obs[j].x, obs[j].y), *route[i], temp) <= obs_dis/pixel_dis)

							{

								reachable = false;

							}

						}

						if(reachable==false) continue;

						nearnodes.push_back(route[i]);

					}

				}

				if(nearnodes.size()==0) continue;

				

				double min_cost=ys::get_distance(*nearnodes[0], temp) + nearnodes[0]->get_cost();

				int index_near = 0;

				for(int i=1; i<nearnodes.size(); i++)

				{

					if(ys::get_distance(*nearnodes[i], temp) + nearnodes[i]->get_cost() <= min_cost)

					{						

						min_cost = ys::get_distance(*nearnodes[i], temp)+ nearnodes[i]->get_cost();

						index_near = i;

					}

				}

				

				temp.parent = nearnodes[index_near];

				ys::node* temp2 = new ys::node;

				temp2->x = temp.x;

				temp2->y = temp.y;

				temp2->parent = temp.parent;

				temp2->cost = min_cost; 

				route.push_back(temp2);

				

				nearnodes.erase(nearnodes.begin(), nearnodes.end());

				for (int i=0; i<route.size(); i++)

				{

					if(ys::get_distance(*route[i], temp) <= rewire_dis/pixel_dis)

					{

						bool reachable = true;

						for(int j=0; j<obs.size() ; j++)

						{

							if(ys::P_Line_distance(ys::point(obs[j].x, obs[j].y), *route[i], temp) <= obs_dis/pixel_dis)

							{

								reachable = false;

							}

						}

						if(reachable==false) continue;

						nearnodes.push_back(route[i]);

					}

						

				}

				if(nearnodes.size()==0) continue;

				for(int i=1; i<nearnodes.size(); i++)

				{

					if(ys::get_distance(*nearnodes[i], temp) + temp.get_cost() < nearnodes[i]->get_cost())

					{

						nearnodes[i]->parent = temp2;

						nearnodes[i]->cost = ys::get_distance(*nearnodes[i], temp) + temp.get_cost();

					}

				}

				

		/*		ys::node* index = route[route.size()-1];

				while(1)

				{

					if(index->parent == NULL)

					{

						cout << index->x << ", " << index->y << endl;

					//	Sleep(50);

						break;

					}

					cout << index->x << ", " << index->y << ", " << index->parent->x << ", " << index->parent->y <<endl; 

					//Sleep(50);

					index = index->parent;

					if(index == NULL)

						break;

				}

		*/	//	cout <<route.size() <<"  "<< route[route.size()-1]->x <<"  "<< route[route.size()-1]->y<< endl;

				//cout <<temp_close.x<<temp_close.y<< endl;

				if(temp.x >= goalx - goal_dis/pixel_dis && temp.x <= goalx + goal_dis/pixel_dis)

				{

					if(temp.y >= goaly - goal_dis/pixel_dis && temp.y <= goaly + goal_dis/pixel_dis)

					{

						//reverse(route.begin(), route.end());

						vector<ys::point> result;

						ys::node* tmp = route[route.size()-1];

						while(1)

						{

							if(tmp->parent == NULL) break;

							result.push_back(ys::point(tmp->x, tmp->y));

							tmp = tmp->parent;

							

							

							

						}

						return result;

					}

				}

			}

			else

			{

				continue;

			}

		}

	}

}

 
/*
int main()

{

	ofstream f;

	f.open("test.txt");

	rrt_ys::rrt_star rrt;

	rrt.init(0.01, 0.1, 0.1, 0.1, 0.02, 0.2);

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

	for(int i=0; i<200; i++)

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

	

	vector<ys::point> route = rrt.rrtstar(map, 0,0,99,99);

	

	for(int i=0; i<route.size(); i++)

	{

		map[route[i].x][route[i].y] = 3;

	}

	

	

	for(int i=0; i<100; i++)

	{

		for(int j=0; j<100; j++)

		{

			f << map[i][j];

			if(map[i][j]==3)

				cout << ".";

			else

				cout << map[i][j];

		}

		f << endl;

		cout << endl;

	}

	return 0;

}
*/