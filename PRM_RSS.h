#ifndef PRM_RSS_H_H
#define PRM_RSS_H_H

#include "map_pgm.h"
#include <vector>
#include <opencv2/opencv.hpp>

struct point
{
	int x;
	int y;
	point(int x=0,int y=0){this->x=x;this->y=y;}
};



struct p_node
{
	point pose;
	std::vector<p_node *> neighbors;
	p_node(point p){this->pose=p;};
};


struct g_node
{
	p_node *  pose;
	int start_cost;
	int goal_cost;
	int cost;
	//std::vector<g_node *> neighbors; 
	g_node * father;
	g_node(p_node * p ,int cost_start,int cost_goal,g_node * fa=NULL)
	{
		this->pose = p;
		if (fa == NULL)
		{
			this->start_cost = 0;
		}
		else
		{
			start_cost =  cost_start;
		}
		goal_cost = cost_goal;
		cost = start_cost + goal_cost;
		this->father = fa;
	}
};



class PRM_RSS_map
{
public:
	PRM_RSS_map(const char * filename,point start,point goal);
	void showResult(const char * filename);
	bool getPath();
	std::vector<p_node *> allnode;
private:
	const int Rmax;
	const int Rmin;
	const int K;
	int getDistance(const point p1,const point p2);
	int sizeX;
	int sizeY;
	std::vector<point > line;
	std::vector<p_node *> path;
	bool if_line(const point p1,const point p2);
	const PRM_RSS_map operator=(const PRM_RSS_map &);
	Pgm_map map;
	//std::vector<tree_node *> graph;
	//std::vector<point> path;
	point start;
	point goal;

	int ** buff;
	cv::Mat img;
};

#endif



