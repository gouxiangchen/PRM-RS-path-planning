#ifndef Astar_H
#define Astar_H

#include <vector>
#include <cmath>
#include "PRM_RSS.h"

/*
// Astar class
*/

class Astar
{
public:
	bool getPath(p_node * start,p_node * goal);
	Astar(){};
	std::vector<p_node * > path;
protected:
	std::vector<g_node *> open;
	std::vector<g_node *> closed;
	int getManDistance(p_node * p1,p_node * p2)
	{
		using namespace std;
		return( abs(p1->pose.x - p2->pose.x)+abs(p1->pose.y - p2->pose.y) );
	}
	int getDistance(p_node * p1,p_node *p2)
	{
		using namespace std;
		return (int)sqrt(double((p1->pose.x-p2->pose.x)*(p1->pose.x-p2->pose.x)+(p1->pose.y-p2->pose.y)*(p1->pose.y-p2->pose.y)));
	}
	
};

#endif