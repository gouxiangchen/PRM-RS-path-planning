#include "PRM_RSS.h"
#include <time.h>
#include <stdio.h>
#define MI 1

int main()
{
	PRM_RSS_map map("intel_binary.jpg",point(652,1158),point(498,312));
	map.getPath();
	map.showResult("PRMËæ»úÈöµã1.jpg");

	return 0;
}