#ifndef MAP_PGM_H_H
#define MAP_PGM_H_H

#include <iostream>
#include <opencv2/opencv.hpp>


class Pgm_map
{
public:
	int ** getGridMap2D();
	Pgm_map(const char * filename);
	bool transformMapToGrid();
	const int getSizeX();
	const int getSizeY();
	cv::Mat getImg();
	~Pgm_map();
private:
	bool isopen;
	cv::Mat	basemap;
	int ** gridmap2D;
	int sizeX;
	int sizeY;
	const Pgm_map & operator=(const Pgm_map &);	//must exist but not use
};


#endif