#ifndef DATA_STRUCTS_H
#define DATA_STRUCT_H

#include<iostream>
#include<vector>

using namespace std;

/* 	*************************
	Car sensor fusion struct
	************************
*/

struct car_sf
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
};

struct prediction{
	int id;
	vector<vector<double>> predictions;
};

#endif
