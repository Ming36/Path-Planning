#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "data_structs.h"

using namespace std;

// GLOBAL DEFINITIONS
const float SPEED_LIMIT = 49;

const int SAFE_DISTANCE = 30;
const int CONTROL_SPEED_THOLD = 40;
const int SAFE_DISTANCE_REAR = 20;
const int PREDICTION_HORIZON = 20;
const int NUMBER_OF_LANES = 3;
const int KEEP_DISTANCE = 30;
const int LANE_BUFFER_SIZE = 25;
const double MAX_ACCEL = 2;
const double MAX_DELTA_V = 0.8;

class Vehicle{
	public:
		struct collider{
		    bool collision ; // is there a collision?
		    int  time; // time collision happens
  		};

	  	int id;
	  	int current_lane;
	  	double x;
	  	double y;
	  	double vx;
	  	double vy;
	  	double s;
	  	double d;
	  	double yaw;
	  	double speed;
			double accel;

  	/**
  * Constructor
  */
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

  /**
  * Destructor
  */
  ~Vehicle();

  vector<vector<double>>  Predict();

	vector<double> state_at(float t);

	vector<int> get_next_lane(vector<prediction> pred);

	bool _will_collide(vector<prediction> pred);

	int _get_safe_lane(vector<prediction> pred);

	double speed_controller(double dist, double velocity);
};

#endif
