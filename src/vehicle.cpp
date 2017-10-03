#include <iostream>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
//#include "data_structs.h"

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d){
	this->id = id;
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy = vy;
	this->s = s;
	this->d = d;
};

Vehicle::~Vehicle() {};

vector<double> Vehicle::state_at(float t){
	double s = this->s + this->speed*t + this->accel*t*t/2;
	double v = this->speed + this->accel*t;
	return {s, v};
};

vector<vector<double>> Vehicle::Predict(){

	/*
		Return a vector of predicted info for specific car
	*/

	vector<vector<double>> prediction;
	this->speed = sqrt(this->vx*this->vx + this->vy*this->vy);
	this->current_lane =  int(this->d / 4);

	for(int i = 1;i <= PREDICTION_HORIZON; i++){
		double pred_s = state_at(0.02*i)[0];
		double pred_v = state_at(0.02*i)[1];
		double pred_l = this->current_lane;
		//std::cout << "id:" << this->id << "\tpred:" << i << "\tpred_s:" \
		<< pred_s << "\tpred_v:" << pred_v << '\n';

		vector<double> pred = {pred_s, pred_l, pred_v};
		prediction.push_back(pred);
	}

	//std::cout << this->id << " " << this->current_lane << " " << this->x << " " << this->y << " " << this->s << " " << this->d << " " << this->vx << " " << this->vy << std::endl;
	return prediction;

};

vector<int> Vehicle::get_next_lane(vector<prediction> pred){
	vector<string> states = {"KL", "LCL", "LCR", "PLCL", "PLCR"};

	int ego_lane = int(this->d/4);
	if(ego_lane == 0){
		states.erase (states.begin()+1);
	}else if(ego_lane == 2){
		states.erase (states.begin()+2);
	}

	if (_will_collide(pred)){

		int safe_lane = _get_safe_lane(pred);

		if(safe_lane!=-1){
			std::cout << "Safe Lane:" << safe_lane << '\n';
			return {safe_lane, 0};
		}else{
			std::cout << "Stay at lane:" << ego_lane << '\n';
			return {ego_lane, 1};
		}
	}else{
		//std::cout << "Ego Lane:" << ego_lane << '\n';
		return {ego_lane, 0};
	}

};

bool Vehicle::_will_collide(vector<prediction> pred){
	// Determine if car will hit any other

	for(int i = 1; i < pred.size(); i++){
		for(int j = 0; j < pred[i].predictions.size(); j++){
			double dist_ego_car = pred[i].predictions[j][0] - pred[0].predictions[j][0];
			double car_lane = pred[i].predictions[j][1];
			if(dist_ego_car > 0 && dist_ego_car < SAFE_DISTANCE \
				&& this->current_lane == car_lane ){
				//std::cout << "Car: "  << i << " too close\t ego_lane: " << this->current_lane\
				<< "\tcar_lane:" << car_lane << std::endl;
				return true;
			}
		}
		}
	return false;
};

int Vehicle::_get_safe_lane(vector<prediction> pred){
	/* *******************************
			Return safe lane to shift to
	*  *******************************/
	int safe_lane;
	int ego_lane = int(this->d/4);
	vector<int> unsafe_lanes;

	for(int i = 1; i < pred.size(); i++){
		for(int j = 0; j < pred[i].predictions.size(); j++){
			double dist_ego_car = pred[i].predictions[j][0] - \
				pred[0].predictions[j][0];

			double car_lane = pred[i].predictions[j][1];
			if((abs(dist_ego_car) < SAFE_DISTANCE && dist_ego_car > 0) || \
					abs(dist_ego_car) < SAFE_DISTANCE_REAR && dist_ego_car < 0){
				//std::cout << "unsafe lane: " << car_lane << std::endl;
				unsafe_lanes.push_back(car_lane);
			}

		}
	}

	sort(unsafe_lanes.begin(), unsafe_lanes.end());
	unsafe_lanes.erase(unique(unsafe_lanes.begin(), unsafe_lanes.end()), unsafe_lanes.end());

	// return safe lane
	if (unsafe_lanes.size()>0){
		std::cout << "unsafe_lanes: ";
		for (int i = 0; i < unsafe_lanes.size(); i++){
			std::cout << unsafe_lanes[i] << " ";
		}
			std::cout << '\n';
		for(int i = 0; i<NUMBER_OF_LANES; i++){
      if(std::find(unsafe_lanes.begin(), unsafe_lanes.end(), i) != unsafe_lanes.end()) {
        /* v contains x */
      } else if(abs(ego_lane-i)<2){
        /* v does not contain x */
        return i;
      }
    }
  }

	return -1;

};

double Vehicle::speed_controller(double dist, double velocity){
	// v = sqrt(Vo^2 + 2*a*delta_s)
	// lets define a = 5 m/s^2

		double accel = MAX_ACCEL;
		// deslt_s > KEEP_DISTANCE
		double delta_s = dist - KEEP_DISTANCE;

		double vf = sqrt(velocity*velocity + 2*accel*delta_s);
		double delta_v = vf-velocity;

		if(abs(delta_v) < MAX_DELTA_V){
			return vf-velocity;
		}else if(delta_s < 0){
			return -MAX_DELTA_V;
		}else{
			return MAX_DELTA_V;
		}


};
