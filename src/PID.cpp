#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	// Initialize errors to zero
	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;
	
}

void PID::UpdateError(double cte) {
	

	this->p_error = cte;
	this->i_error += cte;
	this->d_error = cte;
}

double PID::TotalError() {

	return -this->Kp * this->p_error - this->Ki * this->i_error;
	
}

