/*
 * total_velocity.h
 *
 *  Created on: 14/02/2015
 *      Author: erepiso
 */

#include <eigen3/Eigen/Dense> //Library for matrix. Being here, not needed in cpp. (Al estar aquí, no hacen falta en el cpp).
#include <iostream> //Library for screen messages.
#include <math.h> //library for sen cos sqrt etc.
#include <vector>

class Cvelocity
{
public:
	Cvelocity();
	~Cvelocity();

	void update_velocity(double px, double py, double oz,double total_time);
	void print_velocity();

	double get_total_px(){ return total_px_; };
	double get_total_py(){ return total_py_; };
	double get_total_oz(){ return total_oz_; };
	double get_total_time(){ return total_time_; };

private:

	bool cargo_ants;
	double total_px_; // [m/s]
	double total_py_; // [m/s]
	double total_oz_; // [rad]
	double total_time_;

};

