/*
 * total_velocity.cpp
 *
 *  Created on: 14/02/2015
 *      Author: erepiso
 */

#include "total_velocity.h"

Cvelocity::Cvelocity():
cargo_ants(false),
total_px_(0.0),total_py_(0.0),total_oz_(0.0)
{
}

Cvelocity::~Cvelocity()
{
}

void Cvelocity::update_velocity(double px, double py, double oz, double total_time){

//dt_=time_odom_act_-time_odom_ant_;
	total_time_=total_time;
	total_px_=px;
	total_py_=py;
	total_oz_=oz;

}

void Cvelocity::print_velocity(){
	std::cout<< " Actual velocity \n" << std::endl;
	std::cout<< " px_" <<  total_px_ << std::endl;
	std::cout<< " py_:" << total_py_ << std::endl;
	std::cout<< " oz_:" << total_oz_ << std::endl;
}

