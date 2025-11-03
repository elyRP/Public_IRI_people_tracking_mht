/*
 * iri_geometry.cpp
 *
 *  Created on: 07/07/2013 (aprox)
 *      Author: Ely Repiso
 */
 // Copyright (C) from 2013-until now Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Ely Repiso
// All rights reserved.
/*
*
*      Created on: 2013 by Ely Repiso and published first as her TFC on 13/12/2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2- humble migration) 
*      Author: Ely Repiso (from 2013 and currently).
*      Furthermore, this code is a modification and extension extracted from the theory of the open source papers of Donald Reid IEEE Transaction on Automatic Control 1979 and Kai Oliver Arras ICRA2008. We never had their code, therefore, we implemented this code from scratch, only taking inspiration from their articles.
* 
*      License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
*      (Attribution-NonCommercial-#NoDerivatives 4.0 International)
*       https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
*
*      This license does not allow other authors to modify or to take profit from these works. 
*      Then, for #modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
*  collaborations (for #journals with other researchers, formal collaborations between UPC and companies, and so on).
*
* Please, to use it cite: Vaquero, Victor, et al. "Low cost, robust and real time system for detecting and tracking moving objects to automate cargo handling in port terminals." Robot 2015: Second Iberian Robotics Conference: Advances in Robotics, Volume 2. Cham: Springer International Publishing, 2015.
*  Redistribution and use in source and binary forms, without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*   * Additionally, remember that the derivatives without collaboration 
*     Of the original authors are prohibited
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
***********************************************************
*/
#include "iri_geometry.h"
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include<string>
#include <dirent.h>
#include <stdio.h>

// Spoint functions ------------------------------------------
Spoint::Spoint() : x(0) , y(0),  time_stamp(0) { }

Spoint::Spoint( double x_ , double y_ , double time_stamp_) :
		x(x_), y(y_), time_stamp(time_stamp_) { }
Spoint Spoint::operator+ (Spoint p2) const
{
	return Spoint(  x + p2.x , y + p2.y , time_stamp );
}
Spoint Spoint::operator- (Spoint p2) const
{
	return Spoint(  x - p2.x , y - p2.y , time_stamp);
}
Spoint Spoint::operator* (double k) const
{
	return Spoint(  x *k , y *k , time_stamp);
}
double Spoint::distance(Spoint p2) const
{
	return sqrt( distance2(p2)  );
}
double Spoint::distance2(Spoint p2) const
{
	//std::cout << "(x-p2.x) ="<< x-p2.x << "y-p2.y"<<y-p2.y<< std::endl;
	return (x-p2.x)*(x-p2.x) + (y-p2.y)*(y-p2.y);
}
Spoint Spoint::propagate( double dt , Sforce f, double desired_velocity) const
{
	//std::cout << "Spoint::propagate; dt="<<dt<< std::endl;
	return *this;
}
void Spoint::print() const
{
	std::cout << "(x , y ,t) = (" << x << " , " << y  << " | " << time_stamp << " )" << std::endl;
}

//Spoint_cov methods -----------------------------------------------------------
Spoint_cov::Spoint_cov():
		Spoint()
{
	cov.reserve(4);
	cov.resize(4,0.0);
	cov[0] = 0.2; //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 1
	cov[3] = 0.2;
}
Spoint_cov::Spoint_cov( double x_ , double y_ , double time_stamp_,
		const std::vector<double>& cov_ ):
		Spoint( x_,y_,time_stamp_ )
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 4 )
	{
		cov.reserve(4);
		cov.resize(4,0.0);
		cov[0] = 0.2; //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 1.0
		cov[3] = 0.2;
	}
	else
	{
		cov = cov_;
	}
}
double Spoint_cov::cov_dist( Spoint p2 ) const
{
	//analitical inversion of the 2x2 matrix
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	double cov_det = cov[0] * cov[3] - cov[1] * cov[2];
	double dx = x - p2.x;
	double dy = y - p2.y;
	//std::cout<<" (Prediction_distancia_Interna) cov_det= "<<cov_det<< std::endl;
	//std::cout<<" (Prediction_distancia_Interna) dx= "<<dx<< std::endl;
	//std::cout<<" (Prediction_distancia_Interna) dy= "<<dy<< std::endl;
	//std::cout<<" (Prediction_distancia_Interna) cov[0]= "<<cov[0]<<"; cov[1]="<<cov[1]<<"; cov[2]="<<cov[2]<<"; cov[3]="<<cov[3]<<"; cov_det="<<cov_det<< std::endl;
	//std::cout<<" (Prediction_distancia_Interna)=  "<<( dx*dx*cov[3] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det<< std::endl;
	return ( dx*dx*cov[3] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;
}
Spoint_cov Spoint_cov::operator+ (Spoint_cov p2) const
{
    return Spoint_cov(  x + p2.x , y + p2.y , time_stamp , cov);
}
Spoint_cov Spoint_cov::operator- (Spoint_cov p2) const
{
    return Spoint_cov(  x - p2.x , y - p2.y , time_stamp , cov);
}
Spoint_cov Spoint_cov::operator* (double k) const
{
    return Spoint_cov(  x*k , y*k , time_stamp , cov);
}
double Spoint_cov::cov_xx() const
{
	return cov[0];
}
double Spoint_cov::cov_yy() const
{
	return cov[3];
}
double Spoint_cov::cov_xy() const
{
	return cov[1];
}
void Spoint_cov::print() const
{
    Spoint::print();
    std::cout << "cov = [" << cov[0] << " , " << cov[1] << std::endl;
    std::cout << "       " << cov[2] << " , " << cov[3] << " ]" << std::endl;
}

//SpointV methods -----------------------------------------------------------
SpointV::SpointV() : Spoint(), vx(0), vy(0) { }

SpointV::SpointV( double x_ , double y_, double time_stamp_, double vx_, double vy_ ) :
		Spoint(x_,y_,time_stamp_), vx(vx_),vy(vy_) { }
SpointV SpointV::operator+ (SpointV p2) const
{
	return SpointV(  x + p2.x , y + p2.y , time_stamp,  vx + p2.vx , vy + p2.vy );
}
SpointV SpointV::operator- (SpointV p2) const
{
	return SpointV(  x - p2.x , y - p2.y , time_stamp-p2.time_stamp,  vx - p2.vx , vy - p2.vy );
}
SpointV SpointV::operator* ( double k) const
{
	return SpointV(  x * k , y * k, time_stamp,  vx * k , vy * k );
}
void SpointV::print() const
{
	std::cout << "(x , y ,t) = (" << x << " , " << y  << " | " << time_stamp << " )" <<
			"     (vx,vy) = ( " << vx << " , " << vy << " )" << std::endl;
}
SpointV SpointV::propagate( double dt , Sforce f, double desired_velocity) const
{
	//std::cout << "SpointV::propagate; dt="<<dt<< std::endl;
	//linear propagation (uniformly accelerated system)
	double vxn = vx + f.fx*dt;
	double vyn = vy + f.fy*dt;
	double dx = vx*dt + f.fx*dt*dt*0.5;
	double dy = vy*dt + f.fy*dt*dt*0.5;

	//std::cout << "f.fx=" << f.fx << "f.fy=" << f.fy  << "; dx=" <<dx << "; dy=" <<dy<<"; vx=" <<vx<<"; vy="<<vy<<";  f.fx*dt="<< f.fx*dt<<"; f.fy*dt="<<f.fy*dt<< std::endl;

	//apply constrains:
	double v = sqrt( vxn*vxn + vyn*vyn );
	if ( v > desired_velocity)
	{
		vxn *= desired_velocity /v;
		vyn *= desired_velocity /v;
		dx = vxn*dt;
		dy = vyn*dt;
		//std::cout << "( v > desired_velocity) "<< std::endl;

	}
	//std::cout << "( v > desired_velocity) vxn="<<vxn<<"; vyn"<<vyn<<"; x="<<x<<"; y="<<y<<"; x+dx="<<x+dx<<"; y+dy="<<y+dy<<"; time_stamp+dt="<<time_stamp+dt<<"; desired_velocity="<<desired_velocity<< std::endl;

	return SpointV( x+dx,y+dy, time_stamp+dt, vxn, vyn);
}

SpointV SpointV::propagate2_comp_only_vel(double dt , Sforce f, double desired_velocity, double distance_person_t_x,double distance_person_t_y,double vel_person_x,double vel_person_y) const
{
	//std::cout << "propagate2_comp_only_vel; dt="<<dt<< std::endl;
	/////////////////////////
	//linear propagation (uniformly accelerated system)
	double vxn = vx + f.fx*dt;
	double vyn = vy + f.fy*dt;
	double dx = vx*dt + f.fx*dt*dt*0.5;
	double dy = vy*dt + f.fy*dt*dt*0.5;

	//std::cout << "f.fx=" << f.fx << "f.fy=" << f.fy  << "; vxn=" <<vxn << "; vyn=" <<vyn<<"; vx=" <<vx<<"; vy="<<vy<<";  f.fx*dt="<< f.fx*dt<<"; f.fy*dt="<<f.fy*dt<< std::endl;

	//apply constrains:
	double v = sqrt( vxn*vxn + vyn*vyn );
	if ( v > desired_velocity)
	{
		vxn *= desired_velocity /v;
		vyn *= desired_velocity /v;
		dx = vxn*dt;
		dy = vyn*dt;
		//std::cout << "( v > desired_velocity) "<< std::endl;

	}
	//std::cout << "( v > desired_velocity) vxn="<<vxn<<"; vyn"<<vyn<<"; x="<<x+dx<<"; y="<<y+dy<<"; time_stamp+dt="<<time_stamp+dt<< std::endl;

	SpointV propagated_point=SpointV( x+dx,y+dy, time_stamp+dt, vxn, vyn);


	////////////////

	//double distance_person_t_x;
	//double distance_person_t_y;

	double distance_group_x;
	double distance_group_y;

	double vel_group_x=vx;
	double vel_group_y=vy;

	//double vel_person_x;
	//double vel_person_y;

	//std::cout << "propagated_point.x="<<propagated_point.x<<"; propagated_point.y="<<propagated_point.y<< std::endl;
	//std::cout << "x+distance_group_x="<<x+distance_group_x<<"; y+distance_group_y="<<y+distance_group_y<< std::endl;

	//std::cout << "x="<<x<<"; y="<<y<< std::endl;

	// caso 1
	//distance_group_x=sqrt((distance_person_t_x*(vel_group_x/vel_person_x))*(distance_person_t_x*(vel_group_x/vel_person_x)));
	//distance_group_y=sqrt((distance_person_t_y*(vel_group_y/vel_person_y))*(distance_person_t_y*(vel_group_y/vel_person_y)));

	distance_group_x=sqrt((distance_person_t_x*(vxn/vel_person_x))*(distance_person_t_x*(vxn/vel_person_x)));
	distance_group_y=sqrt((distance_person_t_y*(vyn/vel_person_y))*(distance_person_t_y*(vyn/vel_person_y)));

	double theta_act_robot_=atan(vy/vx);

	//std::cout << "theta_act_robot_="<<theta_act_robot_<<"; cos(theta_act_robot_)="<<cos(theta_act_robot_)<<"; sin(theta_act_robot_)="<<sin(theta_act_robot_)<< std::endl;


	double group_x=x+distance_group_x*cos(theta_act_robot_);
	double group_y=y+distance_group_y*sin(theta_act_robot_);

	double diference_x=sqrt((propagated_point.x-group_x)*(propagated_point.x-group_x)+(propagated_point.y-group_y)*(propagated_point.y-group_y));

	//std::cout << "diference_x="<<diference_x<< std::endl;
	//std::cout << "distance_person_t_x="<<distance_person_t_x<<"distance_person_t_y="<<distance_person_t_y<< std::endl;

	//std::cout << "f.fx=" << f.fx << "f.fy=" << f.fy  << std::endl;
	//std::cout << "distance_group_x="<<distance_group_x<<"distance_group_y="<<distance_group_x<< std::endl;
	//std::cout << "vel_group_x="<<vel_group_x<<"vel_group_y="<<vel_group_y<< "vel_person_x="<<vel_person_x<<"vel_person_y="<<vel_person_y<< std::endl;



	return SpointV( x+distance_group_x,y+distance_group_y,  time_stamp+dt, vxn, vyn);
}

double SpointV::orientation() const
{
	return atan2( vy, vx );
}

double SpointV::v() const
{
	return sqrt( vy*vy + vx*vx );
}

void SpointV::norm_v(double v)
{
	double theta = this->orientation();
	vx = cos(theta)*v;
	vy = sin(theta)*v;
}

double SpointV::angle_heading_point( Spoint p2 ) const
{
	double dx = p2.x - x;
	double dy = p2.y - y;
	return diffangle( atan2( dy , dx ), orientation() );
}

//SpointV_cov methods -----------------------------------------------------------
SpointV_cov::SpointV_cov() :
		SpointV()
{
	cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = 0.2;//0.4; //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 0.5
	cov[5] = 0.2;
	cov[10] = 0.1; //0.1
	cov[15] = 0.1;
}
SpointV_cov::SpointV_cov(double x_ , double y_, double time_stamp_, double vx_, double vy_,
		const std::vector<double>& cov_) :
		SpointV(x_,y_,time_stamp_,vx_,vy_)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 16 )
	{
		cov.reserve(16);
		cov.resize(16,0.0);
		cov[0] = 0.2; //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 0.5
		cov[5] = 0.2;
		cov[10] = 0.1;
		cov[15] = 0.1;
	}
	else
	{
		cov = cov_;
	}
}
SpointV_cov::SpointV_cov(SpointV p, const std::vector<double>& cov_) :
		SpointV( p.x , p.y, p.time_stamp, p.vx, p.vy)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 16 )
	{
		cov.reserve(16);
		cov.resize(16,0.0);
		cov[0] = 0.2; //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 0.5
		cov[5] = 0.2;
		cov[10] = 0.1;
		cov[15] = 0.1;
	}
	else
	{
		cov = cov_;
	}
}
SpointV_cov::SpointV_cov(Spoint_cov p):
		SpointV( p.x , p.y, p.time_stamp, 0.0, 0.0)
{
	cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = p.cov[0];
	cov[1] = p.cov[1];
	cov[4] = p.cov[2];
	cov[5] = p.cov[3];
	cov[10] = 0.1;
	cov[15] = 0.1;
}

/*void SpointV_cov::SpointV_cov_con_Spose(Spose p )
{
	SpointV( p.x , p.y, p.time_stamp, p.v*cos(p.theta), p.v*sin(p.theta));
	//SpointV_cov( p.x , p.y, p.time_stamp, p.vx, p.vy,p.cov)
	cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = 0.5;
	cov[5] = 0.5;
	cov[10] = 0.1;
	cov[15] = 0.1;
}*/


double SpointV_cov::cov_dist( Spoint p2 ) const
{
	// TODO: consider velocities, right now is a covariance distance of positions
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	double cov_det = cov[0] * cov[5] - cov[1] * cov[4];
	double dx = x - p2.x;
	double dy = y - p2.y;
	//
	return ( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;
}
double SpointV_cov::cov_dist( Spoint p2 , double &det) const
{
	det = cov[0] * cov[5] - cov[1] * cov[4];
	double dx = x - p2.x;
	double dy = y - p2.y;
	return ( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/det;
}

double SpointV_cov::cov_distV( SpointV_cov p2, double &distV, double &distxv ) const
{
	// TODO: consider velocities, right now is a covariance distance of positions
	//       det =   cov_xx * cov_yy - cov_xy * cov_xy
	// SpointV_cov external for actual track, SpointV_cov internal for the initial Spoint_cov for this track in the first time that the cross is made.
	double cov_det = cov[0] * cov[4] - cov[1] * cov[5];
	double dx = x - p2.x;
	double dy = y - p2.y;
	double distx=( dx*dx*cov[5] - 2*dx*dy*cov[1] + dy*dy*cov[0] )/cov_det;

	double cov_detV = cov[10] * cov[14] - cov[11] * cov[15];
	double dvx = vx - p2.vx;
	double dvy = vy - p2.vy;
	distV=( dvx*dvx*cov[15] - 2*dvx*dvy*cov[11] + dvy*dvy*cov[10] )/cov_detV;

	distxv=distx+distV;

	return distx;

}

SpointV_cov SpointV_cov::operator+ (SpointV_cov p2) const
{
    return SpointV_cov(  x + p2.x , y + p2.y , time_stamp , vx + p2.vx , vy + p2.vy ,cov);
}
SpointV_cov SpointV_cov::operator- (SpointV_cov p2) const
{
    return SpointV_cov(  x - p2.x , y - p2.y , time_stamp-p2.time_stamp,  vx - p2.vx , vy - p2.vy ,cov);
}
SpointV_cov SpointV_cov::operator* (double k) const
{
    return SpointV_cov(  x * k , y * k, time_stamp,  vx * k , vy * k , cov);
}
SpointV_cov SpointV_cov::propagate( double dt , Sforce f, double desired_velocity) const
{
	//std::cout << "SpointV_cov::propagate; dt="<<dt<< std::endl;

	SpointV new_point = SpointV::propagate(dt,f,desired_velocity);

	//std::cout << "; dt"<<dt<<"; f.fx="<<f.fx<<"; f.fy="<<f.fy<<"; desired_velocity="<<desired_velocity << std::endl;

	//dt=0.2;// provisional

	Eigen::MatrixXd cov_prev(4 , 4 );
	for (unsigned int i = 0; i < 4; ++i)
	{
		cov_prev.row(i) << cov[i*4],cov[i*4+1],cov[i*4+2],cov[i*4+3];
	}
	//std::cout << " (cov inicial) cov_ prev = \n" << std::endl;
	//std::cout << "[ cov_prev(0,0)=" << cov_prev(0,0) <<"; cov_prev(0,1)="<<cov_prev(0,1)<<"; cov_prev(0,2)="<<cov_prev(0,2)<<"; cov_prev(0,3)="<<cov_prev(0,3) <<"]" << std::endl;
	//std::cout << "[ cov_prev(1,0)=" << cov_prev(1,0) <<"; cov_prev(1,1)="<<cov_prev(1,1)<<"; cov_prev(1,2)="<<cov_prev(1,2)<<"; cov_prev(1,3)="<<cov_prev(1,3) <<"]" << std::endl;
	//std::cout << "[ cov_prev(2,0)=" << cov_prev(2,0) <<"; cov_prev(2,1)="<<cov_prev(2,1)<<"; cov_prev(2,2)="<<cov_prev(2,2)<<"; cov_prev(2,3)="<<cov_prev(2,3)<<"]" << std::endl;
	//std::cout << "[ cov_prev(3,0)=" << cov_prev(3,0) <<"; cov_prev(3,1)="<<cov_prev(3,1)<<"; cov_prev(3,2)="<<cov_prev(3,2)<<"; cov_prev(3,3)="<<cov_prev(3,3) <<"]" << std::endl;

	Eigen::MatrixXd Phi(4 , 4 );
	Phi = Eigen::MatrixXd::Identity(4,4);
	Phi(0,2) = dt;
	Phi(1,3) = dt;
	//std::cout << "[ Phi(0,0)=" << Phi(0,0) <<"; Phi(0,1)="<<Phi(0,1)<<"; Phi(0,2)="<<Phi(0,2)<<"; Phi(0,3)="<<Phi(0,3) <<"]" << std::endl;
	//std::cout << "[ Phi(1,0)=" << Phi(1,0) <<"; Phi(1,1)="<<Phi(1,1)<<"; Phi(1,2)="<<Phi(1,2)<<"; Phi(1,3)="<<Phi(1,3) <<"]" << std::endl;
	//std::cout << "[ Phi(2,0)=" << Phi(2,0) <<"; Phi(2,1)="<<Phi(2,1)<<"; Phi(2,2)="<<Phi(2,2)<<"; Phi(2,3)="<<Phi(1,3)<<"]" << std::endl;
	//std::cout << "[ Phi(3,0)=" << Phi(3,0) <<"; Phi(3,1)="<<Phi(3,1)<<"; Phi(3,2)="<<Phi(3,2)<<"; Phi(3,3)="<<Phi(3,3) <<"]" << std::endl;
	Eigen::MatrixXd G(4 , 2 );
	G = Eigen::MatrixXd::Zero(4,2);
	G(0,0) = dt*dt/2;
	G(1,1) = dt*dt/2;
	G(2,0) = dt;
	G(3,1) = dt;
	//std::cout << "[ G(0,0)=" << G(0,0) <<"; G(0,1)="<<G(0,1)<<"]" << std::endl;
	//std::cout << "[ G(1,0)=" << G(1,0) <<"; G(1,1)="<<G(1,1)<<"]" << std::endl;
	//std::cout << "[ G(2,0)=" << G(2,0) <<"; G(2,1)="<<G(2,1)<<"]" << std::endl;
	//std::cout << "[ G(3,0)=" << G(3,0) <<"; G(3,1)="<<G(3,1)<<"]" << std::endl;
	Eigen::MatrixXd new_cov(4,4);
	Eigen::MatrixXd cov_f(2,2);
	cov_f = Eigen::MatrixXd::Zero(2,2);
	cov_f(0,0) = 0.1;//TODO this covariance should be associated to the corresponding force and not generic. before was 0.3
	cov_f(1,1) = 0.1; //TODO: for the side_by_side 2 people. We need less covariance of prediction.

	//std::cout << "[ cov_f(0,0)=" << cov_f(0,0) <<"; cov_f(0,1)="<<cov_f(0,1)<<"]" << std::endl;
	//std::cout << "[ cov_f(1,0)=" << cov_f(1,0) <<"; cov_f(1,1)="<<cov_f(1,1)<<"]" << std::endl;

	new_cov = Phi*cov_prev*Phi.transpose() + G*cov_f*G.transpose();
	std::vector<double> new_cov_std(16,0.0);
	for(unsigned int i = 0; i<16; ++i)
	{
		new_cov_std[i] = new_cov( i/4 , i%4 );
	}

	//std::cout << "[ new_cov(0,0)=" << new_cov(0,0) <<"; new_cov(0,1)="<<new_cov(0,1)<<"; new_cov(0,2)="<<new_cov(0,2)<<"; new_cov(0,3)="<<new_cov(0,3) <<"]" << std::endl;
	//std::cout << "[ new_cov(1,0)=" << new_cov(1,0) <<"; new_cov(1,1)="<<new_cov(1,1)<<"; new_cov(1,2)="<<new_cov(1,2)<<"; new_cov(1,3)="<<new_cov(1,3) <<"]" << std::endl;
	//std::cout << "[ new_cov(2,0)=" << new_cov(2,0) <<"; new_cov(2,1)="<<new_cov(2,1)<<"; new_cov(2,2)="<<new_cov(2,2)<<"; new_cov(2,3)="<<new_cov(1,3)<<"]" << std::endl;
	//std::cout << "[ new_cov(3,0)=" << new_cov(3,0) <<"; new_cov(3,1)="<<new_cov(3,1)<<"; new_cov(3,2)="<<new_cov(3,2)<<"; new_cov(3,3)="<<new_cov(3,3) <<"]" << std::endl;
	//std::cout << "(SpointV) new_point =:" << std::endl;
	//new_point.print();

	return SpointV_cov( new_point, new_cov_std );
}
double SpointV_cov::cov_xx() const
{
	return cov[0];
}
double SpointV_cov::cov_yy() const
{
	return cov[5];
}
double SpointV_cov::cov_xy() const
{
	return cov[1];
}
Spoint_cov SpointV_cov::toSpoint_cov() const
{
	std::vector<double> cov_point;
	cov_point.reserve(4);
	cov_point.resize(4,0.0);
	cov_point[0] = cov[0];
	cov_point[1] = cov[1];
	cov_point[2] = cov[4];
	cov_point[3] = cov[5];
	return Spoint_cov( x,y,time_stamp, cov_point );
}
void SpointV_cov::print() const
{
    SpointV::print();
    std::cout << "covariance = [ " << cov[0] << " , " << cov[1] << " , " <<cov[2]  << " , " << cov[3] << std::endl <<
    		     "               " << cov[4] << " , " << cov[5] << " , " <<cov[6]  << " , " << cov[7] << std::endl <<
    		     "               " << cov[8] << " , " << cov[9] << " , " <<cov[10]  << " , " << cov[11] << std::endl <<
    		     "               " << cov[12] << " , " << cov[13] << " , " <<cov[14]  << " , " << cov[15] << std::endl;
}

//Spose methods -----------------------------------------------------------
Spose::Spose() : Spoint(), theta(0)  , v(0) , w(0)  { }
Spose::Spose( double x_ , double y_ , double time_stamp_ , double theta_ , double v_, double w_) :
	Spoint(x_,y_, time_stamp_), theta(theta_), v(v_), w(w_) { }
Spose Spose::operator+ (Spose p2) const
{
	return Spose(  x + p2.x , y + p2.y , theta + p2.theta, p2.time_stamp , p2.v , p2.w );
}
Spose Spose::operator- (Spose p2) const
{
	return Spose(  x - p2.x , y - p2.y , theta - p2.theta, p2.time_stamp , p2.v , p2.w );
}
Spose Spose::operator* ( double k) const
{
	return Spose(  x * k , y *k , theta, time_stamp , v , w );
}
double Spose::distance(Spoint p2) const
{
	return sqrt(  (x-p2.x)*(x-p2.x) + (y-p2.y)*(y-p2.y) );
}
double Spose::angle_heading_pose( Spose p2 ) const
{
	double dx = p2.x - x;
	double dy = p2.y - y;
	return diffangle( atan2( dy , dx ), theta );
}
double Spose::social_distance( Spose p2 ) const
{
	double lambda = 0.0;
	double phi = this->angle_heading_pose( p2 );
	double anisotropy = (lambda + (1-lambda)*(1 + cos(phi))/2 );
	return this->distance(p2)/anisotropy;
}
void Spose::print() const
{
	std::cout << "(x , y | theta) = (" << x << " , " << y  << " | " << theta <<
			" )  --  at t = " << time_stamp << std::endl <<
			"(v , w) = (" << v  << " , " << w << " )" << std::endl;
}

//Spose_cov methods -----------------------------------------------------------
Spose_cov::Spose_cov():	 Spose()
{
	cov.reserve(25);
	cov.resize(25,0.0);
	cov[0] = 0.2;//xx //TODO: for the side_by_side 2 people. We need less covariance of prediction.
	cov[6] = 0.2;//yy
	cov[12] = 0.05;//theta_theta
	cov[18] = 0.1;//cov_vv
	cov[24] = 0.1;//ww
}
Spose_cov::Spose_cov(double x_ , double y_ ,
		double time_stamp_ , double theta_ ,  double v_, double w_,
		const std::vector<double>& cov_ ):
		Spose( x_,y_,time_stamp_, theta_,v_,w_ )
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 25 )
	{
		cov.reserve(25);
		cov.resize(25,0.0);
		cov[0] = 0.2;//xx //TODO: for the side_by_side 2 people. We need less covariance of prediction. before was 0.5
		cov[6] = 0.2;//yy
		cov[12] = 0.05;//theta_theta
		cov[18] = 0.1;//cov_vv
		cov[24] = 0.1;//ww
	}
	else
	{
		cov = cov_;
	}
}
Spose_cov::Spose_cov( Spose pose_, const std::vector<double>& cov_ ):
		Spose(pose_)
{
	if ( cov_.empty() || cov_.size() < (std::size_t) 25 )
	{
		cov.reserve(25);
		cov.resize(25,0.0);
		cov[0] = 0.2;//xx //TODO: for the side_by_side 2 people. We need less covariance of prediction.
		cov[6] = 0.2;//yy
		cov[12] = 0.05;//theta_theta
		cov[18] = 0.1;//cov_vv
		cov[24] = 0.1;//ww
	}
	else
	{
		cov = cov_;
	}
}
Spose_cov::Spose_cov( SpointV_cov point ):
		Spose(point.x,point.y,point.time_stamp,point.orientation(),point.v())
{
	cov.reserve(25);
	cov.resize(25,0.0);
	cov[0] = point.cov[0];//xx
	cov[6] = point.cov[5];//yy
	cov[12] = 0.05;//theta_theta
	cov[18] = 0.1;//cov_vv
	cov[24] = 0.1;//ww
}
double Spose_cov::distance( Spose_cov p2) const
{
	return Spose::distance( Spose(p2.x,p2.y) );
}
double Spose_cov::cov_dist( Spose p2 ) const
{
	//analitical inversion of the 2x2 matrix
	double cov_xx = cov[0];
	double cov_yy = cov[6];
	double cov_xy = cov[5];
	double cov_det = cov_xx * cov_yy - cov_xy * cov_xy;
	double dx = x - p2.x;
	double dy = y - p2.y;
	return ( dx*dx*cov_yy - 2*dx*dy*cov_xy + dy*dy*cov_xx )/cov_det;
}
double Spose_cov::cov_dist( Spose_cov p2 ) const
{
	return this->cov_dist( Spose(p2.x,p2.y) );
}

//Sdestinations methods -----------------------------------------------------------
Sdestination::Sdestination( int id_, double x_ , double y_ , double prob_ ,
		Sdestination::destination_type type_, std::vector<int> neighbours_ids_ ) :
	Spoint(x_,y_), id(id_), prob(prob_) ,
	type(type_), neighbours_ids(neighbours_ids_)  { }
/*Sdestination::Sdestination( Sdestination dest ) :
	Spoint(dest.x,dest.y), id(dest.id), prob(dest.prob) ,
	type(dest.type), neighbours_ids(dest.neighbours_ids)  { }*/
Sdestination::~Sdestination(){}
bool Sdestination::operator== (Sdestination d2) const
{
	if ( x == d2.x && y == d2.y )
		return true;
	else
		return false;
}

void Sdestination::print() const
{
	std::cout << "destination " << id << "at (" << x << " , " << y  <<  ")  of type " << type  <<
		"   and probability = " << prob  << "neighbours " << neighbours_ids.size() << std::endl;
}


SdetectionObservation::SdetectionObservation() :
				SpointV_cov(), id(0) { }
SdetectionObservation::SdetectionObservation( int id_ , double time_stamp_ ,
		double x_, double y_, double vx_, double vy_, const std::vector<double>& cov_) :
			SpointV_cov(x_,y_,time_stamp_,vx_,vy_,cov_) , id(id_) { }
SdetectionObservation::SdetectionObservation( int id_ , SpointV_cov pointV_cov_) :
	SpointV_cov( pointV_cov_ ), id(id_) {}
void SdetectionObservation::print() const
{
	std::cout <<  "detection Observation = " << id << " th "  << std::endl;
	SpointV_cov::print();
}


//Sforce methods
Sforce::Sforce() : fx(0.0) , fy(0.0) {}
Sforce::Sforce(double fx_ , double fy_) : fx(fx_) , fy(fy_) {}
double Sforce::module() { return sqrt(fx*fx + fy*fy);}
double Sforce::module(double r2) { return sqrt(fx*fx + r2*fy*fy);}
double Sforce::module2( ) { return fx*fx + fy*fy;}
double Sforce::module2( double r2) { return fx*fx + r2*fy*fy;}
void Sforce::print() const
{
	std::cout <<  "Force = (" << fx << " , " << fy  << " )" << std::endl;
}
Sforce Sforce::operator+ (Sforce f2) const
{
	return Sforce(  fx + f2.fx , fy + f2.fy );
}
void Sforce::sum(Sforce f2)
//a method similar to +=
{
	fx += f2.fx;
	fy += f2.fy;
}
Sforce& Sforce::operator +=(Sforce f2)
{
	fx += f2.fx;
	fy += f2.fy;
	return *this;
}

Sforce Sforce::operator* (double k) const
{
	return Sforce(  fx *k , fy *k );
}

double Sforce::operator* (Spoint dr) const
{
	return this->fx * dr.x + this->fy * dr.y;
}
Sforce Sforce::operator- (Sforce f2) const
{
    return Sforce(  fx - f2.fx , fy - f2.fy );
}

double diffangle(double alpha , double beta)
{
	  double delta = alpha - beta;
	  if ( alpha >= beta )
	  {
	    while (delta >  PI)
	      delta -= 2*PI;
	  }
	  else
	  {
		  while (delta < -PI )
			  delta += 2*PI;
	  }
	  return delta;
}


SsavePath_cost_and_values::SsavePath_cost_and_values(int in_end_id_path, std::vector<Spoint> in_path_positions, std::vector<unsigned int> in_all_ids_path_positions, std::vector<std::vector<double>> in_total_costs_of_each_path_position) {
	end_id_path=in_end_id_path;
	path_positions=in_path_positions;
	all_ids_path_positions=in_all_ids_path_positions;
	total_costs_of_each_path_position=in_total_costs_of_each_path_position;
}

void SsavePath_cost_and_values::print() const
{
	std::cout <<  " end_id_path="<<end_id_path << std::endl;
	for(unsigned int i=0; i<path_positions.size();i++){
		std::cout <<  " all_ids_path_positions.id="<<all_ids_path_positions[i] << std::endl;
		path_positions[i].print();
		for(unsigned int j=0; j<total_costs_of_each_path_position[i].size();j++){
			std::cout <<  " toal_costs_of_each_path_position[id]["<<j<<"]="<<total_costs_of_each_path_position[i][j] << std::endl;
		}
	}

}

SsavePath_multiple_paths_and_best_path::SsavePath_multiple_paths_and_best_path(double in_time_act, double in_iter_act, int in_number_of_total_paths,std::vector<SsavePath_cost_and_values> in_nondominated_paths, SsavePath_cost_and_values in_best_path,std::vector<double> in_means,std::vector<double> in_stds){

	time_act=in_time_act;
	iter_act=in_iter_act;
	number_of_total_paths=in_number_of_total_paths;
	nondominated_paths=in_nondominated_paths;
	best_path=in_best_path;
	means=in_means;
	stds=in_stds;

}


void SsavePath_multiple_paths_and_best_path::print() const
{
	std::cout <<  " time_act="<<time_act << std::endl;
	std::cout <<  " iter_act="<<iter_act << std::endl;
	std::cout <<  " number_of_total_paths="<<number_of_total_paths << std::endl;
	std::cout <<  " print nondominated_paths: nondominated_paths.size()="<<nondominated_paths.size() << std::endl;

	for(unsigned int i=0; i<nondominated_paths.size();i++){
		nondominated_paths[i].print();
	}

	std::cout <<  " print best_paths:" << std::endl;
	best_path.print();

	std::cout <<  " print means: mean_distance_[0]; mean_orientation_[1]; mean_robot_[2]; mean_int_forces_[3]; mean_obstacles_[4]; mean_companion_[5]; ant_traj[6]; " << std::endl;
	for(unsigned int i=0; i<means.size();i++){
		std::cout <<  "means["<<i<<"]"<<means[i]<< std::endl;
	}

	for(unsigned int i=0; i<stds.size();i++){
		std::cout <<  "stds["<<i<<"]"<<stds[i]<< std::endl;
	}
}



/* Convert a decimal integer to a c-style string */
char* Spoint::itoa(int a, char b[])
{
   int NumofDigits = getNumofDecDigit(a);
    if(a>0)
    {
    switch(NumofDigits)
    {
        case 1:
        b[0]=getStrChar(a);
        b[1]='\0';
        break;

        case 2:
        b[0]= getStrChar(a/10);
        b[1]= getStrChar(a%10);
        b[2]='\0';
        break;

        case 3:
        b[0]= getStrChar(a/100);
        b[1]= getStrChar((a/10)%10);
        b[2]= getStrChar(a%10);
        b[3]='\0';
        break;

        case 4:
        b[0]= getStrChar(a/1000);
        b[1]= getStrChar((a/100)%10);
        b[2]= getStrChar((a/10)%10);
        b[3]= getStrChar(a%10);
        b[4]='\0';
        break;

        case 5:
        b[0]= getStrChar(a/10000);
        b[1]= getStrChar((a/1000)%10);
        b[2]= getStrChar((a/100)%10);
        b[3]= getStrChar((a/10)%10);
        b[4]= getStrChar(a%10);
        b[5]='\0';
        break;

        case 6:
        b[0]= getStrChar(a/100000);
        b[1]= getStrChar((a/10000)%10);
        b[2]= getStrChar((a/1000)%10);
        b[3]= getStrChar((a/100)%10);
        b[4]= getStrChar((a/10)%10);
        b[5]= getStrChar(a%10);
        b[6]='\0';
        break;

        case 7:
        b[0]= getStrChar(a/1000000);
        b[1]= getStrChar((a/100000)%10);
        b[2]= getStrChar((a/10000)%10);
        b[3]= getStrChar((a/1000)%10);
        b[4]= getStrChar((a/100)%10);
        b[5]= getStrChar((a/10)%10);
        b[6]= getStrChar(a%10);
        b[6]='\0';
        break;

        case 8:
        b[0]= getStrChar(a/10000000);
        b[1]= getStrChar((a/1000000)%10);
        b[2]= getStrChar((a/100000)%10);
        b[3]= getStrChar((a/10000)%10);
        b[4]= getStrChar((a/1000)%10);
        b[5]= getStrChar((a/100)%10);
        b[6]= getStrChar((a/10)%10);
        b[7]= getStrChar(a%10);
        b[8]='\0';
        break;

        default:
        b[0]='\0';
    }
    }
    else
    {
    switch(NumofDigits)
    {
        case 1:
        b[0]='-';
        b[1]=getStrChar(a);
        b[2]='\0';
        break;

        case 2:
        b[0]='-';
        b[1]= getStrChar(a/10);
        b[2]= getStrChar(a%10);
        b[3]='\0';
        break;

        case 3:
        b[0]='-';
        b[1]= getStrChar(a/100);
        b[2]= getStrChar((a/10)%10);
        b[3]= getStrChar(a%10);
        b[4]='\0';
        break;

        case 4:
        b[0]='-';
        b[1]= getStrChar(a/1000);
        b[2]= getStrChar((a/100)%10);
        b[3]= getStrChar((a/10)%10);
        b[4]= getStrChar(a%10);
        b[5]='\0';
        break;

        case 5:
        b[0]='-';
        b[1]= getStrChar(a/10000);
        b[2]= getStrChar((a/1000)%10);
        b[3]= getStrChar((a/100)%10);
        b[4]= getStrChar((a/10)%10);
        b[5]= getStrChar(a%10);
        b[6]='\0';
        break;

        case 6:
        b[0]='-';
        b[1]= getStrChar(a/100000);
        b[2]= getStrChar((a/10000)%10);
        b[3]= getStrChar((a/1000)%10);
        b[4]= getStrChar((a/100)%10);
        b[5]= getStrChar((a/10)%10);
        b[6]= getStrChar(a%10);
        b[7]='\0';
        break;

        case 7:
        b[0]='-';
        b[1]= getStrChar(a/1000000);
        b[2]= getStrChar((a/100000)%10);
        b[3]= getStrChar((a/10000)%10);
        b[4]= getStrChar((a/1000)%10);
        b[5]= getStrChar((a/100)%10);
        b[6]= getStrChar((a/10)%10);
        b[7]= getStrChar(a%10);
        b[8]='\0';
        break;

        case 8:
        b[0]='-';
        b[1]= getStrChar(a/10000000);
        b[2]= getStrChar((a/1000000)%10);
        b[3]= getStrChar((a/100000)%10);
        b[4]= getStrChar((a/10000)%10);
        b[5]= getStrChar((a/1000)%10);
        b[6]= getStrChar((a/100)%10);
        b[7]= getStrChar((a/10)%10);
        b[8]= getStrChar(a%10);
        b[9]='\0';
        break;

        default:
        b[0]='\0';
        break;
    }

    }
    if (a==0)
    {
        b[0]= '0';
        b[1]='\0';
    }

  return b;
}

/* Get number of digits in a positive or negative integer */
int Spoint::getNumofDecDigit(int Number)
{
   int NumofDigits=0;
    if(Number < 0)
      {
          Number=-Number;
      }
      while(Number)
      {
          Number/=10;
          NumofDigits++;
      }
    return NumofDigits;
}

/* Get a character from  digit input number */
char Spoint::getStrChar(int digit)
{
   char  StrChar = ' ';
    switch (digit)
    {
        case 0:
        StrChar = '0';
        break;

        case 1:
        case -1:
        StrChar ='1';
        break;

        case 2:
        case -2:
        StrChar = '2';
        break;

        case 3:
        case -3:
        StrChar = '3';
        break;

        case 4:
        case -4:
        StrChar = '4';
        break;

        case 5:
        case -5:
        StrChar = '5';
        break;

        case 6:
        case -6:
        StrChar = '6';
        break;

        case 7:
        case -7:
        StrChar = '7';
        break;

        case 8:
        case -8:
        StrChar = '8';
        break;

        case 9:
        case -9:
        StrChar = '9';
        break;

        default:
        StrChar ='0';
        break;
    }
    return StrChar;
}


std::string get_path_learning::addPath(std::string a, std::string c){
    std::string b = "/";
    return a+b+c;
}

std::vector<std::string> get_path_learning::folderStream(std::string PATH){
    std::vector<std::string> directories;
    DIR *dir = opendir(PATH.c_str());
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
        if (entry->d_type == DT_DIR){
            std::string aux = entry->d_name;
            if(aux.compare(".") != 0 and aux.compare("..")!=0){
                directories.push_back(entry->d_name);
            }
        }
        entry = readdir(dir);
    }

    closedir(dir);

    return directories;
}

void get_path_learning::pathGeneratoraux(std::string path, int level, std::vector<std::string> &paths){
	std::cout<<"pathGeneratoraux 1"<<std::endl;
    std::vector<std::string> v = folderStream(path);
    std::cout<<"pathGeneratoraux 2"<<std::endl;
    if(level == 0){
    	std::cout<<"pathGeneratoraux 3"<<std::endl;
        for(int i = 0; i<v.size();i++){
            paths.push_back(addPath(path,v[i]));
        }
    }
    else{
        for(int i = 0; i<v.size();i++){
            pathGeneratoraux(addPath(path,v[i]),level-1,paths);
        }
    }
}

std::vector<std::string> get_path_learning::pathGenerator(std::string path){

	 std::cout<<"pathGenerator 1"<<std::endl;
    std::vector<std::string> paths;
    std::cout<<"pathGenerator 2"<<std::endl;
    pathGeneratoraux(path,1,paths);
    std::cout<<"pathGenerator 3"<<std::endl;
    return paths;
}

/*
int main(int argc,char *argv[]){
    std::string path = "/home/aserra/Documents/IRI/Database_creation/Experiments";
    std::vector<std::string> paths = pathGenerator(path);
    for(int i = 0; i<paths.size(); i++){
        std::cout<<paths[i]<<std::endl;
    }
}*/


peopleOtherPersonCompanionInGroup::peopleOtherPersonCompanionInGroup( unsigned int in_real_person_id , unsigned int in_actual_new_id_in_people_simulation){
	real_person_id_=in_real_person_id;
	actual_new_id_in_people_simulation_=in_actual_new_id_in_people_simulation;
}

void peopleOtherPersonCompanionInGroup::print() const
{
	 std::cout<<"real_person_id_="<<real_person_id_<<std::endl;
	 std::cout<<"actual_new_id_in_people_simulation_="<<actual_new_id_in_people_simulation_<<std::endl;
}
