/*
 * person_bhmip.cpp
 *
 *  Created on: Jul 9, 2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration).
 *      Author: gferrer
 *      Author: Ely Repiso (Modifications for people tracking mht)
 *
 *      license (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *      This license does not allow other authors to modify or to take profit from these works. 
 *      Then, for modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
 *   collaborations (for journals with other researchers, formal collaborations between UPC and companies, and so on).
 *
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
 */

#include "scene_elements/person_bhmip.h"
#include <Eigen/Dense>
#include <iostream>

Cperson_bhmip::Cperson_bhmip(unsigned int id, Cperson_abstract::target_type person_target_type,
		 Cperson_abstract::force_type person_force_type, double _time_window) :
    Cperson_abstract(id,person_target_type, person_force_type),
    time_window_( _time_window ) , phi_var_( 0.8*0.8 ), debug_filter_person_velocity_(false),
	debug_add_pointV_(false),
	debug_Cperson_bhmip_add_pointV_(false),
	debug_trajectory_windowing_(false),
	debug_refresh_person_(false),
	debug_prediction_(false),
	debug_intention_precalculation_(false),
	debug_filter_current_state_linear_regression_(false),
	person_before_moving_(false)
{
	destinations_.reserve(15);
}

Cperson_bhmip::~Cperson_bhmip()
{

}

void Cperson_bhmip::add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter, bool robot_or_person, double in_new_value_windowing)
{

	time_window_=in_new_value_windowing;

	if(debug_add_pointV_){
		std::cout<<" (Cperson_bhmip) entro en add_pointV!"<< std::endl;
	}

	SpointV_cov filtered_pose;
	if( filter != Cperson_abstract::No_filtering )
	{
		if(debug_add_pointV_){
			std::cout<<" (Cperson_bhmip) != Cperson_abstract::No_filtering; filter="<<filter<< std::endl;
		}
		SpointV_cov pointV = point;//only positions, position_covariances and time_stamp

		//std::cout<<" pointV.time="<<pointV.time_stamp<<"; point.time_stamp="<<point.time_stamp<< std::endl;

		double dx, dy;
		if ( !trajectory_.empty() )
		{
			if(debug_add_pointV_){
				std::cout<<" (Cperson_bhmip) if !trajectory_.empty()"<< std::endl;
			}

			dx = point.x - trajectory_.back().x;
			dy = point.y - trajectory_.back().y;
			double dt =  point.time_stamp - trajectory_.back().time_stamp ;
			//just to avoid singularities, ideally time stamps are well separated in time
			double marge=0.001;  // marge=0.001;=> anterior valor, que seguía creando singularidades.
			if ( point.time_stamp - trajectory_.back().time_stamp  < marge ) dt = 0.1;

			pointV.vx = dx / dt;
			pointV.vy = dy / dt;

			if(sqrt(pow(pointV.vx,2))>2){
				pointV.vx=trajectory_.back().vx;
			}
			if(sqrt(pow(pointV.vy,2))>2){
				pointV.vy=trajectory_.back().vy;
			}

			if(debug_filter_person_velocity_){
				std::cout<<" (ANT) dt="<<dt<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<<"; time_window_="<<time_window_<< std::endl;
				std::cout<<" (INI FILTRO V) point.time_stamp="<< point.time_stamp<<"; trajectory_.back().time_stamp= " <<trajectory_.back().time_stamp<< std::endl;
				std::cout<<"dx="<< dx<<"; dy= " <<dy<<"; point.x="<< point.x<<"; trajectory_.back().x="<<trajectory_.back().x<<"; trajectory_.back().y="<<trajectory_.back().y<<"; point.y= " <<point.y<< std::endl;
				std::cout<<"; point.vx="<< point.vx<<"; point.vy= " <<point.vy<< std::endl;
				std::cout<<"trajectory_.back().vx="<< trajectory_.back().vx<<"; trajectory_.back().vy= " <<trajectory_.back().vy<< std::endl;
				std::cout<<" dt="<<dt<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<< std::endl;
			}
		}
		else
		{
			if(debug_add_pointV_){
				std::cout<<" ==trajectory_.empty() "<< std::endl;
			}
			if(robot_or_person){
				if(debug_add_pointV_){
					std::cout<<" ant velocity "<< std::endl;
				}
				pointV.vx=point.vx;
				pointV.vy=point.vy;
			}else{
				if(debug_add_pointV_){
					std::cout<<" set 0 velocity "<< std::endl;
				}
				pointV.vx = 0;
				pointV.vy = 0;
			}
		}

		//std::cout<<" before TRAJECTORY WINDOWING"<< std::endl;
		//slide temporal window for all trajectories
		trajectory_.push_back(pointV);
		//std::cout<<" after push_bach"<< std::endl;
		trajectory_windowing();
		//std::cout<<" AFTER TRAJECTORY WINDOWING"<< std::endl;

		if(debug_filter_person_velocity_){
			std::cout<<" AFTER TRAJECTORY WINDOWING"<< std::endl;
			std::cout<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<<"; time_window_="<<time_window_<< std::endl;

			std::cout<<" (INI FILTRO V) point.time_stamp="<< point.time_stamp<<"; trajectory_.back().time_stamp= " <<trajectory_.back().time_stamp<< std::endl;
			std::cout<<"dx="<< dx<<"; dy= " <<dy<<"; point.x="<< point.x<<"; trajectory_.back().x="<<trajectory_.back().x<<"; trajectory_.back().y="<<trajectory_.back().y<<"; point.y= " <<point.y<< std::endl;
			std::cout<<"; point.vx="<< point.vx<<"; point.vy= " <<point.vy<< std::endl;
			std::cout<<"trajectory_.back().vx="<< trajectory_.back().vx<<"; trajectory_.back().vy= " <<trajectory_.back().vy<< std::endl;
			std::cout<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<< std::endl;
		}

		if(debug_add_pointV_){
			std::cout<<" trajectory_.size() "<<trajectory_.size()<< std::endl;
		}

		//std::cout<<" before filter"<< std::endl;
		//filters velocities
		switch( filter )
		{
		  case Cperson_abstract::Linear_regression_filtering:
			  if(debug_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Linear_regression_filtering)" << std::endl;
			  }
			filtered_pose = filter_current_state_linear_regression( );
			break;
		  case Cperson_abstract::Bayes_filtering:
			  if(debug_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Bayes_filtering)" << std::endl;
			  }
			filtered_pose = filter_current_state_linear_regression_bayes( );
		  	break;
		  case Cperson_abstract::Low_pass_linear_regression_filtering:
		  default :
			  if(debug_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Low_pass_linear_regression_filtering)" << std::endl;
			  }
			filtered_pose = low_pass_filter_current_state_linear_regression( );
			break;
		}

	}
	else
	{
		if(debug_add_pointV_){
			std::cout << " (Caso: no filter is required!); trajectory_.size()="<<trajectory_.size() << std::endl;
		}

		//no filter is required
		trajectory_.push_back(point);
		//slide temporal window for all trajectories
		trajectory_windowing();
		filtered_pose = trajectory_.back();
	}

	//std::cout<<" AFTER first if-else"<< std::endl;

	//update current positions and velocities according to filtering results
	diff_pointV_ = filtered_pose - current_pointV_ ;
	//std::cout << "(for tracker update_scene) filtered_pose.x="<<filtered_pose.x<<"; filtered_pose.y="<<filtered_pose.y<< std::endl;
	current_pointV_ = filtered_pose;
	//std::cout << "(for tracker update_scene) current_pointV_.x="<<current_pointV_.x<<"; current_pointV_.y="<<current_pointV_.y<< std::endl;
	//std::cout << "(ANTES FILTRO) desired_velocity_="<<desired_velocity_<< std::endl;
	desired_velocity_ = filtered_pose.v();
	if(debug_filter_person_velocity_){
		std::cout << "(DESPUES FILTRO==PERSONA) desired_velocity_="<<desired_velocity_<< std::endl;
	}
	now_ = filtered_pose.time_stamp;


	//precalculates data required for the BHMIP
	intention_precalculation();

	observation_update_ = true;

	if(debug_add_pointV_){
		std::cout<<" trajectory_.size() "<<trajectory_.size()<< std::endl;
		std::cout<<" (Cperson_bhmip) salgo de add_pointV! observation_update_="<<observation_update_<<": filtered_pose.print()"<< std::endl;
		filtered_pose.print();
	}

}

void Cperson_bhmip::Cperson_bhmip_add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter, bool person_or_robot)
{
	if(debug_Cperson_bhmip_add_pointV_){
		std::cout<<" (Cperson_bhmip) entro en add_pointV!"<< std::endl;
	}

	SpointV_cov filtered_pose;
	if( filter != Cperson_abstract::No_filtering )
	{
		if(debug_Cperson_bhmip_add_pointV_){
			std::cout<<" (Cperson_bhmip) != Cperson_abstract::No_filtering"<< std::endl;
		}

		SpointV_cov pointV = point;//only positions, position_covariances and time_stamp
		double dx, dy;
		if ( !trajectory_.empty() )
		{
			if(debug_Cperson_bhmip_add_pointV_){
				std::cout<<" (Cperson_bhmip) if !trajectory_.empty()"<< std::endl;
			}

			dx = point.x - trajectory_.back().x;
			dy = point.y - trajectory_.back().y;
			double dt =  point.time_stamp - trajectory_.back().time_stamp ;
			//just to avoid singularities, ideally time stamps are well separated in time
			double marge=0.001;  // marge=0.001;=> anterior valor, que seguía creando singularidades.
			if ( point.time_stamp - trajectory_.back().time_stamp  < marge ) dt = 0.1;

			pointV.vx = dx / dt;
			pointV.vy = dy / dt;

			if(sqrt(pow(pointV.vx,2))>2){
				pointV.vx=trajectory_.back().vx;
			}
			if(sqrt(pow(pointV.vy,2))>2){
				pointV.vy=trajectory_.back().vy;
			}

			 if(debug_filter_person_velocity_){
			std::cout<<" (ANT) dt="<<dt<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<<"; time_window_="<<time_window_<< std::endl;

			std::cout<<" (INI FILTRO V) point.time_stamp="<< point.time_stamp<<"; trajectory_.back().time_stamp= " <<trajectory_.back().time_stamp<< std::endl;
			std::cout<<"dx="<< dx<<"; dy= " <<dy<<"; point.x="<< point.x<<"; trajectory_.back().x="<<trajectory_.back().x<<"; trajectory_.back().y="<<trajectory_.back().y<<"; point.y= " <<point.y<< std::endl;
			std::cout<<"; point.vx="<< point.vx<<"; point.vy= " <<point.vy<< std::endl;
			std::cout<<"trajectory_.back().vx="<< trajectory_.back().vx<<"; trajectory_.back().vy= " <<trajectory_.back().vy<< std::endl;
			std::cout<<" dt="<<dt<<"; pointV.vx="<< pointV.vx<<"; pointV.vy= " <<pointV.vy<< std::endl;
			 }
		}
		else
		{
			if(debug_Cperson_bhmip_add_pointV_){
				std::cout<<" ==trajectory_.empty() "<< std::endl;
			}

			if(person_or_robot){

				pointV.vx=point.vx;
				pointV.vy=point.vy;

				if(debug_Cperson_bhmip_add_pointV_){
					std::cout<<" pointV.vx="<<point.vx<< std::endl;
					std::cout<<" pointV.vy="<<point.vy<< std::endl;
				}
			}else{
				pointV.vx = 0;
				pointV.vy = 0;
				if(debug_Cperson_bhmip_add_pointV_){
					std::cout<<" pointV.vx=pointV.vy=0"<< std::endl;
				}
			}
		}

		if(debug_Cperson_bhmip_add_pointV_){
			std::cout<<" pointV_1 "<< std::endl;
			pointV.print();
		}
		//slide temporal window for all trajectories
		trajectory_.push_back(pointV);
		trajectory_windowing();

		if(debug_Cperson_bhmip_add_pointV_){
			std::cout<<" trajectory_.size() "<<trajectory_.size()<< std::endl;
			std::cout<<" pointV_2 "<< std::endl;
			pointV.print();
		}
		//filters velocities
		switch( filter )
		{
		  case Cperson_abstract::Linear_regression_filtering:
			  if(debug_Cperson_bhmip_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Linear_regression_filtering)" << std::endl;
			  }
			filtered_pose = filter_current_state_linear_regression( );
			break;
		  case Cperson_abstract::Bayes_filtering:
			  if(debug_Cperson_bhmip_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Bayes_filtering)" << std::endl;
			  }
			filtered_pose = filter_current_state_linear_regression_bayes( );
		  	break;
		  case Cperson_abstract::Low_pass_linear_regression_filtering:
		  default :
			  if(debug_Cperson_bhmip_add_pointV_){
				  std::cout << " ( PPPPPPPP Caso: Low_pass_linear_regression_filtering)" << std::endl;
			  }
			filtered_pose = low_pass_filter_current_state_linear_regression( );
			break;
		}


	}
	else
	{
		if(debug_Cperson_bhmip_add_pointV_){
			std::cout << " (Caso: no filter is required!); trajectory_.size()="<<trajectory_.size() << std::endl;
		}

		//no filter is required
		trajectory_.push_back(point);
		//slide temporal window for all trajectories
		trajectory_windowing();
		filtered_pose = trajectory_.back();
	}

	//update current positions and velocities according to filtering results
	diff_pointV_ = filtered_pose - current_pointV_ ;
	current_pointV_ = filtered_pose;
	//std::cout << "(ANTES FILTRO) desired_velocity_="<<desired_velocity_<< std::endl;
	desired_velocity_ = filtered_pose.v();
	if(debug_filter_person_velocity_){
		std::cout << "(DESPUES FILTRO==PERSONA) desired_velocity_="<<desired_velocity_<< std::endl;
	}
	now_ = filtered_pose.time_stamp;


	//precalculates data required for the BHMIP
	intention_precalculation();

	observation_update_ = true;
	if(debug_Cperson_bhmip_add_pointV_){
		std::cout<<" (Cperson_bhmip) salgo de add_pointV! observation_update_="<<observation_update_<<"; trajectory_.size()="<<trajectory_.size()<< std::endl;
	}
}


SpointV_cov Cperson_bhmip::filter_current_state_linear_regression( )
{


	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();

	if(debug_filter_person_velocity_){
		std::cout << " PPPPPPPP (Cperson_bhmip::filter_current_state_linear_regression) window_elements= "<< window_elements << std::endl;
	}
	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::MatrixXd R (window_elements , 2 ); //1st order regression -> 2 coeffs
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		R.row ( i ) << 1 , trajectory_.at(i).time_stamp - initial_ts;

		//std::cout <<"; i="<<i<< " vx ( i )="<<vx ( i ) << "; vy ("<<i<<" )="<<vy ( i )<< std::endl;
		//std::cout << "initial_ts="<<initial_ts<<";  trajectory_.at(i).time_stamp"<< trajectory_.at(i).time_stamp<<"; trajectory_.at(i).time_stamp - initial_ts="<<trajectory_.at(i).time_stamp - initial_ts<<std::endl;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::MatrixXd invR ( 2 , window_elements ) , sqrR ( 2 , 2 );
	sqrR = R.transpose() * R;
	invR = sqrR.inverse() * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------
	Eigen::Vector2d  r_pred (1 , trajectory_.back().time_stamp - initial_ts ),vx_coef,vy_coef;
	//invR * v = beta -> parameter estimation
	vx_coef = invR * vx;
	vy_coef = invR * vy;

	//std::cout << " vx_coef="<<vx_coef<<std::endl;
	//std::cout << " vy_coef="<<vy_coef<<std::endl;

	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			r_pred.dot ( vx_coef ),
			r_pred.dot ( vy_coef ) );
    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();


	//covariance of velocity prediction, with respect to regression velocity
	Eigen::VectorXd vx_mean ( window_elements ) , vy_mean ( window_elements );
	vx_mean = R*vx_coef;
	vy_mean = R*vy_coef;

	if(debug_filter_current_state_linear_regression_){
		std::cout << " vx_coef[0]="<<vx_coef[0] << "vx_coef[1]="<<vx_coef[1]<< std::endl;
		std::cout << " vy_coef[0]="<<vy_coef[0] << "vy_coef[1]="<<vy_coef[1]<< std::endl;
		for(unsigned int n=0;n<window_elements;n++){
			std::cout << " vx_mean("<<n<<")="<<vx_mean(n) << "; vx(n)="<<vx(n)<< std::endl;
			std::cout << " vy_mean("<<n<<")="<<vy_mean(n) << "; vy(n)="<<vy(n)<< std::endl;
		}
	}
	//double cov_vx = (vx - vx_mean).dot( vx - vx_mean );
	//double cov_vy = (vy - vy_mean).dot( vy - vy_mean );
	//double cov_vxy = (vy - vy_mean).dot( vx - vx_mean );

	//std::cout <<" low pass filter!!! vx="<<vx<<"; vx_mean="<<vx_mean<<"; vy="<<vy<<"; vy_mean="<<vy_mean<<std::endl;

	// cambiado por (ely)
	double cov_vx = ((vx - vx_mean).dot( vx - vx_mean ))/window_elements;
	double cov_vy = ((vy - vy_mean).dot( vy - vy_mean ))/window_elements;
	double cov_vxy = ((vy - vy_mean).dot( vx - vx_mean ))/window_elements;

	if(debug_filter_current_state_linear_regression_){
		std::cout << " window_elements="<<window_elements<<"; cov_vx= "<<cov_vx<<"; cov_vy="<<cov_vy<<"; cov_vxy="<<cov_vxy << std::endl;
	}

	//regression covariances + projected uncertainty from positions P_x / dt
	filtered_pose.cov[10] = cov_vx + 0.001;
	filtered_pose.cov[11] = cov_vxy;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = cov_vy + 0.001;

	if(debug_filter_person_velocity_){
		std::cout << " PPPPPPPP (Cperson_bhmip::filter_current_state_linear_regression) filtered_pose: " << std::endl;
		filtered_pose.print();
	}

	return filtered_pose;
}


SpointV_cov Cperson_bhmip::filter_current_state_linear_regression_bayes( )
{

	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();

	if(debug_filter_person_velocity_){
	std::cout << " PPPPPPPP (Cperson_bhmip::filter_current_state_linear_regression_bayes) window_elements= "<< window_elements << std::endl;
	}

	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::MatrixXd R (window_elements , 2 ); //1st order regression -> 2 coeffs
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		R.row ( i ) << 1 , trajectory_.at(i).time_stamp - initial_ts;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::MatrixXd invR ( 2 , window_elements ) , sqrR ( 2 , 2 );
	sqrR = R.transpose() * R;
	invR = sqrR.inverse() * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------
	Eigen::Vector2d  r_pred (1 , trajectory_.back().time_stamp - initial_ts ),vx_coef,vy_coef;
	//invR * v = beta -> parameter estimation
	vx_coef = invR * vx;
	vy_coef = invR * vy;
	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			r_pred.dot ( vx_coef ),
			r_pred.dot ( vy_coef ) );
    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();

	//covariance of velocity prediction, average of uncertainty propagated from estimated positions, equidistribited
	double dt;
	Eigen::Matrix2d v_cov_joint = Eigen::Matrix2d::Zero();
	for( unsigned int i = 0; i < window_elements ; i++)
	{
		if( i == 0 )
			dt = R(1,1)-R(0,1);
		else
			dt = R(i,1)-R(i-1,1);
		Eigen::Matrix2d v_cov;
		v_cov << trajectory_.at(i).cov_xx() , trajectory_.at(i).cov_xy(),
				trajectory_.at(i).cov_xy() , trajectory_.at(i).cov_yy();
		v_cov_joint += v_cov / (dt*dt);
	}

	//regression covariances + projected uncertainty from positions P_x / dt
	double N = 2.0*(double)window_elements*(double)window_elements;
	filtered_pose.cov[10] = v_cov_joint(0,0)/N;
	filtered_pose.cov[11] = v_cov_joint(0,1)/N;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = v_cov_joint(1,1)/N;
	if(debug_filter_person_velocity_){
		std::cout << " PPPPPPPP (Cperson_bhmip::filter_current_state_linear_regression_bayes) filtered_pose: " << std::endl;
		filtered_pose.print();
	}

	return filtered_pose;
}

SpointV_cov Cperson_bhmip::low_pass_filter_current_state_linear_regression( )
{

	//Discard a single and double pose trajectory --------------------------------------------
	if (trajectory_.size() <= 2 )
	{
		return trajectory_.back();
	}
	// First order Regression: y(t) ~ beta * x(t) = beta_1 + beta_2 * x(t) ----------------------------
	//vector and matrices initialization
	unsigned int window_elements = trajectory_.size();

	//if(debug_filter_person_velocity_){
	//std::cout << " PPPPPPPP (Cperson_bhmip::low_pass_filter_current_state_linear_regression) window_elements="<<window_elements <<"; time_window_="<<time_window_<< std::endl;
	//}
//	double initial_ts = trajectory_.front().time_stamp;
	Eigen::VectorXd vx ( window_elements ) , vy ( window_elements );
	Eigen::VectorXd R (window_elements ); //0st order regression -> 1 coeffs
	Eigen::VectorXd r_pred (window_elements);
	for (unsigned int i = 0 ; i < window_elements ; i++)
	{
		vx ( i ) = trajectory_.at(i).vx;
		vy ( i ) = trajectory_.at(i).vy;
		//std::cout <<"i="<<i<< "; trajectory_.at(i).vx= "<<trajectory_.at(i).vx<< "; trajectory_.at(i).vy= "<<trajectory_.at(i).vy<< std::endl;
		R(i)=1;
		r_pred(i)=1;
	}
	//pseudo inverse of the R matrix ---------------------------------------------------------------
	Eigen::VectorXd invR (  window_elements );
	double sqrR =  R.dot( R);
	invR = (1/sqrR) * R.transpose() ;

	//linear regression ----------------------------------------------------------------------------

	//invR * v = beta -> parameter estimation
	double vx_coef =  invR.dot(vx.transpose());
	double vy_coef = invR.dot(vy.transpose());

	//std::cout << "vx_coef= "<<vx_coef<< std::endl;
	//std::cout << "vy_coef= "<<vy_coef<< std::endl;

	SpointV_cov filtered_pose(
			trajectory_.back().x, //no filtering at position
			trajectory_.back().y,
			trajectory_.back().time_stamp,
			vx_coef ,
			vy_coef );

    filtered_pose.cov[0] = trajectory_.back().cov_xx();
    filtered_pose.cov[1] = trajectory_.back().cov_xy();
    filtered_pose.cov[4] = trajectory_.back().cov_xy();
    filtered_pose.cov[5] = trajectory_.back().cov_yy();


	//covariance of velocity prediction, with respect to regression velocity
	Eigen::VectorXd vx_mean ( window_elements ) , vy_mean ( window_elements );
	vx_mean = R*vx_coef;
	vy_mean = R*vy_coef;
	//double cov_vx = (vx - vx_mean).dot( vx - vx_mean );
	//double cov_vy = (vy - vy_mean).dot( vy - vy_mean );
	//double cov_vxy = (vy - vy_mean).dot( vx - vx_mean );

	// cambiado por (ely)
	double cov_vx = ((vx - vx_mean).dot( vx - vx_mean ))/window_elements;
	double cov_vy = ((vy - vy_mean).dot( vy - vy_mean ))/window_elements;
	double cov_vxy = ((vy - vy_mean).dot( vx - vx_mean ))/window_elements;


	//regression covariances + projected uncertainty from positions P_x / dt
	filtered_pose.cov[10] = cov_vx;// + 0.001;
	filtered_pose.cov[11] = cov_vxy;
	filtered_pose.cov[14] = filtered_pose.cov[11];
	filtered_pose.cov[15] = cov_vy;// + 0.001;

	//if(debug_filter_person_velocity_){
		//std::cout << " PPPPPPPP (Cperson_bhmip::low_pass_filter_current_state_linear_regression) filtered_pose: " << std::endl;
		//filtered_pose.print();
	//}

	return filtered_pose;
}


void  Cperson_bhmip::intention_precalculation()
{
	if(debug_intention_precalculation_){
		std::cout << " entro en intention_precalculation() " << std::endl;
	}

		double phi , phi_prob;
		std::vector<double> s;
		phi_pose2dest_.push_back( s );
		phi_prob_.push_back( s );
		for(unsigned int i = 0; i < destinations_.size() ; ++i)
		{
			double dx = destinations_[i].x - current_pointV_.x;
			double dy = destinations_[i].y - current_pointV_.y;
			phi = diffangle( atan2(  dy , dx ) , current_pointV_.orientation());
			phi_pose2dest_.back().push_back( phi );
			//c * Pr ( phi | destination_i) calculations
			phi_prob = 100*exp( - phi*phi / phi_var_ );
			phi_prob_.back().push_back( phi_prob );

			/*std::cout << " destinations_.id="<<destinations_[i].id<< std::endl;
			std::cout << "  angle2=atan2(  dy , dx )="<< atan2(  dy , dx )*180/3.14<< std::endl;
			std::cout << "  diffangle( atan2(  dy , dx ) , current_pointV_.orientation())="<< diffangle( atan2(  dy , dx ) , current_pointV_.orientation())*180/3.14<< std::endl;
			std::cout << "  phi_prob="<< phi_prob<<"; phi_var_="<<phi_var_<< std::endl;*/

		}
		//std::cout << " current_pointV_.orientation()="<<current_pointV_.orientation()*180/3.14<< std::endl;


		if(debug_intention_precalculation_){
			std::cout << " salgo de intention_precalculation() phi_pose2dest_.size()<="<<phi_pose2dest_.size()<<"; phi_prob_.size()="<<phi_prob_.size() << std::endl;
		}

		// test destination companion_person!
		//if(){

		//}

}

void Cperson_bhmip::trajectory_windowing()
{
	if(debug_trajectory_windowing_){
		 std::cout << "(trajectory_windowing()) time_window_="<<time_window_<< std::endl;
		// std::cout << "type_="<<type_<< std::endl;
	 }
	//std::cout << "!!!!!!111 !!!!!!!!!! 8888 trajectory_.size()="<<trajectory_.size()<< std::endl;

	while(  trajectory_.back().time_stamp  - trajectory_.front().time_stamp >  time_window_ )
	{
		 //std::cout << "trajectory_.back().time_stamp="<<trajectory_.back().time_stamp<<"; trajectory_.front().time_stamp"<<trajectory_.front().time_stamp<<"; diference="<<trajectory_.back().time_stamp  - trajectory_.front().time_stamp << std::endl;

		 //std::cout <<"; Cperson_abstract::Person="<<Person<<"; Cperson_abstract::Person_companion="<<Person_companion<<"; type_="<< type_ << std::endl;
		 // std::cout << "trajectory_.size()="<<trajectory_.size()<<"; phi_pose2dest_.size()"<<phi_pose2dest_.size()<<"; phi_prob_.size()="<<phi_prob_.size()  << std::endl;
		trajectory_.pop_front();
		if(((type_ == Person)||(type_ == Person_companion))&&(phi_pose2dest_.size()>trajectory_.size())&&(phi_prob_.size()>trajectory_.size()))//Robot and Virtual_Person do not require prediction methods
		{
			phi_pose2dest_.pop_front();
			phi_prob_.pop_front();
		}
	}





}

void Cperson_bhmip::refresh_person( double now )
{
	if(debug_refresh_person_){
		std::cout << "entro en:Cperson_bhmip::refresh_person."<< std::endl;
	}
	//this method is used when manual remove of persons is required and thus, we must
	//guarantee that the trajectory container and other variables are OK
 // Es para mantener la ventana al mismo tamaño siempre.
	if (trajectory_.empty() )
	{
		if(debug_refresh_person_){
			std::cout << "in trajectory_.empty()"<< std::endl;
			std::cout << "(inicial) time_window_="<<time_window_<<"; trajectory_.back().time_stamp - now="<<trajectory_.back().time_stamp - now<< std::endl;
		}

		while( trajectory_.back().time_stamp - now > time_window_)
		{
			if(debug_refresh_person_){
				std::cout << "(while) time_window_="<<time_window_<<"; trajectory_.back().time_stamp - now="<<trajectory_.back().time_stamp - now<< std::endl;
			}

			trajectory_.pop_front();
			if (trajectory_.empty() ) break;
		}
	}
	now_ = now;
}

void Cperson_bhmip::prediction( double min_v_to_predict )
{
	//std::cout << " 333333377777777 IN Cperson_bhmip::prediction( double min_v_to_predict ) current_pointV_.v()="<<current_pointV_.v()<< std::endl;

	if(debug_prediction_){
		std::cout << "IN Cperson_bhmip::prediction( double min_v_to_predict ) destinations_.size()="<<destinations_.size()<< std::endl;
		for(unsigned int y=0;y<destinations_.size();y++){
			std::cout << "IN Cperson_bhmip::prediction destinations_[y].x="<<destinations_[y].x<<"; destinations_[y].y="<<destinations_[y].y<< std::endl;

		}
	}
	//calculate if it is adequate to predict intentionality:
	//if target is almost stopped and decelerating
	//std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!! OUT (case current_pointV_.v() <  min_v_to_predict); min_v_to_predict="<< min_v_to_predict<<"; current_pointV_.v()="<<current_pointV_.v()<< std::endl;
	//std::cout << "desired_velocity_ = 0.0; best destination:"<< std::endl;

	if( current_pointV_.v() <  min_v_to_predict)
	{

		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Stopping);

		desired_velocity_ = 0.0;//to avoid being too reactive to its environment, it remains in place v_d = 0.0

		//if(debug_prediction_){
			//std::cout << "(case current_pointV_.v() <  min_v_to_predict); min_v_to_predict="<< min_v_to_predict<<"; current_pointV_.v()="<<current_pointV_.v()<< std::endl;
			//std::cout << "desired_velocity_ = 0.0; best destination:"<< std::endl;
			//best_destination_.print();
			//before_best_destination_.print();
		//}

		return;
	}

	//calculate probabilities to destinations
	double norm_term(0.0),max_prob(-0.1);
	unsigned int max_prob_index;
	posterior_destinations_prob_.clear();
	//std::cout <<"; destinations_.size()"<<destinations_.size() << std::endl;

	for( unsigned int i = 0; i<destinations_.size() ; ++i)
	{
		//std::cout <<"; destinations_"<<destinations_[i].id <<"  norm_term ="<<norm_term<<"; destinations_[i].prob="<<destinations_[i].prob<< std::endl;
		posterior_destinations_prob_.push_back( destinations_[i].prob);

		if(debug_prediction_){
			std::cout << "destinations_[i].prob="<<destinations_[i].prob<< std::endl;
		}
		for (unsigned int j = 0; j < phi_prob_.size() ; ++j)
		{
			//phi_probabilities [ time ] [ destination ]
			posterior_destinations_prob_[i] *= phi_prob_[j][i];
			//std::cout << "phi_prob_["<<j<<"]["<<i<<"]="<<phi_prob_[j][i]<< std::endl;
			//std::cout << "posterior_destinations_prob_[i]="<<posterior_destinations_prob_[i]<< std::endl;
		}
		norm_term += posterior_destinations_prob_[i];
		//std::cout <<"norm_term= "<<norm_term<< std::endl;

		if( max_prob < posterior_destinations_prob_[i])
		{
			max_prob_index = i;
			max_prob = posterior_destinations_prob_[i];
			//std::cout <<"max_prob= "<<max_prob<< std::endl;
		}
	}

	//uncertain behavior of prediction depending on certain threshold
	if( 0) //max_prob < 1e16 )//hardcoded threshold corresponds aprox to a diff of 45º
	{
		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Uncertain);
		if(debug_prediction_){
			std::cout << "if(0); best destination:"<< std::endl;
			best_destination_.print();
		}
		return;
	}
	//if(debug_prediction_){

	//}
	//normalization of probabilities
	norm_term = 1/norm_term;

	if(debug_prediction_){
		std::cout << " norm_term ="<<norm_term<< std::endl;
	}

	if ( !is_nan( norm_term ) && !destinations_.empty() )
	{
		for (unsigned int i = 0 ; i < destinations_.size() ; ++i)
			posterior_destinations_prob_[i] *= norm_term;
		best_destination_ = destinations_[max_prob_index];
		best_destination_.prob = posterior_destinations_prob_[max_prob_index];

		if(debug_prediction_){
			std::cout << " if(!is_nan( norm_term ) && !destinations_.empty()) "<< std::endl;
			best_destination_.print();
			before_best_destination_.print();
		}
	}
	else
	{
		best_destination_ = Sdestination( 0, current_pointV_.x, current_pointV_.y,
				1.0, Sdestination::Uncertain);

		if(debug_prediction_){
			std::cout << " else if(!is_nan( norm_term ) && !destinations_.empty() )"<< std::endl;
			best_destination_.print();
		}
	}


	/*if(before_best_destination_.id!=best_destination_.id){
		reset_before_destination_prob();
	}*/

	before_best_destination_=best_destination_;

	if(debug_prediction_){
		std::cout << "OUT Cperson_bhmip::prediction( double min_v_to_predict )"<< std::endl;
	}
}

void Cperson_bhmip::reset(  )
{
	trajectory_.clear();
	current_pointV_ = SpointV_cov();
	diff_pointV_ = SpointV_cov();
	phi_pose2dest_.clear();
	phi_prob_.clear();
	force_to_goal_ = Sforce() ;
	force_int_person_ = Sforce() ;
	force_obstacle_ = Sforce() ;
	force_int_robot_ = Sforce() ;
}

void Cperson_bhmip::reset_before_destination_prob(){
	phi_pose2dest_.clear();
	phi_prob_.clear();
}


void Cperson_bhmip::rotate_and_translate_trajectory(double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry){
	// Function for local tracking. This function change the frame of the Current_pose and the window of poses for the person,
	// tacking into account the actual position of the robot.
	// Also compensate the rotational velocities applied to the person when the robot turns.
		// R -> Robot Translation ; thetaZ -> Robot rotation


	if(debug_odometry){
	std::cout << "!!!!!!!!!!!!!!!!!! (people prediction) rotate_and_translate_trajectory !!!!!!!!!!!!!" << std::endl;
	}
	hom2_ant.clear();

	Eigen::MatrixXd homT1; // homogeneous transfor para pasar los tracks al nuevo frame de la odometria.
	Eigen::MatrixXd homT1vis;
	Eigen::MatrixXd homT2;
	Eigen::MatrixXd J; // necesaria para introducir el aumento de la covarianza por culpa del error en R=traslacion y en thetaZ=rotación.
	Eigen::MatrixXd H_cov;
	Eigen::MatrixXd P;
	homT1.resize(3, 3);
	homT1vis.resize(3, 3);
	homT2.resize(3, 3);
	J.resize(4, 2);
	H_cov.resize(4, 4);
	P.resize(2, 2);

	/*homT1.row(0) << cos(thetaZ), -sin(thetaZ), R * cos(thetaZ/2);
	homT1.row(1) << sin(thetaZ), cos(thetaZ), R * sin(thetaZ/2);
	homT1.row(2) << 0, 0, 1;*/
	
	if(debug_odometry){
	std::cout << "(people prediction) homT1 tf=" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[0]<<","<<vect_odom_eigen_tf[1]<<","<< vect_odom_eigen_tf[2]<<"]" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[3]<<","<<vect_odom_eigen_tf[4]<<","<< vect_odom_eigen_tf[5]<<"]" << std::endl;
	std::cout << "["<<vect_odom_eigen_tf[6]<<","<<vect_odom_eigen_tf[7]<<","<< vect_odom_eigen_tf[8]<<"]"  << std::endl;
	}
	
	homT1.row(0) << vect_odom_eigen_tf[0], vect_odom_eigen_tf[1], vect_odom_eigen_tf[2];
	homT1.row(1) << vect_odom_eigen_tf[3], vect_odom_eigen_tf[4], vect_odom_eigen_tf[5];
	homT1.row(2) << vect_odom_eigen_tf[6], vect_odom_eigen_tf[7], vect_odom_eigen_tf[8];

	if(debug_odometry){
	homT1vis.row(0) << cos(thetaZ), -sin(thetaZ), R * cos(thetaZ/2);
	homT1vis.row(1) << sin(thetaZ), cos(thetaZ), R * sin(thetaZ/2);
	homT1vis.row(2) << 0, 0, 1;
		std::cout << "homT1vis _ prediction  = [ " << homT1(0, 0) << " , " << homT1(0, 1)
				<< " , " << homT1(0, 2) << std::endl << "               "
				<< homT1(1, 0) << " , " << homT1(1, 1) << " , " << homT1(1, 2)
				<< std::endl << "               " << homT1(2, 0) << " , "
				<< homT1(2, 1) << " , " << homT1(2, 2) << std::endl;
	}

	//homT2 = homT1.inverse();
	homT2 = homT1;//.inverse();

	if(debug_odometry){
		std::cout << "homT2 _ prediction = [ " << homT2(0, 0) << " , " << homT2(0, 1)
				<< " , " << homT2(0, 2) << std::endl << "               "
				<< homT2(1, 0) << " , " << homT2(1, 1) << " , " << homT2(1, 2)
				<< std::endl << "               " << homT2(2, 0) << " , "
				<< homT2(2, 1) << " , " << homT2(2, 2) << std::endl;
	}
		/*hom2_ant.push_back(homT2(0, 0));
		hom2_ant.push_back(homT2(0, 1));
		hom2_ant.push_back(homT2(0, 2));
		hom2_ant.push_back(homT2(1, 0));
		hom2_ant.push_back(homT2(1, 1));
		hom2_ant.push_back(homT2(1, 2));
		hom2_ant.push_back(homT2(2, 0));
		hom2_ant.push_back(homT2(2, 1));
		hom2_ant.push_back(homT2(2, 2));
		hom2_tf.push_front(hom2_ant);*/

	H_cov.row(0) << cos(thetaZ), sin(thetaZ), 0, 0;
	H_cov.row(1) << -sin(thetaZ), cos(thetaZ), 0, 0;
	H_cov.row(2) << 0, 0, cos(thetaZ), sin(thetaZ);
	H_cov.row(3) << 0, 0, -sin(thetaZ), cos(thetaZ);

	P.row(0) << 10 * R / 100, 0; //[G_R^2       0    ]
	P.row(1) << 0, 10 * thetaZ / 100; //[  0    G_thetaZ^2]

	// Robot local velocity
	trajectory_local_velocity_x_.push_back(linear_vx);
	trajectory_local_velocity_y_.push_back(linear_vy);


	// Change tracks trajectories to the actual robot frame.

	for(unsigned int i=0; i<trajectory_.size(); i++){

		/*Eigen::MatrixXd homT2_act;
		for(unsigned int j=0;j=<i;j++){
			homT2_act=homT2_act*hom2_tf
		}*/

		SpointV_cov track_ant=trajectory_[i];
		if(debug_odometry){
			std::cout << " Before (people prediction) trajectory_("<<i<<")"<< std::endl;
				trajectory_[i].print();
		}
		SpointV_cov track_act; //trajectory point transformed.

		Eigen::MatrixXd pos_ant(3,1);
		Eigen::MatrixXd pos_act(3,1);
		Eigen::MatrixXd vel_ant(3,1);
		Eigen::MatrixXd vel_act(3,1);
		Eigen::MatrixXd cov_ant(4,4);
		Eigen::MatrixXd cov_act(4,4);
		Eigen::MatrixXd robot_linear_vel(3,1);

		robot_linear_vel(0, 0) = linear_vx;
		robot_linear_vel(1, 0) = linear_vy;
		robot_linear_vel(2, 0) = 0;

		pos_ant(0, 0) = track_ant.x;
		pos_ant(1, 0) = track_ant.y;
		pos_ant(2, 0) = 1;

		pos_act = homT2 * pos_ant;

		vel_ant.row(0) << track_ant.vx;
		vel_ant.row(1) << track_ant.vy;
		vel_ant.row(2) << 0;

		vel_act = homT2 * vel_ant;

		//vel_act=vel_act-robot_linear_vel- w_robot x pos_act;

		J.row(0) << -cos(thetaZ), -sin(thetaZ) * track_ant.x
						+ cos(thetaZ) * track_ant.y + R * sin(thetaZ );
		J.row(1) << sin(thetaZ), -cos(thetaZ) * track_ant.x
						- sin(thetaZ) * track_ant.y + R * cos(thetaZ );
		J.row(2) << 0, -sin(thetaZ) * track_ant.vx + cos(thetaZ) * track_ant.vy;
		J.row(3) << 0, -cos(thetaZ) * track_ant.vx - sin(thetaZ) * track_ant.vy;

		cov_ant.row(0) << track_ant.cov[0], track_ant.cov[1], track_ant.cov[2], track_ant.cov[3];
		cov_ant.row(1) << track_ant.cov[4], track_ant.cov[5], track_ant.cov[6], track_ant.cov[7];
		cov_ant.row(2) << track_ant.cov[8], track_ant.cov[9], track_ant.cov[10], track_ant.cov[11];
		cov_ant.row(3) << track_ant.cov[12], track_ant.cov[13], track_ant.cov[14], track_ant.cov[15];

		//cov_act = H_cov * cov_ant * H_cov.transpose() + J * P * J.transpose();
		cov_act = H_cov * cov_ant * H_cov.transpose();

		std::vector<double> vect_cov_act;
		vect_cov_act.reserve(16);
		vect_cov_act.resize(16, 0.0);
		vect_cov_act[0] = (double) cov_act(0, 0); // cov_xx
		vect_cov_act[1] = (double) cov_act(0, 1); //cov_xy
		vect_cov_act[2] = (double) cov_act(0, 2); // cov_xvx
		vect_cov_act[3] = (double) cov_act(0, 3); //cov_xvy
		vect_cov_act[4] = (double) cov_act(1, 0); //cov_yx
		vect_cov_act[5] = (double) cov_act(1, 1); //cov_yy
		vect_cov_act[6] = (double) cov_act(1, 2); //cov_yvx
		vect_cov_act[7] = (double) cov_act(1, 3); //cov_yvy
		vect_cov_act[8] = (double) cov_act(2, 0); // cov_vxvx
		vect_cov_act[9] = (double) cov_act(2, 1); //cov_vxvy
		vect_cov_act[10] = (double) cov_act(2, 2); // cov_vxx
		vect_cov_act[11] = (double) cov_act(2, 3); //cov_vxy
		vect_cov_act[12] = (double) cov_act(3, 0); //cov_vyvx
		vect_cov_act[13] = (double) cov_act(3, 1); //cov_vyvy
		vect_cov_act[14] = (double) cov_act(3, 2); //cov_vyx
		vect_cov_act[15] = (double) cov_act(3, 3); //cov_vyy

		track_act = SpointV_cov(pos_act(0, 0), pos_act(1, 0),track_ant.time_stamp, vel_act(0, 0), vel_act(1, 0),vect_cov_act);

		trajectory_[i]=track_act;

		if(debug_odometry){
			std::cout << "(people prediction) track_ant("<<i<<")" << std::endl;
			track_ant.print();
			std::cout << " (people prediction) track_act("<<i<<")"<< std::endl;
			track_act.print();
			std::cout << " After (people prediction) trajectory_("<<i<<")"<< std::endl;
			trajectory_[i].print();
		}
	}

	// change current_pointV_ with odometry
	SpointV_cov ant_current_pointV=current_pointV_;
	if(debug_odometry){
		std::cout << " ANT current_pointV_="<< std::endl;
		current_pointV_.print();
	}
	//SpointV_cov act_current_pointV;
	Eigen::MatrixXd pos_ant_current_pointV(3,1);
	Eigen::MatrixXd pos_act_current_pointV(3,1);
	Eigen::MatrixXd vel_ant_current_pointV(3,1);
	Eigen::MatrixXd vel_act_current_pointV(3,1);
	Eigen::MatrixXd cov_ant_current_pointV(4,4);
	Eigen::MatrixXd cov_act_current_pointV(4,4);

	pos_ant_current_pointV(0, 0) = ant_current_pointV.x;
	pos_ant_current_pointV(1, 0) = ant_current_pointV.y;
	pos_ant_current_pointV(2, 0) = 1;

	pos_act_current_pointV = homT2 * pos_ant_current_pointV;

	vel_ant_current_pointV.row(0) << ant_current_pointV.vx;
	vel_ant_current_pointV.row(1) << ant_current_pointV.vy;
	vel_ant_current_pointV.row(2) << 0;

	vel_act_current_pointV = homT2 * vel_ant_current_pointV;

	J.row(0) << -cos(thetaZ), -sin(thetaZ) * ant_current_pointV.x + cos(thetaZ) * ant_current_pointV.y + R * sin(thetaZ / 2);
	J.row(1) << sin(thetaZ), -cos(thetaZ) * ant_current_pointV.x - sin(thetaZ) * ant_current_pointV.y + R * cos(thetaZ / 2);
	J.row(2) << 0, -sin(thetaZ) * ant_current_pointV.vx + cos(thetaZ) * ant_current_pointV.vy;
	J.row(3) << 0, -cos(thetaZ) * ant_current_pointV.vx - sin(thetaZ) * ant_current_pointV.vy;

	cov_ant_current_pointV.row(0) << ant_current_pointV.cov[0], ant_current_pointV.cov[1], ant_current_pointV.cov[2], ant_current_pointV.cov[3];
	cov_ant_current_pointV.row(1) << ant_current_pointV.cov[4], ant_current_pointV.cov[5], ant_current_pointV.cov[6], ant_current_pointV.cov[7];
	cov_ant_current_pointV.row(2) << ant_current_pointV.cov[8], ant_current_pointV.cov[9], ant_current_pointV.cov[10], ant_current_pointV.cov[11];
	cov_ant_current_pointV.row(3) << ant_current_pointV.cov[12], ant_current_pointV.cov[13], ant_current_pointV.cov[14], ant_current_pointV.cov[15];

	cov_act_current_pointV = H_cov * cov_ant_current_pointV * H_cov.transpose() + J * P * J.transpose();

	std::vector<double> vect_cov_act_current_pointV;
	vect_cov_act_current_pointV.reserve(16);
	vect_cov_act_current_pointV.resize(16, 0.0);
	vect_cov_act_current_pointV[0] = (double) cov_act_current_pointV(0, 0); // cov_xx
	vect_cov_act_current_pointV[1] = (double) cov_act_current_pointV(0, 1); //cov_xy
	vect_cov_act_current_pointV[2] = (double) cov_act_current_pointV(0, 2); // cov_xvx
	vect_cov_act_current_pointV[3] = (double) cov_act_current_pointV(0, 3); //cov_xvy
	vect_cov_act_current_pointV[4] = (double) cov_act_current_pointV(1, 0); //cov_yx
	vect_cov_act_current_pointV[5] = (double) cov_act_current_pointV(1, 1); //cov_yy
	vect_cov_act_current_pointV[6] = (double) cov_act_current_pointV(1, 2); //cov_yvx
	vect_cov_act_current_pointV[7] = (double) cov_act_current_pointV(1, 3); //cov_yvy
	vect_cov_act_current_pointV[8] = (double) cov_act_current_pointV(2, 0); // cov_vxvx
	vect_cov_act_current_pointV[9] = (double) cov_act_current_pointV(2, 1); //cov_vxvy
	vect_cov_act_current_pointV[10] = (double) cov_act_current_pointV(2, 2); // cov_vxx
	vect_cov_act_current_pointV[11] = (double) cov_act_current_pointV(2, 3); //cov_vxy
	vect_cov_act_current_pointV[12] = (double) cov_act_current_pointV(3, 0); //cov_vyvx
	vect_cov_act_current_pointV[13] = (double) cov_act_current_pointV(3, 1); //cov_vyvy
	vect_cov_act_current_pointV[14] = (double) cov_act_current_pointV(3, 2); //cov_vyx
	vect_cov_act_current_pointV[15] = (double) cov_act_current_pointV(3, 3); //cov_vyy

	//double track_vx=vel_act_current_pointV(0,0) - v_rot_x;
	//double track_vy=vel_act_current_pointV(1,0) - v_rot_y;

	//vel_act_current_pointV(0,0)=track_vx;
	//vel_act_current_pointV(1,0)=track_vy;



	current_pointV_ = SpointV_cov(pos_act_current_pointV(0, 0), pos_act_current_pointV(1, 0), ant_current_pointV.time_stamp, vel_act_current_pointV(0,0), vel_act_current_pointV(1,0),vect_cov_act_current_pointV);
	if(debug_odometry){
		std::cout << " AcT current_pointV_="<< std::endl;
		current_pointV_.print();
	}
}
