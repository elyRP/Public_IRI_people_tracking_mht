#include "scene_elements/person_abstract.h"
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>



Cperson_abstract::Cperson_abstract(  unsigned int id , Cperson_abstract::target_type target_type,
		Cperson_abstract::force_type person_force_type, double _time_window):
	person_id_( id ) ,
	desired_velocity_(0.0),
	type_(target_type),
	person_force_type_(person_force_type),
	observation_update_(false),
	now_(0.0),
	debug_velocity_file_(false),
	debug_velocity_cout_(false),
	externa_force_k_near_goal_(2.3),
	externa_force_k_far_goal_(2.3),
	max_dist_to_near_goal_force_(3),
	debug_force_goal_(false)
{
	//std::cout  << " type_=" << type_ << std::endl;
	//std::cout  << " person_id_=" << person_id_ << std::endl;

}

Cperson_abstract::~Cperson_abstract()
{

}

bool Cperson_abstract::operator== (Cperson_abstract& p2) const
{
	if ( person_id_ == p2.get_id() && type_ == p2.get_person_type() )
		return true;
	else
		return false;
}

//TODO add method for implicitly calculating force depending on prediction if none is provided
SpointV_cov Cperson_abstract::pointV_propagation( double dt , Sforce force )
{
	//std::cout << std::endl << "  (pointV_propagation) now_- current_pointV_.time_stamp" << now_- current_pointV_.time_stamp<<" now_="<<now_ << "dt="<<dt<<"desired_velocity_"<<desired_velocity_ << std::endl;
	//std::cout << std::endl << " current_pointV_.x=" << current_pointV_.x<< "; current_pointV_.y=" << current_pointV_.y<< "; current_pointV_.vy=" << current_pointV_.vy<<"; current_pointV_.vx=" << current_pointV_.vx<< std::endl;

	return current_pointV_.propagate( now_- current_pointV_.time_stamp + dt , force , desired_velocity_);
}

SpointV_cov Cperson_abstract::pointV_propagation( double dt , Sforce force, SpointV_cov point )
{
	//std::cout << std::endl << "  (pointV_propagation) now_- current_pointV_.time_stamp" << now_- current_pointV_.time_stamp << "dt="<<dt<<"desired_velocity_"<<desired_velocity_ << std::endl;
	return point.propagate( now_- current_pointV_.time_stamp + dt,force, desired_velocity_ );
}


void Cperson_abstract::print()
{
	std::cout << std::endl << "              last pose " << person_id_ << "th print :" << std::endl;
	current_pointV_.print();
	return;
}

void Cperson_abstract::print_dest()
{
	std::cout << std::endl << "                  Best destination of person " << person_id_ << std::endl;
	best_destination_.print();
}

Sforce Cperson_abstract::force_ellip( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{

	//if force is calculated from a propagated pose, then virtual_current_point should be pointing to a valid SpointV.
	//Otherwise, current point is current point class variable at time "now"
	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	//geometry calculations
	double tau = social_forces_param->at(4);
	SpointV pose_dif = (SpointV)(*current_point) - interacting_person;
	Spoint d_point( pose_dif.x,pose_dif.y );
	Spoint v_current = Spoint( current_point->vx , current_point->vy );
	Spoint v_int = Spoint( interacting_person.vx , interacting_person.vy );
	//Spoint y_point = d_point - (v_int - v_current )*tau;
	//force module calculation
	double b = d_point.distance() + (d_point - ( v_int - v_current )*tau ).distance();
	b = 0.5* sqrt( ( b*b - (( v_int - v_current )*tau ).distance() * (( v_int - v_current )*tau ).distance()
			) / ( 1 + v_current.distance() * tau ) );
	double f = social_forces_param->at(2)* exp( (- b) / social_forces_param->at(3));
	f = f * (d_point.distance() + (d_point - ( v_int - v_current )*tau ).distance()) / sqrt( 1 + v_current.distance() * tau ) / (4*b);
	//force direction
	double phi = current_point->angle_heading_point( interacting_person );
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	double dx,dy;
	if ( (( v_int - v_current )*tau).distance() > 0.01 )
	{
	dx = d_point.x/d_point.distance() + (( v_int - v_current )*tau).x / (( v_int - v_current )*tau).distance() ;
	dy = d_point.y/d_point.distance() + (( v_int - v_current )*tau).y / (( v_int - v_current )*tau).distance() ;
	}
	else
	{
		dx = d_point.x/d_point.distance();
		dy = d_point.y/d_point.distance();
	}
	force.fx = f * dx * anisotropy;
	force.fy = f * dy * anisotropy;
	return force;
}

Sforce Cperson_abstract::force_cp( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point )
{


	//if force is calculated from a propagated pose, then virtual_current_point won't be null.
	//Otherwise, current point is current point class variable
	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	//obtaining time of collision prediction
	//t = - (dx*dvx + dy * dvy ) / (dvx^2 + dvy^2)
	double dx = current_point->x - interacting_person.x;
	double dy = current_point->y - interacting_person.y;
	double vx = current_point->vx;
	double vx_i = interacting_person.vx;
	double dvx =  vx - vx_i;
	double vy = current_point->vy;
	double vy_i = interacting_person.vy;
	double dvy = vy - vy_i;
	double time_minimum_distance = - (dx*dvx + dy*dvy ) / (dvx*dvx + dvy*dvy);

	//propagating state-
	double dx_prop = current_point->x + time_minimum_distance * vx -
			interacting_person.x - time_minimum_distance * vx_i;
	double dy_prop = current_point->y + time_minimum_distance * vy -
			interacting_person.y - time_minimum_distance * vy_i;
	double d_cp = dx_prop * dx_prop + dy_prop * dy_prop;

	//exponential force module f = A*v/t* exp(d_cp / B)
	double f;
	if ( time_minimum_distance > 0  &&
		fabs(diffangle(diffangle(current_point->orientation(),interacting_person.orientation()) , atan2( dvy , dvx ) )) < PI/2 )
	{
		if (time_minimum_distance < 0.2 ) time_minimum_distance = 0.2;
		if (d_cp < 0.36)
		{
			dx_prop = dx_prop / d_cp * 0.36;//divide mal... too bad a corregir mañana
			dy_prop = dy_prop / d_cp * 0.36;
			d_cp = 0.36;//0.3 m radii per person
		}
		f = social_forces_param->at(2)*current_point->v() / time_minimum_distance * exp(- d_cp / social_forces_param->at(3));
		//force direction
		double phi = diffangle( atan2(vy,vx), atan2(dy_prop,dx_prop) );
		double anisotropy =  (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
		force.fx = f * dx_prop * anisotropy;
		force.fy = f * dy_prop * anisotropy;
	}

	return force;
}

Sforce Cperson_abstract::force_sphe( const Spoint& interacting_person ,  const std::vector<double>* social_forces_param , const SpointV* virtual_current_point, Cperson_abstract::companion_reactive reactive )
{ // ESTE ES EL QUE USA GONZALO!!!

	Sforce force,force2;
	double f,d;

	//if force is calculated from a propagated pose, then virtual_current_point won't be null.
	//Otherwise, current point is current point class variable
	//std::cout << " IN :Cperson_abstract::force_sphe => case(Reactiva_repulsive+atractive) " <<  std::endl;


	//std::cout << "IN force_sphe!!!;   target_type="<< type_<< std::endl;

	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	//geometry calculations
	double dx = current_point->x - interacting_person.x;
	double dy = current_point->y - interacting_person.y;
	double vx = current_point->vx;
	double vy = current_point->vy;
	d = sqrt(dx*dx+dy*dy);
	dx /= d;
	dy /= d;
	//dx=dx-current_point->vx*0.2;
	//dy=dy-current_point->vy*0.2;
	//d = sqrt(dx*dx+dy*dy);


	/*if(d<1.25){
		std::cout << " [FFFFORCEEEEE] social_forces_param->at(2)="<<social_forces_param->at(2)<<"; social_forces_param->at(4)="<< social_forces_param->at(4) <<"; social_forces_param->at(3)="<<social_forces_param->at(3)<<"; social_forces_param->at(1)="<<social_forces_param->at(1)<<  std::endl;
		social_forces_param->at(2)=7;
		 social_forces_param->at(4)=0.2;
		 social_forces_param->at(3)=0.91;

	}*/


	double anisotropy;
	double phi;

	//std::cout << "(reactive atractive) IN force_sphe!!! target_type="<<type_<< std::endl;
	//force module calculation
	f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));

	phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
	// diffangle= second_angle - first_angle

	anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	// [lambda + (1- lambda)*((1-cos(phi))/2)]
	double y;
	double y2;

	switch(reactive)
	{

		case Reactive_atractive:
			//std::cout << "(reactive atractive) IN force_sphe!!! person_id_="<<person_id_<< std::endl;
			//force module calculation
				f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));
			//std::cout << "social_forces_param->at(1)="<<social_forces_param->at(1)<<"; social_forces_param->at(2)="<<social_forces_param->at(2)<<"; social_forces_param->at(3)="<<social_forces_param->at(3)<<"; social_forces_param->at(4)="<<social_forces_param->at(4)<< std::endl;

			//f = 0.3373* exp((16.9302-d) / (-5.2005));
			//y= 5.59* exp((0.25-d) / (0.91));
			//y2= -5.2005* exp((0.2-d) / (16.93));

				//f = y+y2;//5.59* exp((0.25-d) / (0.91));

				//std::cout << " [FFFFORCEEEEE] social_forces_param->at(2)="<<social_forces_param->at(2)<<"; social_forces_param->at(4)="<< social_forces_param->at(4) <<"; social_forces_param->at(3)="<<social_forces_param->at(3)<<"; social_forces_param->at(1)="<<social_forces_param->at(1)<<  std::endl;

				// Aj*exp((dj-d)/Bj)
				//force direction
				phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
				// diffangle= second_angle - first_angle
				anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
				//anisotropy = (9.2875 + (1-9.2875)*(1 + cos(phi))/2 );
				// [lambda + (1- lambda)*((1-cos(phi))/2)]
			//std::cout << " IN :Cperson_abstract::force_sphe => case: Reactive_atractive " <<  std::endl;
			force.fx = (f * dx * anisotropy);
			force.fy = (f * dy * anisotropy);
		break;

		case Reactiva_repulsive:
		case Akp_planning:
		default:
			//std::cout << "(Reactiva_repulsive) IN force_sphe!!!"<< std::endl;
			//std::cout << "social_forces_param->at(1)="<<social_forces_param->at(1)<<"; social_forces_param->at(2)="<<social_forces_param->at(2)<<"; social_forces_param->at(3)="<<social_forces_param->at(3)<<"; social_forces_param->at(4)="<<social_forces_param->at(4)<<"; d="<<d<< std::endl;
			//force module calculation
				f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));
				//f = 0.3373* exp((16.9302-d) / (-5.2005));
			///y= 5.59* exp((0.25-d) / (0.91));
			 //f= -5.2005* exp((0.2-d) / (16.93));

				//f = y+y2;//5.59* exp((0.25-d) / (0.91));

				//std::cout << " [FFFFORCEEEEE] AKP planning; social_forces_param->at(2)="<<social_forces_param->at(2)<<"; social_forces_param->at(4)="<< social_forces_param->at(4) <<"; social_forces_param->at(3)="<<social_forces_param->at(3)<<"; social_forces_param->at(1)="<<social_forces_param->at(1)<<  std::endl;

				// Aj*exp((dj-d)/Bj)
				//force direction
				phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
				// diffangle= second_angle - first_angle
				anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
				//anisotropy = (9.2875 + (1-9.2875)*(1 + cos(phi))/2 );
				// [lambda + (1- lambda)*((1-cos(phi))/2)]
			//std::cout << " IN :Cperson_abstract::force_sphe => case: Reactive_repulsive and AKP_planning " <<  std::endl;
			force.fx = f * dx * anisotropy;
			force.fy = f * dy * anisotropy;
			//std::cout << " IN :Cperson_abstract::force_sphe => force.fx="<<force.fx<<"; force.fy="<<force.fy <<  std::endl;
		break;
	}

	/*double dist=0.81;
	double f2 = social_forces_param->at(2)* exp((social_forces_param->at(4)-dist) / social_forces_param->at(3));
	double phi2 = diffangle( atan2(vy,vx), atan2(-0.6,-0.55) );//minus difference vector
	double anisotropy2 = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi2))/2 );
	force2.fx = f2 * 0.55 * anisotropy2;
	force2.fy = f2 * 0.6 * anisotropy2;
	std::cout << " [FFFFORCEEEEE] force2.fx="<<force2.fx<<"force2.fy="<< force2.fy<<  std::endl;*/

	return force;
}


Sforce Cperson_abstract::force( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point, Cperson_abstract::companion_reactive reactive )
{

	Sforce force_res;
	switch(reactive)
	{

		case Reactiva_repulsive:
		case Reactive_atractive:
			//std::cout << " IN :Cperson_abstract::force => case(Reactiva_repulsive+atractive) " <<  std::endl;
			assert( !social_forces_param->empty() );
			switch(person_force_type_)
			{
				case Elliptical:
					force_res = force_ellip(  interacting_person ,  social_forces_param, virtual_current_point  );
					break;
				case Collision_Prediction:
					force_res = force_cp(  interacting_person ,  social_forces_param, virtual_current_point );
					break;
				case Spherical:
				default:
					force_res = force_sphe(  interacting_person ,  social_forces_param, virtual_current_point , reactive );
					break;
			}
		break;

		case Akp_planning:
		default:
			assert( !social_forces_param->empty() );
			switch(person_force_type_)
			{
			case Elliptical:
				force_res = force_ellip(  interacting_person ,  social_forces_param, virtual_current_point  );
				break;
			case Collision_Prediction:
				force_res = force_cp(  interacting_person ,  social_forces_param, virtual_current_point );
				break;
			case Spherical:
			default:
				force_res = force_sphe(  interacting_person ,  social_forces_param, virtual_current_point  );
				break;
			}
		break;

	}
	return force_res;
}



Sforce Cperson_abstract::forceAnticipateCollision( force_type act_person_force_type, const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point, Cperson_abstract::companion_reactive reactive )
{

	Sforce force_res;
	switch(reactive)
	{

		case Reactiva_repulsive:
		case Reactive_atractive:
			//std::cout << " IN :Cperson_abstract::force => case(Reactiva_repulsive+atractive) " <<  std::endl;
			assert( !social_forces_param->empty() );
			switch(act_person_force_type)
			{
				case Elliptical:
					force_res = force_ellip(  interacting_person ,  social_forces_param, virtual_current_point  );
					break;
				case Collision_Prediction:
					force_res = force_cp(  interacting_person ,  social_forces_param, virtual_current_point );
					break;
				case Spherical:
				default:
					force_res = force_sphe(  interacting_person ,  social_forces_param, virtual_current_point , reactive );
					break;
			}
		break;

		case Akp_planning:
		default:
			assert( !social_forces_param->empty() );
			switch(act_person_force_type)
			{
			case Elliptical:
				force_res = force_ellip(  interacting_person ,  social_forces_param, virtual_current_point  );
				break;
			case Collision_Prediction:
				force_res = force_cp(  interacting_person ,  social_forces_param, virtual_current_point );
				break;
			case Spherical:
			default:
				force_res = force_sphe(  interacting_person ,  social_forces_param, virtual_current_point  );
				break;
			}
		break;

	}
	return force_res;
}

Sforce Cperson_abstract::force_goal(  const Sdestination& dest, const std::vector<double>* social_forces_param, const SpointV* virtual_current_point, unsigned int parent_vertex, bool output_mesages )
{

	if(parent_vertex<0){
		std::cout << "IN (Cperson_abstract::force_goal); k="<<social_forces_param->at(0)<<"; desired_velocity_="<<desired_velocity_ << std::endl;
	}

	//if there are no is inferred, then no force to goal is possible (linear propagation + interaction forces)
	if ( dest.type == Sdestination::Uncertain  ) return Sforce();

	//std::cout << "(force_goal) 2=" << std::endl;

	//if force is calculated from a propagated pose, then index would be different form zero and
	// the prediction vector should not be empty
	const SpointV* current_point;

	if ( virtual_current_point == NULL ){
		//std::cout << "SI NULL pointer=" << std::endl;
		current_point = &current_pointV_;
	}
	else{
		//std::cout << " no null pointer=" << std::endl;
		current_point = virtual_current_point;
	}

	//std::cout << "(force_goal) 3=" << std::endl;
	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	//std::cout << "(force_goal) 4=" << std::endl;
	double v_desired,v_desired_x,v_desired_y;
	double act_module_dist=0.0;
	if ( dest.type == Sdestination::Map_goal  )
	{
		if(debug_force_goal_){
			std::cout << "MAP GOAL if( dest.type == Sdestination::Map_goal ); (dest.x="<<dest.x<<"; dest.y="<<dest.y<<")=(current_point->x="<<current_point->x<<"; current_point->y="<<current_point->y<<")" << std::endl;
		}

		//v_desired_x = dest.x - current_point->x;
		//v_desired_y = dest.y - current_point->y;
		v_desired_x = dest.x - current_point->x;
		v_desired_y = dest.y - current_point->y;

		if(parent_vertex<0){ //(debug_sideBySide2019_)
			std::cout << "(MAP GOAL if) v_desired_x="<<v_desired_x<< "; dest.x ="<<dest.x <<"; current_point->x="<<current_point->x <<"; desired_velocity_="<<desired_velocity_ << std::endl;
			std::cout << "(MAP GOAL if) v_desired_y="<<v_desired_y<< "; dest.y ="<<dest.y <<"; current_point->y="<<current_point->y << std::endl;
		}

		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		act_module_dist=v_desired;

		if(debug_force_goal_){
			std::cout << "(ini iter!) v_desired="<<v_desired<< std::endl;
			std::cout << " desired_velocity_="<<desired_velocity_ << std::endl;
		}

		v_desired_x *= desired_velocity_/v_desired;
		v_desired_y *= desired_velocity_/v_desired;

		//v_desired_x = (v_desired_x*desired_velocity_)/v_desired;
		//v_desired_y = (v_desired_y*desired_velocity_)/v_desired;

		//v_desired_x = (v_desired_x);///v_desired;
		//v_desired_y = (v_desired_y);///v_desired;

		if(parent_vertex<0){ //(debug_sideBySide2019_)
			std::cout << "v_desired_x="<<v_desired_x<<"v_desired_y="<<v_desired_y<< std::endl;
			std::cout <<"; desired_velocity_="<<desired_velocity_<<"; v_desired="<<v_desired<<  std::endl;
		}
	}
	else //Sdestination::Stopping
	{
		if(output_mesages){
			std::cout << "else if( dest.type == Sdestination::Map_goal ) v_desired_x=v_desired_y=0" << std::endl;
		}

		v_desired_x = 0.0;
		v_desired_y = 0.0;
	}


	//std::cout << "(force_goal) 5=" << std::endl;
	if(output_mesages){
	//if(parent_vertex<2){
		std::cout << "k=social_forces_param->at(0)="<<social_forces_param->at(0) <<"; v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y<<"; current_point->vx="<<current_point->vx<<"; current_point->vy="<<current_point->vy<< std::endl;
		std::cout << "(v_desired_x - current_point->vx)="<<(v_desired_x - current_point->vx) <<"; social_forces_param->at(0)*(v_desired_x - current_point->vx)="<<social_forces_param->at(0)*(v_desired_x - current_point->vx)<< std::endl;
		std::cout << "(v_desired_y - current_point->vy)="<<(v_desired_y - current_point->vy) <<"; social_forces_param->at(0)*(v_desired_y - current_point->vy)="<<social_forces_param->at(0)*(v_desired_y - current_point->vy)<< std::endl;

	}
			//}


	v_desired_robot_x_.push_back(v_desired_x);
	v_desired_robot_y_.push_back(v_desired_y);
	//v_current_robot_x_.push_back(dest.y - current_point->y);
	//v_current_robot_y_.push_back(v_desired);
	v_current_robot_x_.push_back(current_point->vx);
	v_current_robot_y_.push_back(current_point->vy);
	//actual_random_goal_x_.push_back(desired_velocity_);
	actual_random_goal_x_.push_back(dest.x);
	actual_random_goal_y_.push_back(dest.y);
	//actual_random_goal_y_.push_back(dest.x - current_point->x);



    actual_distance_in_x_.push_back(dest.x - current_point->x);
   	actual_distance_in_y_.push_back(dest.y - current_point->y);
	actual_module_distance_.push_back(act_module_dist);
	actual_in_desired_velocity_.push_back(desired_velocity_);

	force_to_goal_ = Sforce(	social_forces_param->at(0) * (v_desired_x - current_point->vx),
			social_forces_param->at(0) * (v_desired_y - current_point->vy));

	//if(debug_force_goal_){
	if(parent_vertex<0){ //    if(debug_sideBySide2019_){
		std::cout << "(force_goal) => v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y <<"; dest.x="<<dest.x<<"; dest.y="<<dest.y<< std::endl;
		std::cout << "(force_goal) => force_to_goal_.fx="<<force_to_goal_.fx<<"; force_to_goal_.fy="<<force_to_goal_.fy << std::endl;
		std::cout << "(force_goal) => current_point->vx="<<current_point->vx<<"; current_point->vy="<<current_point->vy << std::endl;
	}


	//std::cout << " k=social_forces_param->at(0)="<<social_forces_param->at(0) << std::endl;

	return  force_to_goal_;
}


Sforce Cperson_abstract::force_goal_companion(  const Sdestination& dest, const std::vector<double>* social_forces_param, const SpointV* virtual_current_point, unsigned int parent_vertex, bool output_mesages )
{

	if(parent_vertex<0){
		std::cout << "IN (Cperson_abstract::force_goal); k="<<social_forces_param->at(0)<<"; desired_velocity_="<<desired_velocity_ << std::endl;
	}

	//if there are no is inferred, then no force to goal is possible (linear propagation + interaction forces)
	if ( dest.type == Sdestination::Uncertain  ) return Sforce();

	//std::cout << "(force_goal) 2=" << std::endl;

	//if force is calculated from a propagated pose, then index would be different form zero and
	// the prediction vector should not be empty
	const SpointV* current_point;

	if ( virtual_current_point == NULL ){
		//std::cout << "SI NULL pointer=" << std::endl;
		current_point = &current_pointV_;
	}
	else{
		//std::cout << " no null pointer=" << std::endl;
		current_point = virtual_current_point;
	}

	//std::cout << "(force_goal) 3=" << std::endl;
	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	//std::cout << "(force_goal) 4=" << std::endl;
	double v_desired,v_desired_x,v_desired_y;
	double act_module_dist=0.0;
	if ( dest.type == Sdestination::Map_goal  )
	{
		if(debug_force_goal_){
			std::cout << "MAP GOAL if( dest.type == Sdestination::Map_goal ); (dest.x="<<dest.x<<"; dest.y="<<dest.y<<")=(current_point->x="<<current_point->x<<"; current_point->y="<<current_point->y<<")" << std::endl;
		}

		//v_desired_x = dest.x - current_point->x;
		//v_desired_y = dest.y - current_point->y;
		v_desired_x = (dest.x - current_point->x); // dt_=0.2 (luego cogerlo de fuera.
		v_desired_y = (dest.y - current_point->y);

		double v_desired_module=sqrt((dest.x - current_point->x)*(dest.x - current_point->x)+(dest.y - current_point->y)*(dest.y - current_point->y));

		if(parent_vertex<0){
			std::cout << "(MAP GOAL if) v_desired_x="<<v_desired_x<< "; dest.x ="<<dest.x <<"; current_point->x="<<current_point->x <<"; desired_velocity_="<<desired_velocity_ << std::endl;
			std::cout << "(MAP GOAL if) v_desired_y="<<v_desired_y<< "; dest.y ="<<dest.y <<"; current_point->y="<<current_point->y << std::endl;
		}

		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		act_module_dist=v_desired;

		if(debug_force_goal_){
			std::cout << "(ini iter!) v_desired="<<v_desired<< std::endl;
			std::cout << " desired_velocity_="<<desired_velocity_ << std::endl;
		}

		//if((desired_velocity_<v_desired_module)||(desired_velocity_==0)||(v_desired_module<0)){
			v_desired_module=desired_velocity_;
		//}

		//v_desired_x *= desired_velocity_/v_desired;
		//v_desired_y *= desired_velocity_/v_desired;
		//desired_velocity_=0.6;
		v_desired_x *= v_desired_module/v_desired;
		v_desired_y *= v_desired_module/v_desired;

		//v_desired_x = (v_desired_x)/v_desired;
		//v_desired_y = (v_desired_y)/v_desired;

		if(parent_vertex<0){
			std::cout << "v_desired_x="<<v_desired_x<<"v_desired_y="<<v_desired_y<<"; v_desired_module="<<v_desired_module<< std::endl;
			std::cout <<"; desired_velocity_="<<desired_velocity_<<"; v_desired="<<v_desired<<  std::endl;
		}
	}
	else //Sdestination::Stopping
	{
		if(output_mesages){
			std::cout << "else if( dest.type == Sdestination::Map_goal ) v_desired_x=v_desired_y=0" << std::endl;
		}

		v_desired_x = 0.0;
		v_desired_y = 0.0;
	}


	//std::cout << "(force_goal) 5=" << std::endl;
	if(output_mesages){
	//if(parent_vertex<2){
		std::cout << "k=social_forces_param->at(0)="<<social_forces_param->at(0) <<"; v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y<<"; current_point->vx="<<current_point->vx<<"; current_point->vy="<<current_point->vy<< std::endl;
		std::cout << "(v_desired_x - current_point->vx)="<<(v_desired_x - current_point->vx) <<"; social_forces_param->at(0)*(v_desired_x - current_point->vx)="<<social_forces_param->at(0)*(v_desired_x - current_point->vx)<< std::endl;
		std::cout << "(v_desired_y - current_point->vy)="<<(v_desired_y - current_point->vy) <<"; social_forces_param->at(0)*(v_desired_y - current_point->vy)="<<social_forces_param->at(0)*(v_desired_y - current_point->vy)<< std::endl;

	}
			//}


	v_desired_robot_x_.push_back(v_desired_x);
	v_desired_robot_y_.push_back(v_desired_y);
	//v_current_robot_x_.push_back(dest.y - current_point->y);
	//v_current_robot_y_.push_back(v_desired);
	v_current_robot_x_.push_back(current_point->vx);
	v_current_robot_y_.push_back(current_point->vy);
	//actual_random_goal_x_.push_back(desired_velocity_);
	actual_random_goal_x_.push_back(dest.x);
	actual_random_goal_y_.push_back(dest.y);
	//actual_random_goal_y_.push_back(dest.x - current_point->x);



    actual_distance_in_x_.push_back(dest.x - current_point->x);
   	actual_distance_in_y_.push_back(dest.y - current_point->y);
	actual_module_distance_.push_back(act_module_dist);
	actual_in_desired_velocity_.push_back(desired_velocity_);

	force_to_goal_ = Sforce(	social_forces_param->at(0) * (v_desired_x - current_point->vx),
			social_forces_param->at(0) * (v_desired_y- current_point->vy));

	//if(debug_force_goal_){
	if(parent_vertex<0){
		std::cout << "(force_goal) => v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y <<"; dest.x="<<dest.x<<"; dest.y="<<dest.y<< std::endl;
		std::cout << "(force_goal) => force_to_goal_.fx="<<force_to_goal_.fx<<"; force_to_goal_.fy="<<force_to_goal_.fy << std::endl;
		std::cout << "(force_goal) => current_point->vx="<<current_point->vx<<"; current_point->vy="<<current_point->vy << std::endl;
	}


	//std::cout << " k=social_forces_param->at(0)="<<social_forces_param->at(0) << std::endl;

	return  force_to_goal_;
}



Sforce Cperson_abstract::force_goal_near(  const Sdestination& dest, const std::vector<double>* social_forces_param, const SpointV* virtual_current_point, double d_max_reduce_vel , bool in_go_with_vel_per, double real_person_vel)
{

	//std::cout << "  force_goal_near!!! " << std::endl;

	/*if(real_person_vel==0.9){
		real_person_vel=desired_velocity_;
	}*/

	double act_module_dist=0.0;
	//if there are no is inferred, then no force to goal is possible (linear propagation + interaction forces)
	if ( dest.type == Sdestination::Uncertain  ) return Sforce();

	//if force is calculated from a propagated pose, then index would be different form zero and
	// the prediction vector should not be empty
	const SpointV* current_point;
	if ( virtual_current_point == NULL )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );


	Spoint robot_act_pose=Spoint(current_point->x,current_point->y,current_point->time_stamp);
	Spoint goal_act_pose=Spoint(dest.x,dest.y,dest.time_stamp);
	double dist_to_near_goal= robot_act_pose.distance( goal_act_pose);


	double v_desired,v_desired_x,v_desired_y;
	if ( dest.type == Sdestination::Map_goal  )
	{
		v_desired_x = dest.x - current_point->x;
		v_desired_y = dest.y - current_point->y;

		/*std::cout << "dest.x="<<dest.x<<"; current_point->x="<<current_point->x<< std::endl;
		std::cout << "dest.x="<<dest.x<<"; current_point->y="<<current_point->y<< std::endl;
		std::cout << "v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y<< std::endl;
		 */
		/*if(debug_velocity_){
			std::cout << "(IFFFFF) v_desired_x="<<v_desired_x<< "; dest.x ="<<dest.x <<"; current_point->x="<<current_point->x << std::endl;
			std::cout << "v_desired_y="<<v_desired_y<< "; dest.y ="<<dest.y <<"; current_point->y="<<current_point->y << std::endl;
		}*/

		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		act_module_dist=v_desired;
		//std::cout << "v_desired="<<v_desired<< std::endl;
		/*if(debug_velocity_){
			std::cout << "(if) v_desired="<<v_desired<< std::endl;
			std::cout << "desired_velocity_="<<desired_velocity_ << std::endl;
		}*/


		double desired_vel_act;

		/*if(dist_to_near_goal>d_max_reduce_vel){
			vel_per_ok_in=Far_goal;
		}*/

/*
		switch(vel_per_ok_in)
		{
			case Vel_per:
				std::cout << "case Cplan_local_nav::Vel_per:"<< std::endl;
				desired_vel_act=desired_velocity_;
				break;

			case Near_goal:
				std::cout << "case Cplan_local_nav::Near_goal:"<< std::endl;
				desired_vel_act=desired_velocity_ * dist_to_near_goal / d_max_reduce_vel;
				std::cout << "desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<< std::endl;
				if(desired_vel_act>desired_velocity_){
					desired_vel_act=desired_velocity_;
				}

				break;

			case Far_goal:
			 default :
				 std::cout << "case Cplan_local_nav::Far_goal:"<< std::endl;
				 desired_vel_act=desired_velocity_;
				break;

		}*/
		// debug: debug_sideBySide2019_
//std::cout << "[velocities last step goal] d_max_reduce_vel="<<d_max_reduce_vel<<"; dist_to_near_goal="<<dist_to_near_goal<<"; in_go_with_vel_per"<<in_go_with_vel_per<<"; desired_velocity_="<<desired_velocity_<< std::endl;

		if((dist_to_near_goal<d_max_reduce_vel)&&(!in_go_with_vel_per)){ //robot_person_proximity_distance + 0.5 (TODO: habra que entrar esas distancias para hacerlo bien!)
		//if(!vel_per_ok){ //robot_person_proximity_distance + 0.5 (TODO: habra que entrar esas distancias para hacerlo bien!)

			desired_vel_act=desired_velocity_ * dist_to_near_goal / d_max_reduce_vel;


			if(desired_vel_act>desired_velocity_){
				desired_vel_act=desired_velocity_;
			}

			/*if(desired_vel_act<real_person_vel){
				desired_vel_act=real_person_vel;
			}*/

			//std::cout << "(case near goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<<"; desired_vel_act="<<desired_vel_act<<";  d_max_reduce_vel="<< d_max_reduce_vel<<"; dist_to_near_goal="<<dist_to_near_goal<<";desired_vel_act="<<desired_vel_act<< std::endl;


			if(debug_velocity_file_){
				std::ofstream fileMatlab2;
				fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
				fileMatlab2 << "(case near goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<< "\n";
				fileMatlab2 << "(case near goal) desired_velocity_ * dist_to_near_goal / d_max_reduce_vel="<<desired_velocity_ * dist_to_near_goal / d_max_reduce_vel<< "\n";
				fileMatlab2.close();
			}
			if(debug_velocity_cout_){
				std::cout << "(case near goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<<"; desired_vel_act="<<desired_vel_act<< std::endl;
			}






		}else{

			desired_vel_act=desired_velocity_;//* dist_to_near_goal / d_max_reduce_vel;
			//desired_vel_act=desired_velocity_; //TODO: en teoria para este caso esta es la buena, pero habria que ampliar el margen de diferencia entre angulos de 10 a 15, en el caso companion recto.

			if(debug_velocity_cout_){ //debug_sideBySide2019_
				std::cout << "(case person or far_goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<< std::endl;
			}
			if(debug_velocity_file_){
				std::ofstream fileMatlab2;
				fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
				fileMatlab2 << "(case person or far_goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<< "\n";
				fileMatlab2.close();
				//std::cout << "(case near goal) desired_velocity_="<<desired_velocity_<<"; dist_to_near_goal="<<dist_to_near_goal<< std::endl;

			}

		}


		v_desired_x *= desired_vel_act/v_desired;
		v_desired_y *= desired_vel_act/v_desired;

		if(debug_velocity_cout_){
			std::cout << " v_desired_x="<<v_desired_x<< std::endl;
			std::cout << "v_desired_y="<<v_desired_y<< std::endl;
		}
	}
	else //Sdestination::Stopping
	{
		v_desired_x = 0.0;
		v_desired_y = 0.0;
		// debug_sideBySide2019_
		//std::cout << "//Sdestination::Stopping! v_desired_x="<<v_desired_x<<"; v_desired_y"<<v_desired_y<< std::endl;
	}


	/*if(debug_velocity_){
		std::cout << "current_point->vx="<<current_point->vx<<"; current_point->vy"<<current_point->vy<< std::endl;
		std::cout << "fx="<<social_forces_param->at(0) * (v_desired_x - current_point->vx)<<"; fy"<<social_forces_param->at(0) * (v_desired_y - current_point->vy)<< std::endl;
	}


	if(debug_velocity_){
		std::cout << "(ely) vx_desired=(dest.x - current_point->x)/0.2="<<(dest.x - current_point->x)/0.2<<"; vy_desired=(dest.y- current_point->y)/0.2="<<(dest.y - current_point->y)/0.2<< std::endl;
	}*/

	//std::cout << "k=social_forces_param->at(0)="<<social_forces_param->at(0) <<"; v_desired_x="<<v_desired_x<<"; v_desired_y="<<v_desired_y<<"; current_point->vx="<<current_point->vx<<"; current_point->vy="<<current_point->vy<< std::endl;
	double act_k;
	if(dist_to_near_goal<max_dist_to_near_goal_force_){ // max_dist_to_near_goal_force_=3
		act_k=externa_force_k_near_goal_;//2.7;
		//std::cout << "act_k="<<act_k<<"; max_dist_to_near_goal_force_="<<max_dist_to_near_goal_force_<<"; dist_to_near_goal="<<dist_to_near_goal<< std::endl;
	}else{
		act_k=externa_force_k_far_goal_;//2.3;
		//std::cout << "act_k="<<act_k<<"; max_dist_to_near_goal_force_="<<max_dist_to_near_goal_force_<<"; dist_to_near_goal="<<dist_to_near_goal<< std::endl;
	}
	//act_k=2.3;






	v_desired_robot_x_.push_back(v_desired_x);
	v_desired_robot_y_.push_back(v_desired_y);
	//v_current_robot_x_.push_back(dest.y - current_point->y);
	//v_current_robot_y_.push_back(v_desired);
	v_current_robot_x_.push_back(current_point->vx);
	v_current_robot_y_.push_back(current_point->vy);
	//actual_random_goal_x_.push_back(desired_velocity_);
	actual_random_goal_x_.push_back(dest.x);
	actual_random_goal_y_.push_back(dest.y);
	//actual_random_goal_y_.push_back(dest.x - current_point->x);



    actual_distance_in_x_.push_back(dest.x - current_point->x);
   	actual_distance_in_y_.push_back(dest.y - current_point->y);
	actual_module_distance_.push_back(act_module_dist);
	actual_in_desired_velocity_.push_back(desired_velocity_);

	force_to_goal_ = Sforce(	act_k * (v_desired_x - current_point->vx),
			act_k * (v_desired_y - current_point->vy));
	//std::cout<<"; act_k="<<act_k << "force_to_goal_.fx="<<force_to_goal_.fx<<"; force_to_goal_.fy="<<force_to_goal_.fy<< std::endl;


	return  force_to_goal_;
}



Sforce Cperson_abstract::force_sphe_prob( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
    //uniformly sampling around std
	double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Sforce f;
    Spoint p(interacting_person);
    for( unsigned int i=0; i< 4; ++i )
    {
        f += this->force_sphe( p-Spoint( sqrt(interacting_person.cov_xx())*cos(theta) ,  sqrt(interacting_person.cov_yy())*sin(theta)), social_forces_param, virtual_current_point );
        theta += PI/2.0;
    }
    return f*0.25;
}

Sforce Cperson_abstract::force_sphe_mahalanobis( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
	//TODO la calcula mal
    //Mahalanobis distance to interacting target: gives a relative distance that dpeneds on the covariance matrix
    double d_mah = sqrt( interacting_person.cov_dist( this->current_pointV_ ) );
    double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Spoint p(interacting_person),dt( d_mah*cos(theta) , d_mah*sin(theta) );
	return this->force_sphe( p-dt , social_forces_param, virtual_current_point );
}

Sforce Cperson_abstract::force_sphe_worst( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
        const SpointV* virtual_current_point)
{
    //worst case scenario: the interacting target is in the covariance elipsoid and the nearest point to center.
    double theta = interacting_person.angle_heading_point( this->current_pointV_ );
    Spoint p(this->current_pointV_),dt( interacting_person.cov_xx()*cos(theta) , interacting_person.cov_yy()*sin(theta) );
	return this->force_sphe( p-dt , social_forces_param, virtual_current_point );
}

Sforce Cperson_abstract::get_forces_person(Sforce&  force_to_goal, Sforce& force_int_person, Sforce& force_int_robot, Sforce& force_obstacle ) const
{
	//make sure whenever you use this function to previously calculate the correspondent forces
	force_to_goal = force_to_goal_;
	force_int_person = force_int_person_;
	force_int_robot = force_int_robot_;
	force_obstacle = force_obstacle_;
	return force_to_goal_ + force_int_person_ + force_int_robot_+  force_obstacle_ ;
}

Sforce Cperson_abstract::get_forces_person_companion(Sforce&  force_to_goal, Sforce& force_int_person, Sforce& force_int_robot, Sforce& force_obstacle, Sforce& force_companion ) const
{
	//make sure whenever you use this function to previously calculate the correspondent forces
	//std::cout << " 33333333333 get_forces_person_companion  " << std::endl;
	//force_to_goal_.print();

	force_to_goal = force_to_goal_;
	force_int_person = force_int_person_;
	force_int_robot = force_int_robot_;
	force_obstacle = force_obstacle_;
	force_companion=force_companion_;
	return force_to_goal_ + force_int_person_ + force_int_robot_+  force_obstacle_ + force_companion_;
}
void Cperson_abstract::set_forces_person( Sforce force_to_goal, Sforce force_int_person, Sforce force_int_robot, Sforce force_obstacle )
{
	force_to_goal_ = force_to_goal;
	force_int_person_ = force_int_person;
	force_int_robot_ = force_int_robot;
	force_obstacle_ = force_obstacle;
}
void Cperson_abstract::set_forces_person_companion( Sforce force_to_goal, Sforce force_int_person, Sforce force_int_robot, Sforce force_obstacle , Sforce force_companion )
{
	force_to_goal_ = force_to_goal;
	force_int_person_ = force_int_person;
	force_int_robot_ = force_int_robot;
	force_obstacle_ = force_obstacle;
	force_companion_=force_companion;
	//std::cout << " Set_forces comp! 33333333333 get_forces_person_companion  " << std::endl;
	//	force_to_goal_.print();

}

Sbehavior::Sbehavior( unsigned int id ) : related_person_id(id)
{
	int N = Cperson_abstract::Count_behaviors;
	expectation.resize( N , 1.0/(double)N );
}

Sbehavior::~Sbehavior()
{

}
void Sbehavior::print() const
{
	std::cout << " Behavior to the related person  " << related_person_id <<
			" and expected behaviors = { " << expectation[0] <<
			" , " << expectation[1] << " , " << expectation[2] << " }" << std::endl;
	return;
}


Sforce force_sphe( const SpointV& center_person, const SpointV& interacting_person , const std::vector<double>* social_forces_param )
{
	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	double f,d;
	//geometry calculations
	double dx = center_person.x - interacting_person.x;
	double dy = center_person.y - interacting_person.y;
	double vx = center_person.vx;
	double vy = center_person.vy;
	d = sqrt(dx*dx+dy*dy);
	dx /= d;
	dy /= d;
	//force module calculation
	f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));

	//force direction
	double phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	force.fx = f * dx * anisotropy;
	force.fy = f * dy * anisotropy;
	return force;
}

Sforce force_goal( const Sdestination dest, const SpointV& center_person , const std::vector<double>* social_forces_param )
{
	//setting the target type and its corresponding parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force_to_goal;
	double desired_velocity = center_person.v();


	double v_desired,v_desired_x,v_desired_y;
	if ( dest.type == Sdestination::Map_goal  )
	{
		v_desired_x = dest.x - center_person.x;
		v_desired_y = dest.y - center_person.y;
		v_desired = sqrt(v_desired_x*v_desired_x + v_desired_y*v_desired_y);
		v_desired_x *= desired_velocity/v_desired;
		v_desired_y *= desired_velocity/v_desired;
	}
	else //Sdestination::Stopping
	{
		v_desired_x = 0.0;
		v_desired_y = 0.0;
	}
	force_to_goal = Sforce(	social_forces_param->at(0) * (v_desired_x - center_person.vx),
			social_forces_param->at(0) * (v_desired_y - center_person.vy));
	return  force_to_goal;
}



/* functions person companion (ely) */

Sdestination Cperson_abstract::get_companion_force_goal() const
{
	return robot_companion_dest_;
}

Sforce Cperson_abstract::force_sphe_atractive( const Spoint& interacting_person ,  const std::vector<double>* social_forces_param , const SpointV* virtual_current_point )
{
	//if force is calculated from a propagated pose, then virtual_current_point won't be null.
	//Otherwise, current point is current point class variable
	std::cout << "entering force_sphe (1) (params not transmuted)"<< std::endl;

	const SpointV* current_point;
	if ( virtual_current_point == NULL  )
		current_point = &current_pointV_;
	else
		current_point = virtual_current_point;

	//setting the corresponding SFM parameters:
	// if no forces param provided, exit program
	assert( social_forces_param != NULL );
	assert( social_forces_param->size() >= 5 );

	Sforce force;
	double f,d;
	//geometry calculations
	double dx = current_point->x - interacting_person.x;
	double dy = current_point->y - interacting_person.y;
	double vx = current_point->vx;
	double vy = current_point->vy;
	d = sqrt(dx*dx+dy*dy);
	dx /= d;
	dy /= d;
	//force module calculation
	f = social_forces_param->at(2)* exp((social_forces_param->at(4)-d) / social_forces_param->at(3));
	//force direction
	double phi = diffangle( atan2(vy,vx), atan2(-dy,-dx) );//minus difference vector
	double anisotropy = (social_forces_param->at(1) + (1-social_forces_param->at(1))*(1 + cos(phi))/2 );
	force.fx = -f * dx * anisotropy;
	force.fy = -f * dy * anisotropy;
	return force;
}

void Cperson_abstract::clear_Zanlungo_vectors(){

	//std::cout << "IN clear_Zanlungo_vectors"<< std::endl;

	v_desired_robot_x_.clear(); // estan dentro de person abstract es mas complicado.
	v_desired_robot_y_.clear();
	v_current_robot_x_.clear();
	v_current_robot_y_.clear();
	actual_random_goal_x_.clear();
	actual_random_goal_y_.clear();

    actual_distance_in_x_.clear();
   	actual_distance_in_y_.clear();
	actual_module_distance_.clear();
	actual_in_desired_velocity_.clear();
}

std::vector<double> Cperson_abstract::get_v_desired_robot_x(){
	return v_desired_robot_x_;
}
std::vector<double> Cperson_abstract::get_v_desired_robot_y(){
	return v_desired_robot_y_;
}

std::vector<double> Cperson_abstract::get_v_current_robot_x(){
	return v_current_robot_x_;
}

std::vector<double> Cperson_abstract::get_v_current_robot_y(){
	return v_current_robot_y_;
}

std::vector<double> Cperson_abstract::get_actual_random_goal_x(){
	return actual_random_goal_x_;
}

std::vector<double> Cperson_abstract::get_actual_random_goal_y(){
	return actual_random_goal_y_;
}

//////////

std::vector<double> Cperson_abstract::get_actual_distance_in_x(){
	return actual_distance_in_x_;
}
std::vector<double> Cperson_abstract::get_actual_distance_in_y(){
	return actual_distance_in_y_;
}

std::vector<double> Cperson_abstract::get_actual_module_distance(){
	return actual_module_distance_;
}

std::vector<double> Cperson_abstract::get_actual_in_desired_velocity(){
	return actual_in_desired_velocity_;
}


