#ifndef iri_geometry_H
#define iri_geometry_H

#include <vector>
#include <iostream>
const static double PI = 3.1415926535897932384626433;

/**
 *	\brief iri_geometry library
 *
 *	It consists of a set of data structures, similar to structs, where all values are accessible from
 *	the outside and a set of simple methods for each geometry primitive.
 *
 *
 *  Note: In general, these classes are designed as simple data structures with methods associated
 *  By no means they are intended to be created as pointers and new or used polymorphically.
 *  No dynamical binding of the functions
 *  has been designed (only statical, beware on how you declare them), so they won't work!!!!
 *  Simply use it as data types, as can be seen in test/test_geometry.
 *
  */


/**
 *	\brief Sforce Struct
 *
 *	data struct describing a 2d force (fx,fy)
 *	and their corresponding methods
 *	TODO: add force covariances
 */
class Spoint;
class Sforce
{
  public:
	double fx, fy;
	Sforce();
	Sforce(double fx_ , double fy_);
	double module( void );
	double module( double r2 );//r2 if the square factor to compate fx and r2*fy
	double module2( void );
	double module2( double r2 );
	Sforce operator+ (Sforce f2) const;
	void sum(Sforce f2);//a method similar to +=
	Sforce& operator+= (Sforce f2);
	Sforce operator* (double k) const;
	double operator* (Spoint dr) const;
    Sforce operator- (Sforce f2) const;
	void print() const;
};

/**
 *	\brief Spoint Struct
 *
 *	data struct describing a 2d point (x,y).
 */
class Spoint
{
  public:
	double x;
	double y;
	double time_stamp; //sec ;
	Spoint();
	Spoint( double x_ , double y_ , double time_stamp_=0.0) ;
	Spoint operator+ (Spoint p2) const ;
	Spoint operator- (Spoint p2) const ;
	Spoint operator* (double k) const ;
	double distance(Spoint p2 = Spoint() ) const;
	double distance2( Spoint p2 = Spoint() ) const;
	Spoint propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	char* itoa(int a, char b[]);

	/* Get number of digits in an integer */
	int getNumofDecDigit(int Number);

	/* get a character from  digit input number */
	char getStrChar(int digit);
	void print() const;
};

/**
 *	\brief Spoint_cov Struct
 *
 *	data struct describing a 2d point (x,y). and its covariance. This covariance is represented as
 *	a vector of 4 elements
 */
class Spoint_cov : public Spoint
{
  public:
	std::vector<double> cov; // size = 16= 4x4
	Spoint_cov();
	Spoint_cov( double x_ , double y_ , double time_stamp_=0.0, const std::vector<double>& cov_= std::vector<double>() );
	double cov_dist( Spoint p2=Spoint() ) const;
    Spoint_cov operator+ (Spoint_cov p2) const ;
    Spoint_cov operator- (Spoint_cov p2) const ;
    Spoint_cov operator* (double k) const ;
    double cov_xx() const;
    double cov_yy() const;
    double cov_xy() const;
	Spoint_cov propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	void print() const;
};


/**
 *	\brief SpointV Struct
 *
 *	data struct describing a 2d point (x,y) and its velocities (derivate)
 */
class SpointV : public Spoint
{
  public:
	double vx;
	double vy;
	SpointV();
	SpointV( double x_ , double y_, double time_stamp_=0.0, double vx_=0.0, double vy_=0.0 ) ;
	SpointV operator+ (SpointV p2) const ;
	SpointV operator- (SpointV p2) const ;
	SpointV operator* (double k) const ;
	SpointV propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0) const;
	SpointV propagate2_comp_only_vel( double dt , Sforce f= Sforce(), double desired_velocity=2.0, double distance_person_t_x=0.0,double distance_person_t_y=0.0,double vel_person_x=0.0,double vel_person_y=0.0) const;
	double orientation() const;
	double v() const;
	void norm_v(double v);
	double angle_heading_point( Spoint p2 ) const;
	void print() const;
};

/**
 *	\brief SpointV Struct
 *
 *	data struct describing a 2d point (x,y) and its velocities (derivate)
 */
class SpointV_cov : public SpointV
{
  public:
	std::vector<double> cov;
	SpointV_cov();
	SpointV_cov(double x_ , double y_, double time_stamp_=0.0, double vx_=0.0, double vy_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	SpointV_cov(SpointV p,	const std::vector<double>& cov_= std::vector<double>());
	SpointV_cov(Spoint_cov p );
	//void SpointV_cov_con_Spose(Spose p);
	//SpointV_cov(SpointV_cov& p );
	double cov_dist( Spoint p2=Spoint() ) const;
	double cov_dist( Spoint p2 , double &det) const;
	double cov_distV( SpointV_cov p2, double &distV, double &distxv  ) const;
    SpointV_cov operator+ (SpointV_cov p2) const ;
    SpointV_cov operator- (SpointV_cov p2) const ;
    SpointV_cov operator* (double k) const ;
    SpointV_cov propagate( double dt , Sforce f = Sforce(), double desired_velocity=2.0 ) const;
    double cov_xx() const;
    double cov_yy() const;
    double cov_xy() const;
    Spoint_cov toSpoint_cov() const;
    void print() const;
};
/**
 *	\brief Spose Struct
 *
 *	data struct describing a pose consisting of position, velocity
 *	and time.
 */
class Spose : public Spoint
{
  public:
	double theta;
	double v;
	double w;
	Spose();
	Spose( double x_ , double y_ , double time_stamp_=0.0 , double theta_=0.0 ,  double v_=0.0, double w_=0.0);
	Spose operator+ (Spose p2) const;
	Spose operator- (Spose p2) const;
	Spose operator* (double k) const;
	double distance(Spoint p2) const;
	double angle_heading_pose( Spose p2 ) const;
	double social_distance( Spose p2 ) const;
	void print() const;
};
/**
 *	\brief SposeCov Struct
 *
 *	data struct describing a pose and its covariance (only in position)
 *	It also supports distance to other pose or distances-based on covariances
 */
class Spose_cov : public Spose
{
  public:
	std::vector<double> cov;
	Spose_cov();
	Spose_cov( double x_ , double y_ ,
			double time_stamp_=0.0 ,double theta_=0.0 ,  double v_=0.0, double w_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	Spose_cov( Spose pose_, const std::vector<double>& cov_= std::vector<double>());
	Spose_cov( SpointV_cov point );//conversion from point to pose
	double distance( Spose_cov p2 ) const;
	double cov_dist( Spose p2 ) const;
	double cov_dist( Spose_cov p2 ) const;
};

/**
 *	\brief Sdestination Struct
 *
 *	data struct describing the destination information:
 *	position, prior probability and type{fixed, temporal}
 */
class Sdestination : public Spoint
{
  public:
	enum destination_type { Map_goal=0 , Stopping, Uncertain};
	Sdestination( int id_=0, double x_=0.0 , double y_=0.0 ,
			double prob_=0.0 ,
			destination_type type_=Map_goal ,
			std::vector<int> neighbours_ids_ = std::vector<int>());
	//Sdestination( Sdestination dest );
	~Sdestination();
	int id;
	double prob;
	destination_type type;
	std::vector<int> neighbours_ids;
	bool operator== (Sdestination d2) const;
	void print() const;

	void sed_dest_type(destination_type type_in){
		type=type_in;
	}
};

/**
 *
 *  \brief detectionObservation Struct to be deprecated... pass a vector of Spoint/Pose and
 *  a vector of id's TODO
 *
 */

class SdetectionObservation : public SpointV_cov
{
  public:
	int id;
	SdetectionObservation();
	SdetectionObservation( int id_ , double time_stamp_=0.0, double x_=0.0, double y_=0.0, double vx_=0.0, double vy_=0.0,
			const std::vector<double>& cov_= std::vector<double>());
	SdetectionObservation( int id_ , SpointV_cov pointV_cov_);
	void print() const;
};



double diffangle(double alpha , double beta);
inline bool is_nan(double x) { return x != x;}

class SsavePath_cost_and_values
{
  public:
	int end_id_path;
	std::vector<Spoint> path_positions;
	std::vector<unsigned int> all_ids_path_positions;
	std::vector<std::vector<double> > total_costs_of_each_path_position;


	SsavePath_cost_and_values(int in_end_id_path=0, std::vector<Spoint> in_path_positions=std::vector<Spoint>(), std::vector<unsigned int> in_all_ids_path_positions=std::vector<unsigned int>(), std::vector<std::vector<double> > in_total_costs_of_each_path_position=std::vector<std::vector<double> >());
	void set_end_id_path(int in_end_id_path){end_id_path=in_end_id_path;}
	void set_path_positions(std::vector<Spoint> in_path_positions){path_positions=in_path_positions;};
	void set_all_ids_path_positions(std::vector<unsigned int> in_all_ids_path_positions){all_ids_path_positions=in_all_ids_path_positions;};
	void set_total_costs_of_each_path_position(std::vector<std::vector<double> > in_total_costs_of_each_path_position){

		total_costs_of_each_path_position=in_total_costs_of_each_path_position;
		//std::cout << " ENTRO EN: set_total_costs_of_each_path_position; total_costs_of_each_path_position.size()="<<total_costs_of_each_path_position.size()<< std::endl;
	};
	void print() const;
};

class SsavePath_multiple_paths_and_best_path // habra un struct de esto por cada iteración.
{
  public:
	double time_act;
	double iter_act;
	int number_of_total_paths;
	std::vector<SsavePath_cost_and_values> nondominated_paths;
	SsavePath_cost_and_values best_path;
	std::vector<double> means;
	std::vector<double> stds;

	SsavePath_multiple_paths_and_best_path(double in_time_act=0.0, double in_iter_act=0.0, int in_number_of_total_paths=0.0,std::vector<SsavePath_cost_and_values> in_nondominated_paths=std::vector<SsavePath_cost_and_values>(), SsavePath_cost_and_values in_best_path=SsavePath_cost_and_values(),std::vector<double> in_means=std::vector<double>(),std::vector<double> in_stds=std::vector<double>());
	void set_time_act(double in_time_act){time_act=in_time_act;};
	void set_iter_act(double in_iter_act){iter_act=in_iter_act;};
	void set_number_of_total_paths(int in_number_of_total_paths){
		number_of_total_paths=in_number_of_total_paths;
		//std::cout << " ENTRO EN: set_number_of_total_paths; number_of_total_paths="<<number_of_total_paths<< std::endl;
	};
	void set_nondominated_paths(std::vector<SsavePath_cost_and_values> in_nondominated_paths){

		nondominated_paths=in_nondominated_paths;
		//std::cout << " ENTRO EN: set_nondominated_paths; nondominated_paths.size()="<<nondominated_paths.size()<< std::endl;

	};
	std::vector<SsavePath_cost_and_values> get_nondominated_paths(){ return nondominated_paths;};
	void set_best_path(SsavePath_cost_and_values in_best_path){
		best_path=in_best_path;
		//std::cout << " ENTRO EN: set_best_path; best_path.total_costs_of_each_path_position.size()="<<best_path.total_costs_of_each_path_position.size()<< std::endl;
	};
	void set_means(std::vector<double> in_means){means=in_means;};
	void set_stds(std::vector<double> in_stds){stds=in_stds;};
	void print() const;
};


class get_path_learning // habra un struct de esto por cada iteración.
{
    public:
	std::string addPath(std::string a, std::string c);
	std::vector<std::string> folderStream(std::string PATH);

	void pathGeneratoraux(std::string path, int level, std::vector<std::string> &paths);
	std::vector<std::string> pathGenerator(std::string path);


};

class peopleOtherPersonCompanionInGroup
{
  public:
	unsigned int real_person_id_;
	unsigned int actual_new_id_in_people_simulation_;
	peopleOtherPersonCompanionInGroup( unsigned int in_real_person_id , unsigned int in_actual_new_id_in_people_simulation);
	void print() const;
};

#endif
