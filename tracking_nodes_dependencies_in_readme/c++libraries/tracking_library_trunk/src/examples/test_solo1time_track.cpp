#include "track.h"
#include "prediction.h"
//#include "iri_geometry.h"

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

  //Ctrack track;


  //test methods

	Eigen::MatrixXd	H_(2,2);
    H_(0,0)=1;
    H_(0,1)=0;
    H_(1,0)=0;
    H_(1,1)=1;
  //  Eigen::MatrixXi matrix_hipotesis_anterior_(1,1);

/*double a;
a=1.0;
double b;
b=0.0;*/
	// inicio MAIN
    //inicialization_laser_=0; // COMO HAGO ESTOOOOOOOOOOOOOOOOOOO!!!, que la primera vez sea zero y luego al cambiarla no me la vuelva a cambiar cuando haga lo del tracker...

 for(double r=1.0 ;r<10.0 ;r++)
 {
/*	double y;
	y=r*a+b;
	double y_point;
	y_point= y + (rand() % 1)/2;
	double x_point;
	x_point= r + (rand() % 1)/2;

	double y_pose;
	y_pose= y + (rand() % 1)/2;
	double x_pose;
	x_pose= r + (rand() % 1)/2;

	 std::cout<< "\n y_point=" << y_point<< std::endl;
	 std::cout<< "\n x_point=" << x_point<< std::endl;
	 std::cout<< "\n y_pose=" << y_pose<< std::endl;
	 std::cout<< "\n x_pose=" << x_pose<< std::endl;
*/

    Ctrack::detection_type type = Ctrack::Laser;
    Spoint detection_n(20.0,30.0);//Spoint::Spoint( double x_ , double y_ )
    Spose track_n(22.0 , 31.0);
    Ctrack track1(1);

    track1.set_tracks_and_detections(detection_n, track_n, type);

    if((track1.have_detection_laser_==1) && (track1.inicialization_laser_==0)) //hay detección laser!
    {
    	track1.ini_track();
    	track1.inicialization_laser_=1;
    }

    if(track1.inicialization_laser_==1)  //si el laser se ha inicializado!!!
    {

    	track1.update_track();

    }

 }
	//fin MAIN






}
