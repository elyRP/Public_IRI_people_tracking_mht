#include "track.h"
#include "prediction.h"
//#include "iri_geometry.h"
#include "peoplePredictionStructs.h"
#include <stdio.h> //scanf, printf

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

  //Ctrack track;


  //test methods


 //   Ctrack::detection_type type = Ctrack::Laser; //de momento no hace falta, porque el tipo por defecto es laser y en la función al no ponerlo me coge la muestra como laser.

    Ctrack track1(1);


    unsigned int laser_size=0; //laser_size==num iteration=num detection one person, time.

    printf ("Enter laser size: \n");
    scanf("%u",&laser_size);

    std::vector<double> laser_detection_x; //mirar!!!
    std::vector<double> laser_detection_y; //mirar!!!
    laser_detection_x.reserve(laser_size);
    laser_detection_y.reserve(laser_size);
    std::vector<double> laser_prediction_x; //mirar!!!
    std::vector<double> laser_prediction_y; //mirar!!!
    laser_prediction_x.reserve(laser_size);
    laser_prediction_y.reserve(laser_size);
   // Eigen::MatrixXd laser_prediction(2,laser_size);

    //coger laser detection points.
    printf ("Enter laser_detection points: \n");



        for (unsigned int i=0; i< laser_size; ++i )
        {
        	double x, y;

        	if ((i==0)||(i==1)){
        		scanf("%lf\n\n\n\n",&x);
        		scanf("%lf\n\n\n\n",&y);

        	}else if(i<=21){
        		scanf("%lf\n\n\n",&x);
        		scanf("%lf\n\n\n",&y);

        	}else if(i==(laser_size-1)){
        		scanf("%lf\n",&x);
        		scanf("%lf",&y);

        	}else if((i>=22)||(i<(laser_size-1))){
        		scanf("%lf\n\n",&x);
        		scanf("%lf\n\n",&y);

        	}

  //      	std::cout<< "\n x:" << x<< std::endl;
  //      	std::cout<< "y:" << y<< std::endl;
 //       	std::cout<< "size = " << laser_detection_x.size()<< std::endl;

        	laser_detection_x.push_back(x);
        	laser_detection_y.push_back(y);
//        	std::cout<< "\n laser_detection_x_vector:\n" << laser_detection_x[i]<< std::endl;
 //       	std::cout<< "\n laser_detection_y_vector:\n" << laser_detection_y[i]<< std::endl;



        }

        //coger laser prediction points.
        printf ("Enter laser_prediction points: ");

        for (unsigned int i=0; i< laser_size; ++i )
                {
                	double x, y;

                	if ((i==0)||(i==1)){
                		scanf("%lf\n\n\n\n",&x);
                		scanf("%lf\n\n\n\n",&y);

                	}else if(i<=21){
                		scanf("%lf\n\n\n",&x);
                		scanf("%lf\n\n\n",&y);

                	}else if(i==(laser_size-1)){
                		scanf("%lf\n",&x);
                		scanf("%lf",&y);

                	}else if((i>=22)||(i<(laser_size-1))){
                		scanf("%lf\n\n",&x);
                		scanf("%lf\n\n",&y);

                	}

 //               	std::cout<< "\n x:" << x<< std::endl;
 //               	std::cout<< "y:" << y<< std::endl;

                	laser_prediction_x.push_back(x);
                	laser_prediction_y.push_back(y);
 //               	std::cout<< "\n laser_prediction_x_vector:\n" << laser_prediction_x[i]<< std::endl;
 //               	std::cout<< "\n laser_prediction_y_vector:\n" << laser_prediction_y[i]<< std::endl;

                	//laser.push_back( Spoint(x,y) );

                }




        ///////////////


for (unsigned int i=0; i< laser_size; ++i )
{
	Spoint detection_n(laser_detection_x[i], laser_detection_y[i]  );
	Spoint track_n( laser_prediction_x[i],laser_prediction_y[i] );
	Spoint kalmanstate_modifi;
	printf("\n \n ITERATION:%u \n",i);


	track1.set_tracks_and_detections( detection_n, track_n);//type laser by default
	//printf("\n \n detection_n.x:%f",detection_n.x);
	//printf("\n \n detection_n.y:%f",detection_n.y);

	//printf("\n \n track_n.x:%f",track_n.x);
	//printf("\n \n track_n.y:%f",track_n.y);




    if ( track1.is_initialized() )  //si el laser se ha inicializado!!!
	{


    	//cálculo distancias.
        track1.calculation_of_distances_detectors();


        if( (track1.have_detection_laser()==false) || (track1.get_laser_distance()>track1.get_threshold_distance()) )
	 	{

	    //caso no tengo detección de ninguno de los tres!!! (ahora mismo 2, pq solo será vision y laser!!!
        	track1.update_track(track_n);
        	printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter() );
        	printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );
        	kalmanstate_modifi=track1.get_kalman_update();
        	printf("\n \n kalmanstate_modifi.x:%f",kalmanstate_modifi.x);
        	printf("\n \n kalmanstate_modifi.y:%f",kalmanstate_modifi.y);
        	printf( " \n \n \n" );

	 	}
	 	else if( (track1.have_detection_laser()==true) && (track1.get_laser_distance()<=track1.get_threshold_distance()) )
	 	{

	 		track1.update_track(detection_n, track_n);  //TODO: cambiar el detection_n y track_n, pq serán solo los del cluster, a d<=!!!
	 		printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter() );
	 		printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );
	 		kalmanstate_modifi=track1.get_kalman_update();
	 		printf("\n \n kalmanstate_modifi.x:%f",kalmanstate_modifi.x);
	 		printf("\n \n kalmanstate_modifi.y:%f",kalmanstate_modifi.y);
	 		printf( " \n \n \n" );

	 	}
	 }



    if( track1.have_detection_laser() &&  !track1.is_initialized() ) //hay detección laser!
    {

    	track1.ini_track();
    	track1.set_initialized();

    	printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter());
    	printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );

    }





 }
	//fin MAIN






}
