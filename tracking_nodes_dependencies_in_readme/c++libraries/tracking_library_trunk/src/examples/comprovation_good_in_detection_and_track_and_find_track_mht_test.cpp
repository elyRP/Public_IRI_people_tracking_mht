//librerias nodo Gonzalo.
#include "prediction.h"
//#include "iri_geometry.h"
#include "peoplePredictionStructs.h"
#include "scene/scene.h"

//librerias mi nodo.
#include "mhtStructs.h"
#include "track.h"
#include "mht.h"

#include <stdio.h> //scanf, printf

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

 //   Ctrack::detection_type type = Ctrack::Laser; //de momento no hace falta, porque el tipo por defecto es laser y en la función al no ponerlo me coge la muestra como laser.

    //Ctrack track1(1);
	//Cmht mht1;

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

    //coger laser detection points.
    printf ("Enter laser_detection points: \n");

        for (unsigned int i=0; i< laser_size; ++i )
        {
        	double x, y;

        	if ((i==0)||(i==1)){

        		scanf("%lf\n\n\n\n",&x);
        		scanf("%lf\n\n\n\n",&y);

        	/*}else if(i<=21){
        		scanf("%lf\n\n\n",&x);
        		scanf("%lf\n\n\n",&y);*/

        	}else if(i==(laser_size-1)){
        		scanf("%lf\n",&x);
        		scanf("%lf",&y);

        	}else if((i>=22)||(i<(laser_size-1))){
        		scanf("%lf\n\n",&x);
        		scanf("%lf\n\n",&y);
        	}

//        	std::cout<< "\n x:" << x<< std::endl;
//        	std::cout<< "y:" << y<< std::endl;
//        	std::cout<< "size = " << laser_detection_x.size()<< std::endl;

        	laser_detection_x.push_back(x);
        	laser_detection_y.push_back(y);
//        	std::cout<< "\n laser_detection_x_vector:\n" << laser_detection_x[i]<< std::endl;
//        	std::cout<< "\n laser_detection_y_vector:\n" << laser_detection_y[i]<< std::endl;

        }

        //coger laser prediction points.
        printf ("Enter laser_prediction points: ");

        for (unsigned int i=0; i< laser_size; ++i )
                {
                	double x, y;

                	if ((i==0)||(i==1)){
                		scanf("%lf\n\n\n\n",&x);
                		scanf("%lf\n\n\n\n",&y);

                	/*}else if(i<=21){
                		scanf("%lf\n\n\n",&x);
                		scanf("%lf\n\n\n",&y);*/

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
//                	std::cout<< "\n laser_prediction_x_vector:\n" << laser_prediction_x[i]<< std::endl;
 //              	std::cout<< "\n laser_prediction_y_vector:\n" << laser_prediction_y[i]<< std::endl;

                	//laser.push_back( Spoint(x,y) );

                }

//comprovar que verdaderamente va bien pq se hace dentro del tracker o en el mht lo de comprovar que haya deteccion laser
        //y su distancia sea menor que el humbral de 2metros y que el track este inicializado. (main para probar la clase track)


        for (unsigned int i=0; i< laser_size; ++i )
        {

        	//hacer los structs con las detecciones=spoint i los tracks=spose

        	//deteccion1!

        	int type=0; //type laser.
        	double probability=0.0;
        	Sdetection detection1(laser_detection_x[i],laser_detection_y[i],type,probability);

        	//track_in_1

        	double time_stamp=double(i)*0.2;
        	SdetectionObservation track_in_1( 1 , time_stamp , laser_prediction_x[i], laser_prediction_y[i],0.0, 0.0);
        	Cscene escena;

        	//track1 mio salida.

        	Strack track1; //salida!

        	//printf detecciones entrada.
        	printf("\n \n detection1.x:%f",detection1.x);
        	printf("\n \n detection1.y:%f",detection1.y);
        	printf("\n \n type:%d",detection1.type);
        	printf("\n \n probability:%f",detection1.probability);
        	//printf tracks entrada.
        	track_in_1.print();


//vectores detecciones y tracks entrada.
//vector dets
        	 std::vector<Sdetection> obs_test;
        	 obs_test.push_back(detection1);
//vector tracks
        	 std::vector<SdetectionObservation> track_test;
        	 track_test.push_back(track_in_1);
        	 escena.update_scene(track_test);

 //printf valores vectores entrada!!!
        	 //detections
        	 Sdetection obs1=obs_test[0];
        	 obs1.print_spoint_DET();
        	 std::cout<< " type:" <<  obs1.type << std::endl;
        	 std::cout<< " probability detection:" <<  obs1.probability << std::endl;
        	 //tracks

        	 escena.print();

        	 //track_vec.print();


  // comprovacion buen uso find person!!!      find_person(int id , list<Cperson>::iterator& person)
        	 printf("track 1 find!!!");
        	 //SdetectionObservation track_find;

        	 list<Cperson>::iterator person_obj;
        	 escena.find_person(1,person_obj);
        	 Spose person = person_obj->get_current_pose();
        	 person.print();
        	 person_obj->print();

        	printf("\n \n ITERATION:%u \n",i);


        	//MI CLASE!!!!

        	//Cmht mh1;
        	//mh1.mht_update(obs_test,track_test);
        	//mh1.return_results();

        }

}
