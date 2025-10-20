#include "people_tracking_mht_alg.h"

PeopleTrackingMhtAlgorithm::PeopleTrackingMhtAlgorithm(void)
{
  myPeopleTrackingMht = new Cmht();
}

PeopleTrackingMhtAlgorithm::~PeopleTrackingMhtAlgorithm(void)
{
  delete myPeopleTrackingMht;
}

void PeopleTrackingMhtAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  myPeopleTrackingMht->set_threshold_distance(new_cfg.association_threshold);
  myPeopleTrackingMht->set_threshold_confirmation_track(new_cfg.confirmation_threshold);
  myPeopleTrackingMht->set_threshold_no_deteccion(new_cfg.deletion_threshold);
  myPeopleTrackingMht->set_laser_Pd(new_cfg.laser_probability);
  myPeopleTrackingMht->set_laser_beta_ft(new_cfg.laser_false_alarm);
  myPeopleTrackingMht->set_laser_beta_nt(new_cfg.laser_new_track);
  myPeopleTrackingMht->set_laser_beta_no_detection(new_cfg.laser_no_detection);
  myPeopleTrackingMht->set_laser_beta_track_no_confirmed(new_cfg.laser_no_confirmed);
  myPeopleTrackingMht->set_increment_iteration_track_no_confirmed(new_cfg.laser_no_confirmed_iteration);
  myPeopleTrackingMht->set_covariance_no_track(new_cfg.covariance_no_track_conf);

 myPeopleTrackingMht->set_generate_database_companion(new_cfg.generate_database_companion);
 myPeopleTrackingMht->set_fuse_tibi_and_teo_tracks(new_cfg.fuse_tibi_and_teo_tracks);
 myPeopleTrackingMht->set_debug_fuse_tibi_and_teo_tracks(new_cfg.debug_fuse_tibi_and_teo_tracks);

 myPeopleTrackingMht->set_mht_velocity_margin(new_cfg.mht_velocity_margin);

	myPeopleTrackingMht->set_augment_covariance_track(new_cfg.augment_covariance_track);
  this->unlock();
}

// PeopleTrackingMhtAlgorithm Public API

bool PeopleTrackingMhtAlgorithm::iteration(const std::vector<Sdetection> &detections, std::vector<Strack> &tracks, std::vector<Strack> &predictions, std::vector<Exitcluster> &clusters, Codometry &odom, Cvelocity &velocity,std::vector<exitGroup> &groups, bool &local_tracker)
{

  //ROS_INFO(" inicio (iteration) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
  //myPeopleTrackingMht->mht_Cscene_se_in_tracks();

	// ROS_INFO("PeopleTrackingMhtAlgorithm::iteration");
	// ROS_INFO("PeopleTrackingMhtAlgorithm::iteration %d", myDetections.size());
  std::vector<Sdetection> myDetections= detections;
  

    //ROS_INFO(" (2) inicio (iteration) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
    //myPeopleTrackingMht->mht_Cscene_se_in_tracks();
      
    if(local_tracker){  
    	 ROS_INFO("PeopleTrackingMhtAlgorithm::ODOMETRIA");
    	Codometry myOdom=odom;
    	myPeopleTrackingMht->mht_odom_update(myOdom);
     
    	Cvelocity myVelocity=velocity;
    	myPeopleTrackingMht->get_total_velocity(myVelocity);
    }



	  //ros::Time now    = ros::Time::now();
    //ROS_INFO( "time of processing mht_update= %f", now.toSec());
   /* if(!detections.empty()){
      ROS_INFO( "myDetections[0].time_stamp= %f",detections[0].time_stamp);
    }*/

   //ROS_INFO( "save_time_stamp_odometry_to_tracker_without_det_= %f",save_time_stamp_odometry_to_tracker_without_det_);
    //ROS_INFO(" ANTES (myPeopleTrackingMht->mht_update) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
   // myPeopleTrackingMht->mht_Cscene_se_in_tracks();

   myPeopleTrackingMht->mht_update(myDetections, save_time_stamp_odometry_to_tracker_without_det_); //

   // ROS_INFO("TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
   // myPeopleTrackingMht->mht_Cscene_se_in_tracks();
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: mht_update done! Detections sent!");
    
    tracks = myPeopleTrackingMht->get_persons_mht();
     
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: get_persons_mht done! %d Tracks received!", tracks.size());
    
    predictions = myPeopleTrackingMht->get_propagations_mht();
     
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: get_propagations_mht done! %d propagations received!", tracks.size());
    
    //Function for clusters (cruces personas, crossing people)
    
    clusters = myPeopleTrackingMht->get_clusters_mht();
    
    groups = myPeopleTrackingMht->get_groups_mht();
    
    if(local_tracker){
    	odom.reset_odometry();
    }
  
  return true;
}
