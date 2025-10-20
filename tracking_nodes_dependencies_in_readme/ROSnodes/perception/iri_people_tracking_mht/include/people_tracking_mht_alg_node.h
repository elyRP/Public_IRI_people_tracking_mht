// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _people_tracking_mht_alg_node_h_
#define _people_tracking_mht_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "people_tracking_mht_alg.h"

#include <tf/transform_listener.h>

// [publisher subscriber headers]
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <iri_perception_msgs/detectionArray.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PeopleTrackingMhtAlgNode : public algorithm_base::IriBaseAlgorithm<PeopleTrackingMhtAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher tracksMarkers_publisher_;
    visualization_msgs::MarkerArray MarkerArray_msg_;
    visualization_msgs::Marker id_track_marker_;
    visualization_msgs::Marker id_cluster_marker_;
    visualization_msgs::Marker cov_track_marker_;
    visualization_msgs::Marker cov_track_marker_head_;
    visualization_msgs::Marker cov_track_marker_arm1_;
    visualization_msgs::Marker cov_track_marker_arm2_;
    visualization_msgs::Marker velocity_track_marker_;
    visualization_msgs::Marker cov_detection_marker_;
    visualization_msgs::Marker cov_prediction_marker_;
    visualization_msgs::Marker cluster_marker_;
    visualization_msgs::Marker group_marker_;

    
    ros::Publisher tracks_publisher_;
    iri_perception_msgs::detectionArray trackingArray_msg_;

    // [subscriber attributes]
    ros::Subscriber odom_for_medium_velocity_subscriber_;
    void odom_for_medium_velocity_callback(const nav_msgs::Odometry::ConstPtr& msg);
    CMutex odom_for_medium_velocity_mutex_;
    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    CMutex odom_mutex_;
    ros::Subscriber detections_subscriber_;
    void detections_callback(const iri_perception_msgs::detectionArray::ConstPtr& msg);
    CMutex detections_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    std::string fixed_frame;
	tf::TransformListener tf_;

    std::vector<Sdetection> detections;
    std::vector<Strack>     tracks;
    std::vector<Strack> predictions;   
    std::vector<Exitcluster> clusters;
    std::vector<exitGroup> groups;
    Codometry odom;
    Cvelocity  velocity;
    bool cargo_ANTS;
    bool first_odometry;
    bool first_odometry_total;
    bool detectionsReceived;
    std::string frame_id;
    std::string robot_frame;
    double total_time;
    double total_time_ant;
    std::vector<std::vector<Strack>> tracks_path;
    double first_time;


   	bool tf_to_eigen_odom(const tf::Transform& tf_odom_pose_in, Eigen::Matrix4d& act_odom_matrix_tf, double dt); //for odometry 
    void ini_markers();
    void fillTracksMsg();
    void publishMarkers();
    void fill_my_covariance_marker_track( unsigned int i );
    void fill_my_covariance_marker_track_head( unsigned int i );
    void fill_my_covariance_marker_track_arm1( unsigned int i );
    void fill_my_covariance_marker_track_arm2( unsigned int i );
    void fill_my_covariance_marker_detection( unsigned int i );
    void fill_my_covariance_marker_prediction( unsigned int i );
    void fill_my_marker_cluster( unsigned int i );
    void fill_my_marker_group( unsigned int i );
    tf::TransformListener tf_listener_;

    bool id_markers, id_cluster_markers, cov_markers, vel_markers, covdet_markers, covpred_markers, clust_markers,group_markers, cov_markers_head, cov_markers_arm1,cov_markers_arm2,track_path;
    bool local_tracker;

    double max_dist_to_detect_people_;
    double robot_actual_pose_x_,robot_actual_pose_y_;


  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PeopleTrackingMhtAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PeopleTrackingMhtAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
