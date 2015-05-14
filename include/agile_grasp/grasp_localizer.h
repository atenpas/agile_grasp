/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
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

#ifndef GRASP_LOCALIZER_H_
#define GRASP_LOCALIZER_H_

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <vector>

#include <agile_grasp/CloudSized.h>
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>
#include <agile_grasp/grasp_hypothesis.h>
#include <agile_grasp/handle.h>
#include <agile_grasp/localization.h>
#include <agile_grasp/rotating_hand.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/** GraspLocalizer class
 *
 * \brief Repeatedly search for antipodal grasps in point clouds.
 * 
 * This class is a ROS node that repeatedly takes a point cloud from a given ROS topic as input, 
 * searches for antipodal grasps in that cloud, and publishes the results on a ROS topic. Incoming 
 * point cloud messages can be of two different types: (1) standard sensor_msgs/PointCloud2, (2) 
 * agile_grasp/CloudSized messages. The second type is for two point clouds send in the same 
 * message (see msg/CloudSized.msg or rosmsg agile_grasp/CloudSized).
 * 
*/
class GraspLocalizer
{
public:

  /**
   * \brief Parameters for hand search and handle search.
  */
  struct Parameters
  {
    /** hand search parameters */
    int num_threads_;
    int num_samples_;
    int num_clouds_;    
    Eigen::Matrix4d cam_tf_left_;
    Eigen::Matrix4d cam_tf_right_;
    Eigen::VectorXd workspace_;
    
    /** hand geometry parameters */
    double finger_width_;
    double hand_outer_diameter_;
    double hand_depth_;
    double hand_height_;
    double init_bite_;
        
    // handle search parameters
    int min_inliers_;
    
    // visualization parameters
		int plotting_mode_;
		double marker_lifetime_;
  };
  
  /**
	 * \brief Constructor.
	 * \param node the ROS node
	 * \param cloud_topic the ROS topic that contains the input point cloud
	 * \param cloud_frame the coordinate frame of the point cloud
	 * \param cloud_type the type of the point cloud message (see constants for input point cloud types)
	 * \param svm_file_name the location and filename of the SVM
	 * \param params a set of parameters for hand search and handle search
	*/
  GraspLocalizer(ros::NodeHandle& node, const std::string& cloud_topic, 
    const std::string& cloud_frame, int cloud_type, const std::string& svm_file_name,  
    const Parameters& params);
  
  /**
   * \brief Destructor.
  */
  ~GraspLocalizer() { delete localization_; }
  
  /**
   * \brief Repeatedly localize grasps in the input point cloud.
  */
  void localizeGrasps();

private:
	
	/**
	 * \brief Callback function for the ROS topic that contains the input point cloud.
	 * \param msg the incoming ROS message (of type sensor_msgs/PointCloud2)
	*/
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  
  /**
	 * \brief Callback function for the ROS topic that contains the input point cloud.
	 * \param msg the incoming ROS message (of type agile_grasp/CloudSized)
	*/
  void cloud_sized_callback(const agile_grasp::CloudSized& msg);
  
  /**
	 * \brief Create a grasps message from a list of handles. The message consists of all the grasps 
	 * contained in the handles.
	 * \param handles the set of handles from which the grasps message is created
	*/
  agile_grasp::Grasps createGraspsMsgFromHands(const std::vector<Handle>& handles);
  
  /**
	 * \brief Create a grasps message from a list of handles. The message consists of the "average" 
	 * handle grasps.
	 * \param handles the set of handles from which the grasps message is created
	*/
  agile_grasp::Grasps createGraspsMsg(const std::vector<Handle>& handles);
  
  /**
	 * \brief Create a grasp message from a handle.
	 * \param handles the handle from which the grasp message is created
	*/
  agile_grasp::Grasp createGraspMsg(const Handle& handle);
  
  /**
	 * \brief Create a grasp message from a list of grasp hypotheses.
	 * \param hands the set of grasp hypotheses from which the grasps message is created
	*/
  agile_grasp::Grasps createGraspsMsg(const std::vector<GraspHypothesis>& hands);
  
  /**
	 * \brief Create a grasp message from a grasp hypothesis.
	 * \param hand the grasp hypothesis from which the grasp message is created
	*/
  agile_grasp::Grasp createGraspMsg(const GraspHypothesis& hand);
  
  std::string svm_file_name_; ///< the location and filename of the SVM
  std::string cloud_frame_; ///< the coordinate frame of the point cloud
  PointCloud::Ptr cloud_left_, cloud_right_; ///< the point clouds
  ros::Subscriber cloud_sub_; ///< the subscriber for the point cloud topic
  ros::Publisher grasps_pub_; ///< the publisher for the antipodal grasps
  Localization* localization_; ///< a pointer to a localization object
  std::vector<GraspHypothesis> hands_; ///< the grasp hypotheses found by the hand search
  std::vector<GraspHypothesis> antipodal_hands_; ///< the antipodal grasps predicted by the SVM
  std::vector<Handle> handles_; ///< the handles found by the handle search
  int num_clouds_received_; ///< the number of point clouds that have been received
  int num_clouds_; ///< the maximum number of point clouds that can be received
  int size_left_; ///< the size of the first point cloud
  int min_inliers_; ///< the minimum number of inliers for the handle search
  bool plots_handles_; ///< whether handles are plotted
  
  /** constants for input point cloud types */
	static const int POINT_CLOUD_2 = 0; ///< sensor_msgs/PointCloud2
	static const int CLOUD_SIZED = 1; ///< agile_grasp/CloudSized
};

#endif /* GRASP_LOCALIZER_H_ */
