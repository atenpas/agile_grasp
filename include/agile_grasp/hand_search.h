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

#ifndef HAND_SEARCH_H
#define HAND_SEARCH_H


#include <Eigen/Dense>

#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <agile_grasp/finger_hand.h>
#include <agile_grasp/grasp_hypothesis.h>
#include <agile_grasp/plot.h>
#include <agile_grasp/quadric.h>
#include <agile_grasp/rotating_hand.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/** HandSearch class
 *
 * \brief Search for grasp hypotheses.
 * 
 * This class searches for grasp hypotheses in a point cloud by first fitting a quadratic surface 
 * to a small, local point neighborhood, and then finding geometrically feasible grasp hypotheses for 
 * a larger point neighborhood. It can also estimate whether the grasp is antipodal from the normals of 
 * the point neighborhood.
 * 
*/
class HandSearch
{
  public:
    
    /**
		 * \brief Constructor.
		 * \param finger_width the width of the robot hand fingers
		 * \param hand_outer_diameter the maximum robot hand aperture
		 * \param hand_depth the hand depth (length of fingers)
		 * \param hand_height the hand extends plus/minus this value along the hand axis
		 * \param init_bite the minimum object height
		 * \param num_threads the number of CPU threads to be used for the search
		 * \param num_samples the number of samples drawn from the point cloud
		 * \param plots_hands whether grasp hypotheses are plotted
		*/
    HandSearch(double finger_width, double hand_outer_diameter, 
      double hand_depth, double hand_height, double init_bite, int num_threads, 
      int num_samples, bool plots_hands) : finger_width_(finger_width), 
      hand_outer_diameter_(hand_outer_diameter), hand_depth_(hand_depth), 
      hand_height_(hand_height), init_bite_(init_bite), 
      num_threads_(num_threads), num_samples_(num_samples), 
      plots_hands_(plots_hands), plots_samples_(false), 
      plots_local_axes_(false), uses_determinstic_normal_estimation_(false), 
      nn_radius_taubin_(0.03), nn_radius_hands_(0.08) { }
    
    /**
		 * \brief Find grasp hypotheses in a point cloud.
		 * 
		 * If the parameter @p indices is of size 0, then indices are randomly drawn from the point 
		 * cloud according to a uniform distribution.
		 * 
		 * \param cloud the point cloud that is searched for grasp hypotheses
		 * \param pts_cam_source the camera source for each point in the point cloud
		 * \param indices the list of point cloud indices that are used in the search
		 * \param cloud_plot the point cloud that is used for plotting
		 * \param calculates_antipodal whether grasp hypotheses are estimated to be antipodal
		 * \param uses_clustering whether clustering is used to divide the point cloud (useful for creating training data)
		 * \return a list of grasp hypotheses
		*/
    std::vector<GraspHypothesis> findHands(const PointCloud::Ptr cloud, 
      const Eigen::VectorXi& pts_cam_source, const std::vector<int>& indices,
      const PointCloud::Ptr cloud_plot, bool calculates_antipodal, 
      bool uses_clustering);
      
  
  private:
    
    /**
		 * \brief Find quadratic surfaces in a point cloud.
		 * \param cloud the point cloud that is searched for quadrics
		 * \param pts_cam_source the camera source for each point in the point cloud
		 * \param kdtree the KD tree that is used for finding point neighborhoods
		 * \param indices the list of point cloud indices that are used in the search
		 * \return a list of quadratic surfaces
		*/
    std::vector<Quadric> findQuadrics(const PointCloud::Ptr cloud, const Eigen::VectorXi& pts_cam_source,
			const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, const std::vector<int>& indices);
		
		/**
		 * \brief Find grasp hypotheses in a point cloud given a list of quadratic surfaces.
		 * \param cloud the point cloud that is searched for quadrics
		 * \param pts_cam_source the camera source for each point in the point cloud
		 * \param quadric_list the list of quadratic surfaces
		 * \param hands_cam_source the camera source for each sample
		 * \param kdtree the KD tree that is used for finding point neighborhoods
		 * \return a list of grasp hypotheses
		*/
    std::vector<GraspHypothesis> findHands(const PointCloud::Ptr cloud, const Eigen::VectorXi& pts_cam_source,
			const std::vector<Quadric>& quadric_list, const Eigen::VectorXi& hands_cam_source, 
      const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
    
    Eigen::Matrix4d cam_tf_left_, cam_tf_right_; ///< camera poses
    
    /** hand search parameters */
    double finger_width_;
    double hand_outer_diameter_; ///< the maximum robot hand aperture
    double hand_depth_; ///< the finger length
    double hand_height_; ///< the hand extends plus/minus this value along the hand axis
    double init_bite_; ///< the minimum object height
    int num_threads_; ///< the number of threads used in the search
    int num_samples_; ///< the number of samples used in the search
    
    Eigen::Matrix3Xd cloud_normals_; ///< a 3xn matrix containing the normals for points in the point cloud
    Plot plot_; ///< plot object for visualization of search results
    
    bool uses_determinstic_normal_estimation_; ///< whether the normal estimation for the quadratic surface is deterministic (used for debugging)
    bool tolerant_antipodal_; ///< whether the antipodal testing uses "tolerant" thresholds
    
    bool plots_samples_; ///< whether the samples drawn from the point cloud are plotted
    bool plots_camera_sources_; ///< whether the camera source for each point in the point cloud is plotted
    bool plots_local_axes_; ///< whether the local axes estimated for each point neighborhood are plotted
    bool plots_hands_; ///< whether the grasp hypotheses are plotted
    
    double nn_radius_taubin_; ///< the radius for the neighborhood search for the quadratic surface fit
    double nn_radius_hands_; ///< the radius for the neighborhood search for the hand search
};

#endif /* HAND_SEARCH_H */ 
