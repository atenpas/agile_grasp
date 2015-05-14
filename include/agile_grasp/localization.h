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

#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

// system dependencies
#include <iostream>
#include <math.h>
#include <set>
#include <string>
#include <time.h>

// PCL dependencies
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// project dependencies
#include <agile_grasp/grasp_hypothesis.h>
#include <agile_grasp/hand_search.h>
#include <agile_grasp/handle.h>
#include <agile_grasp/handle_search.h>
#include <agile_grasp/learning.h>
#include <agile_grasp/plot.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/** Localization class
 *
 * \brief High level interface for the localization of grasp hypotheses and handles
 * 
 * This class provides a high level interface to search for grasp hypotheses, antipodal grasps, and 
 * handles. It can also preprocess the point clouds.
 * 
*/
class Localization
{
public:

	/**
	 * \brief Default Constructor.
	*/
	Localization() : num_threads_(1), plotting_mode_(1), plots_camera_sources_(false), cloud_(new PointCloud)
	{ }
	
	/**
	 * \brief Constructor.
	 * \param num_threads the number of threads to be used in the search
	 * \param filter_boundaries whether grasp hypotheses that are close to the point cloud boundaries are filtered out
	 * \param plots_hands whether grasp hypotheses are plotted
	*/
	Localization(int num_threads, bool filters_boundaries, int plotting_mode) :
			num_threads_(num_threads), filters_boundaries_(filters_boundaries), 
      plotting_mode_(plotting_mode), plots_camera_sources_(false), 
      cloud_(new PointCloud)
	{ }
	
	/**
	 * \brief Find handles given a list of grasp hypotheses.
	 * \param hand_list the list of grasp hypotheses
	 * \param min_inliers the minimum number of handle inliers
	 * \param min_length the minimum length of the handle
	 * \param is_plotting whether the handles are plotted
	*/
	std::vector<Handle> findHandles(const std::vector<GraspHypothesis>& hand_list, int min_inliers,	double min_length);
	
	/**
	 * \brief Predict antipodal grasps given a list of grasp hypotheses.
	 * \param hand_list the list of grasp hypotheses
	 * \param svm_filename the location and filename of the SVM
	*/
	std::vector<GraspHypothesis> predictAntipodalHands(const std::vector<GraspHypothesis>& hand_list,
			const std::string& svm_filename);
	
	/**
	 * \brief Localize hands in a given point cloud.
	 * \param cloud_in the input point cloud
	 * \param indices the set of point cloud indices for which point neighborhoods are found
	 * \param calculates_antipodal whether the grasp hypotheses are checked for being antipodal
	 * \param uses_clustering whether clustering is used for processing the point cloud
	 * \return the list of grasp hypotheses found
	*/
	std::vector<GraspHypothesis> localizeHands(const PointCloud::Ptr& cloud_in, int size_left,
			const std::vector<int>& indices, bool calculates_antipodal, bool uses_clustering);
	
	/**
	 * \brief Localize hands given two point cloud files.
	 * \param pcd_filename_left the first point cloud file location and name
	 * \param pcd_filename_right the second point cloud file location and name
	 * \param calculates_antipodal whether the grasp hypotheses are checked for being antipodal
	 * \param uses_clustering whether clustering is used for processing the point cloud
	 * \return the list of grasp hypotheses found
	*/
	std::vector<GraspHypothesis> localizeHands(const std::string& pcd_filename_left,
			const std::string& pcd_filename_right, bool calculates_antipodal = false, 
      bool uses_clustering = false);
	
	/**
	 * \brief Localize hands given two point cloud files and a set of point cloud indices.
	 * \param pcd_filename_left the first point cloud file location and name
	 * \param pcd_filename_right the second point cloud file location and name
	 * \param indices the set of point cloud indices for which point neighborhoods are found
	 * \param calculates_antipodal whether the grasp hypotheses are checked for being antipodal
	 * \param uses_clustering whether clustering is used for processing the point cloud
	 * \return the list of grasp hypotheses found
	*/
	std::vector<GraspHypothesis> localizeHands(const std::string& pcd_filename_left,
			const std::string& pcd_filename_right, const std::vector<int>& indices, 
      bool calculates_antipodal =	false, bool uses_clustering = false);
	
	/**
	 * \brief Set the camera poses.
	 * \param cam_tf_left the pose of the left camera
	 * \param cam_tf_right the pose of the right camera
	*/
	void setCameraTransforms(const Eigen::Matrix4d& cam_tf_left, const Eigen::Matrix4d& cam_tf_right)
	{
		cam_tf_left_ = cam_tf_left;
		cam_tf_right_ = cam_tf_right;
	}
	
	/**
	 * \brief Return the camera pose of one given camera.
	 * \param is_left true if the pose of the left camera is wanted, false if the pose of the right camera is wanted
	 * \return the pose of the camera specified by @p is_left
	*/
	const Eigen::Matrix4d& getCameraTransform(bool is_left)
	{
		if (is_left)
			return cam_tf_left_;
		else
			return cam_tf_right_;
	}
	
	/**
	 * \brief Set the dimensions of the robot's workspace.
	 * \param workspace 1x6 vector containing the robot's workspace dimensions
	*/
	void setWorkspace(const Eigen::VectorXd& workspace)
	{
		workspace_ = workspace;
	}
	
	/**
	 * \brief Set the number of samples to be used for the search.
	 * \param num_samples the number of samples to be used for the search
	*/
	void setNumSamples(int num_samples)
	{
		num_samples_ = num_samples;
	}
	
	/**
	 * \brief Set the radius to be used for the point neighborhood search in the hand search.
	 * \param nn_radius_hands the radius to be used for the point neighborhood search
	*/
	void setNeighborhoodRadiusHands(double nn_radius_hands)
	{
		nn_radius_hands_ = nn_radius_hands;
	}
	
	/**
	 * \brief Set the radius to be used for the point neighborhood search in the quadric fit.
	 * \param nn_radius_hands the radius to be used for the point neighborhood search
	*/
	void setNeighborhoodRadiusTaubin(double nn_radius_taubin)
	{
		nn_radius_taubin_ = nn_radius_taubin;
	}
	
	/**
	 * \brief Set the finger width of the robot hand.
	 * \param finger_width the finger width
	*/
	void setFingerWidth(double finger_width)
	{
		finger_width_ = finger_width;
	}
	
	/**
	 * \brief Set the hand depth of the robot hand.
	 * \param hand_depth the hand depth of the robot hand (usually the finger length)
	*/
	void setHandDepth(double hand_depth)
	{
		hand_depth_ = hand_depth;
	}
	
	/**
	 * \brief Set the maximum aperture of the robot hand.
	 * \param hand_outer_diameter the maximum aperture of the robot hand
	*/
	void setHandOuterDiameter(double hand_outer_diameter)
	{
		hand_outer_diameter_ = hand_outer_diameter;
	}
	
	/**
	 * \brief Set the initial "bite" of the robot hand (usually the minimum object "height").
	 * \param init_bite the initial "bite" of the robot hand (@see FingerHand)
	*/
	void setInitBite(double init_bite)
	{
		init_bite_ = init_bite;
	}
	
	/**
	 * \brief Set the height of the robot hand.
	 * \param hand_height the height of the robot hand, the hand extends plus/minus this value along the hand axis
	*/
	void setHandHeight(double hand_height)
	{
		hand_height_ = hand_height;
	}
		
	/**
	 * \brief Set the publisher for Rviz visualization, the lifetime of visual markers, and the frame associated with 
	 * the grasps.
	 * \param node the ROS node
	 * \param marker_lifetime the lifetime of each visual marker
	 * \param frame the frame to which the grasps belong
	*/ 
	void createVisualsPub(ros::NodeHandle& node, double marker_lifetime, const std::string& frame)
	{
		plot_.createVisualPublishers(node, marker_lifetime);
		visuals_frame_ = frame;
	}
	

private:

	/**
	 * \brief Comparator for checking uniqueness of two 3D-vectors. 
	*/
	struct UniqueVectorComparator
	{
		/**
		 * \brief Compares two 3D-vectors for uniqueness.
		 * \param a the first 3D-vector
		 * \param b the second 3D-vector
		 * \return true if they have no equal elements, false otherwise
		*/
		bool operator ()(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
		{
			for (int i = 0; i < a.size(); i++)
			{
				if (a(i) != b(i))
				{
					return a(i) < b(i);
				}
			}

			return false;
		}
	};
	
	/**
	 * \brief Voxelize the point cloud and keep track of the camera source for each voxel.
	 * \param[in] cloud_in the point cloud to be voxelized
	 * \param[in] pts_cam_source_in the camera source for each point in the point cloud
	 * \param[out] cloud_out the voxelized point cloud
	 * \param[out] pts_cam_source_out the camera source for each point in the voxelized cloud
	 * \param[in] cell_size the size of each voxel
	*/
	void voxelizeCloud(const PointCloud::Ptr& cloud_in, const Eigen::VectorXi& pts_cam_source_in,
			PointCloud::Ptr& cloud_out, Eigen::VectorXi& pts_cam_source_out, double cell_size);
	
	/**
	 * \brief Filter out points in the point cloud that lie outside the workspace dimensions and keep 
	 * track of the camera source for each point that is not filtered out.
	 * \param[in] cloud_in the point cloud to be filtered
	 * \param[in] pts_cam_source_in the camera source for each point in the point cloud
	 * \param[out] cloud_out the filtered point cloud
	 * \param[out] pts_cam_source_out the camera source for each point in the filtered cloud
	*/
	void filterWorkspace(const PointCloud::Ptr& cloud_in, const Eigen::VectorXi& pts_cam_source_in,
			PointCloud::Ptr& cloud_out, Eigen::VectorXi& pts_cam_source_out);
	
	/**
	 * \brief Filter out grasp hypotheses that are close to the workspace boundaries.
	 * \param hand_list the list of grasp hypotheses to be filtered
	 * \return the list of grasp hypotheses that are not filtered out
	*/
	std::vector<GraspHypothesis> filterHands(const std::vector<GraspHypothesis>& hand_list);
	
	/**
	 * \brief Round a 3D-vector down to the closest, smaller integers.
	 * \param a the 3D-vector to be rounded down
	 * \return the rounded down 3D-vector
	*/ 
	Eigen::Vector3i floorVector(const Eigen::Vector3d& a);

	Plot plot_; ///< the plot object
	Eigen::Matrix4d cam_tf_left_, cam_tf_right_; ///< the camera poses
	Eigen::VectorXd workspace_; ///< the robot's workspace dimensions
	Eigen::Matrix3Xd cloud_normals_; ///< the normals for each point in the point cloud
	PointCloud::Ptr cloud_; ///< the input point cloud
	int num_threads_; ///< the number of CPU threads used in the search
	int num_samples_; ///< the number of samples used in the search
	double nn_radius_taubin_; ///< the radius of the neighborhood search used in the grasp hypothesis search
	double nn_radius_hands_; ///< the radius of the neighborhood search used in the quadric fit
	double finger_width_; ///< width of the fingers
	double hand_outer_diameter_; ///< maximum hand aperture
	double hand_depth_; ///< hand depth (finger length)
	double hand_height_; ///< hand height
	double init_bite_; ///< initial bite
  bool plots_camera_sources_; ///< whether the camera source is plotted for each point in the point cloud	
	bool filters_boundaries_; ///< whether grasp hypotheses close to the workspace boundaries are filtered out
	int plotting_mode_; ///< what plotting mode is used
	std::string visuals_frame_; ///< visualization frame for Rviz
	
	/** constants for plotting modes */
	static const int NO_PLOTTING = 0; ///< no plotting
	static const int PCL_PLOTTING = 1; ///< plotting in PCL
	static const int RVIZ_PLOTTING = 2; ///< plotting in Rviz
};

#endif
