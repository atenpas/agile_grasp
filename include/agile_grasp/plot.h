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

#ifndef PLOT_H
#define PLOT_H

#include <pcl/visualization/pcl_visualizer.h>

#include <agile_grasp/grasp_hypothesis.h>
#include <agile_grasp/quadric.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;

/** Plot class
* 
* This class provides a set of visualization methods that visualize the output of the algorithm and 
* some intermediate steps. The visualization is done using PCL Visualizer.
* 
*/
class Plot
{
	public:
		
		/**
		 * \brief Plot a set of grasp hypotheses.
		 * \param hand_list the set of grasp hypotheses to be plotted
		 * \param cloud the point cloud to be plotted
		 * \param str the title of the visualization window
		 * \param use_grasp_bottom whether the grasps plotted originate from the grasp bottom point
		*/
		void plotHands(const std::vector<GraspHypothesis>& hand_list, const PointCloud::Ptr& cloud, std::string str,
			bool use_grasp_bottom = false);
		
		/** 
		 * \brief Plot a set of grasp hypotheses and a set of antipodal grasps.
		 * \param hand_list the set of grasp hypotheses to be plotted
		 * \param antipodal_hand_list the set of antipodal grasps to be plotted
		 * \param cloud the point cloud to be plotted
		 * \param str the title of the visualization window
		 * \param use_grasp_bottom whether the grasps plotted originate from the grasp bottom point
		*/ 
		void plotHands(const std::vector<GraspHypothesis>& hand_list,
			const std::vector<GraspHypothesis>& antipodal_hand_list, const PointCloud::Ptr& cloud, std::string str,
			bool use_grasp_bottom = false);
		
		/** 
		 * \brief Plot a set of samples.
		 * \param index_list the list of samples (indices into the point cloud)
		 * \param cloud the point cloud to be plotted
		*/
		void plotSamples(const std::vector<int>& index_list, const PointCloud::Ptr& cloud);
		
		/** 
		 * \brief Plot a set of quadrics by plotting their local axes.
		 * \param quadric_list the list of quadrics to be plotted
		 * \param cloud the point cloud to be plotted
		*/
		void plotLocalAxes(const std::vector<Quadric>& quadric_list, const PointCloud::Ptr& cloud);
		
		/** 
		 * \brief Plot the camera source for each point in the point cloud.
		 * \param pts_cam_source_in the camera source for each point in the point cloud
		 * \param cloud the point cloud to be plotted
		*/
		void plotCameraSource(const Eigen::VectorXi& pts_cam_source_in, const PointCloud::Ptr& cloud);
		
	private:
		
		/** 
		 * \brief Create a point cloud with normals that represent the approach vectors for a set of 
		 * grasp hypotheses.
		 * \param hand_list the set of grasp hypotheses
		 * \param plots_only_antipodal whether only the approach vectors of antipodal grasps are 
		 * considered
		 * \param plots_grasp_bottom whether the approach vectors plotted originate from the grasp 
		 * bottom point 
		*/
		PointCloudNormal::Ptr createNormalsCloud(const std::vector<GraspHypothesis>& hand_list,
			bool plots_only_antipodal, bool plots_grasp_bottom);
		
		/** 
		 * \brief Add a point cloud with normals to a PCL visualizer.
		 * \param viewer the PCL visualizer that the cloud is added to
		 * \param cloud the cloud to be added
		 * \param line_width the line width for drawing normals
		 * \param color_cloud the color that is used to draw the cloud
		 * \param color_normals the color that is used to draw the normals
		 * \param cloud_name an identifier string for the cloud
		 * \param normals_name an identifier string for the normals
		*/
		void addCloudNormalsToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
			const PointCloudNormal::Ptr& cloud, double line_width, double* color_cloud, double* color_normals,
			const std::string& cloud_name, const std::string& normals_name);
		
		/** 
		 * \brief Plot two point clouds representing grasp hypotheses and antipodal grasps, 
		 * respectively.
		 * \param hands_cloud the cloud that represents the grasp hypotheses
		 * \param antipodal_hands_cloud the cloud that represents the antipodal grasps
		 * \param cloud the point cloud to be plotted
		 * \param str the title of the visualization window
		 * \param use_grasp_bottom whether the grasps plotted originate from the grasp bottom point
		*/
		void plotHandsHelper(const PointCloudNormal::Ptr& hands_cloud,
			const PointCloudNormal::Ptr& antipodal_hands_cloud, const PointCloud::Ptr& cloud,
			std::string str, bool use_grasp_bottom);
		
		/** 
		 * \brief Run/show a PCL visualizer until an escape key is hit.
		 * \param viewer the PCL visualizer to be shown
		*/
		void runViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);
		
		/** 
		 * \brief Create a PCL visualizer.
		 * \param title the title of the visualization window
		*/
		boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer(std::string title);
};

#endif /* PLOT_H */ 
