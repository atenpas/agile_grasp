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

#ifndef ROTATING_HAND_H
#define ROTATING_HAND_H

#include <Eigen/Dense>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <agile_grasp/antipodal.h>
#include <agile_grasp/finger_hand.h>
#include <agile_grasp/grasp_hypothesis.h>

/** RotatingHand class
 *
 * \brief Calculate collision-free grasp hypotheses that fit a point neighborhood.
 * 
 * This class calculates a set of collision-free grasp hypotheses that fit a point neighborhood. 
 * The point neighborhood is first transformed into the robot hand frame, then rotated into one 
 * of the evaluated hand orientations, and finally tested for finger placements. A grasp 
 * hypothesis can also be tested for being antipodal with this class.
 * 
*/
class RotatingHand
{
public:

  /**
	 * \brief Default constructor.
	*/
  RotatingHand() : tolerant_antipodal_(true), cam_source_(-1) { }

	/**
	 * \brief Constructor.
	 * \param tolerant_antipodal whether the antipodal quality estimation uses "tolerant" thresholds
	 * \param the camera source of the sample for which the point neighborhood was found
	*/
	RotatingHand(bool tolerant_antipodal, int cam_source) 
    : tolerant_antipodal_(tolerant_antipodal), cam_source_(cam_source)
	{	}

	/**
	 * \brief Constructor.
	 * \param camera_origin_left the position of the left camera
	 * \param camera_origin_right the position of the right camera
	 * \param finger_hand the FingerHand object to be used
	 * \param tolerant_antipodal whether the antipodal quality estimation uses "tolerant" thresholds
	 * \param the camera source of the sample for which the point neighborhood was found
	 */
	RotatingHand(const Eigen::VectorXd& camera_origin_left, const Eigen::VectorXd& camera_origin_right,
			const FingerHand& finger_hand, bool tolerant_antipodal, int cam_source);

	/**
	 * \brief Transforms a set of points into the hand frame, defined by normal and axis.
	 * \param points the set of points to be transformed
	 * \param normal the normal, used as one axis in the hand frame
	 * \param axis the axis, used as another axis in the hand frame
	 * \param normals the set of normals for the set of points
	 * \param points_cam_source the set of camera sources for the set of points
	 * \param hand_height the hand height, the hand extends plus/minus this value along the hand axis
	 */
	void transformPoints(const Eigen::Matrix3Xd& points, const Eigen::Vector3d& normal,
			const Eigen::Vector3d& axis, const Eigen::Matrix3Xd& normals, const Eigen::VectorXi& points_cam_source,
			double hand_height);
	
	/**
	 * \brief Evaluate which hand orientations and which finger placements lead to a valid grasp.
	 * \param init_bite the minimum object height
	 * \param sample the sample for which the point neighborhood was found
	 * \param use_antipodal whether the hand orientations are checked for antipodal grasps
	 */
	std::vector<GraspHypothesis> evaluateHand(double init_bite, const Eigen::Vector3d& sample, bool use_antipodal);
			  
  /**
   * \brief Return the camera positions.
   * \return the 3x2 matrix containing the positions of the cameras
  */
  const Eigen::Matrix<double, 3, 2>& getCams() const 
  {
    return cams_;
  }
  
  /**
   * \brief Return the camera source for each point in the point neighborhood.
   * \return the 1xn matrix containing the camera source for each point in the point neighborhood
  */
  const Eigen::VectorXi& getPointsCamSource() const 
  {
    return points_cam_source_;
  }
  
  /**
   * \brief Return the camera source of the sample for which the point neighborhood was calculated.
   * \return the camera source of the sample
  */
  int getCamSource() const
  {
    return cam_source_;
  }


private:

  int cam_source_; ///< the camera source of the sample
	Eigen::Matrix<double, 3, 2> cams_; ///< the camera positions
	Eigen::VectorXd angles_; ///< the hand orientations that are evaluated
	Eigen::Matrix3Xd points_; ///< the points in the point neighborhood
	Eigen::Vector3d hand_axis_; ///< the robot hand axis for the point neighborhood
	Eigen::Matrix3d frame_; ///< the robot hand frame for the point neighborhood
	Eigen::Matrix3Xd normals_; ///< the normals for each point in the point neighborhood
	Eigen::VectorXi points_cam_source_;	 ///<	the camera source for each point in the point neighborhood
  FingerHand finger_hand_; ///< the finger hand object
  
  /** parameters */
  bool tolerant_antipodal_; ///< whether the antipodal testing uses "tolerant" thresholds
};

#endif
