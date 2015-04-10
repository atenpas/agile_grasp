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

#ifndef HANDLE_H_
#define HANDLE_H_

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <agile_grasp/grasp_hypothesis.h>

/** HandleSearch class
 *
 * \brief Handle data structure
 * 
 * This class stores a single handle. The handle represents (a) a list of grasp hypotheses that 
 * are part of the handle, and (b) a grasp that is an "average" grasp over these grasp hypotheses.
 * 
*/
class Handle
{
public:
	
	/**
	 * \brief Constructor.
	 * \param hand_list the list of grasp hypotheses
	 * \param indices the list of indices of grasp hypotheses that are part of the handle
	*/
	Handle(const std::vector<GraspHypothesis>& hand_list, const std::vector<int>& inliers);
	
	/**
	 * \brief Return the handle's approach vector.
	 * \return the 3x1 handle approach vector
	*/
	const Eigen::Vector3d& getApproach() const
	{
		return approach_;
	}
	
	/**
	 * \brief Return the handle's axis.
	 * \return the 3x1 handle axis
	*/
	const Eigen::Vector3d& getAxis() const
	{
		return axis_;
	}

	/**
	 * \brief Return the centroid of the handle.
	 * \return the 3x1 handle centroid
	*/
	const Eigen::Vector3d& getCenter() const
	{
		return center_;
	}
	
	/**
	 * \brief Return the handle's grasp surface position.
	 * \return the 3x1 grasp position
	*/
	const Eigen::Vector3d& getHandsCenter() const
	{
		return hands_center_;
	}
	
	/**
	 * \brief Return the width of the object contained in the handle grasp.
	 * \return the width of the contained object
	*/
	double getWidth() const
	{
		return width_;
	}
	
	/**
	 * \brief Return the list of grasp hypotheses.
	 * \return the list of grasp hypotheses
	*/
	const std::vector<GraspHypothesis>& getHandList() const
	{
		return hand_list_;
	}
	
	/**
	 * \brief Return the list of indices of grasp hypotheses that are part of the handle.
	 * \return the list of indices of grasp hypotheses that are part of the handle
	*/
	const std::vector<int>& getInliers() const
	{
		return inliers_;
	}

private:
	
	/**
	 * \brief Set the variables of the grasp.
	*/
	void setGraspVariables();
	
	/**
	 * \brief Set the hand axis of the grasp.
	*/
	void setAxis();

	/**
	 * \brief Set the distance along the handle's axis for each grasp hypothesis.
	*/
	void setDistAlongHandle();
	
	/**
	 * \brief Set the width of the object contained in the handle grasp.
	*/
	void setGraspWidth();

	std::vector<int> inliers_; ///< the list of indices of grasp hypotheses that are part of the handle
	std::vector<GraspHypothesis> hand_list_; ///< the list of grasp hypotheses
	Eigen::Vector3d center_; ///< the center of the "average" grasp
	Eigen::Vector3d axis_; ///< the hand axis of the "average" grasp
	Eigen::Vector3d approach_; ///< the approach vector of the "average" grasp
	double width_; ///< the width of the object contained in the "average" grasp
	Eigen::Vector3d hands_center_; ///< the center of the "average" grasp projected onto the back of the hand
	Eigen::VectorXd dist_along_handle_; ///< the 1xn vector of distances along the handle's axis for each grasp hypothesis
};

#endif /* HANDLE_H_ */
