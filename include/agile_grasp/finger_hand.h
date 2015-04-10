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

#ifndef FINGER_HAND_H
#define FINGER_HAND_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

/** FingerHand class
 *
 * \brief Calculate collision-free fingers
 * 
 * This class calculates a set of collision-free finger placements. The parameters are the dimensions of the 
 * robot hand and the desired "bite" that the grasp must have. The bite is how far the hand can be 
 * moved into the object.
 * 
*/
class FingerHand
{
public:

  /**
   * \brief Default constructor.
  */
  FingerHand()
  {
  }
	
	/**
   * \brief Constructor.
   * \param finger_width the width of the fingers
   * \param hand_outer_diameter the maximum aperture of the robot hand
   * \param hand_depth the length of the fingers
  */
  FingerHand(double finger_width, double hand_outer_diameter, double hand_depth);

  /**
   * \brief Find collision-free finger placements.
   * \param bite the minimum object height
  */
  void evaluateFingers(double bite);
	
	/**
   * \brief Find robot hand configurations that fit the cloud.
  */
  void evaluateHand();
	
	/**
   * \brief Set the grasp parameters.
   * 
   * The parameter @p bite is used to calculate the grasp width by only evaluating the width of 
   * the points below @p bite.
   * 
   * \param bite the minimum object height
  */
  void evaluateGraspParameters(double bite);
	
	/**
   * \brief Select center hand and try to extend it into the object as deeply as possible.
   * \param init_deepness the initial deepness (usually the minimum object height)
   * \param max_deepness the maximum allowed deepness (usually the finger length)
  */
  void deepenHand(double init_deepness, double max_deepness);
	
	/**
	 * \brief Set the points.
	 * 
	 * The points contained in the matrix @p points are assumed to be already rotated into the robot 
	 * hand frame and offset to a point within the local neighborhood.
	 * 
	 * \param points the points to be set
	*/
  void setPoints(const Eigen::MatrixXd& points)
  {
    this->points_ = points;
  }
	
	/**
	 * \brief Return the depth of the hand.
	 * \return the hand depth
	*/
  double getHandDepth() const
  {
    return hand_depth_;
  }
	
	/**
	 * \brief Return the hand configuration evaluations.
	 * \return the hand configuration evaluations
	*/
  const Eigen::Array<bool, 1, Eigen::Dynamic>& getHand() const
  {
    return hand_;
  }
	
	/**
	 * \brief Return the finger placement evaluations.
	 * \return the hand configuration evaluations
	*/
  const Eigen::Array<bool, 1, Eigen::Dynamic>& getFingers() const
  {
    return fingers_;
  }
	
	/**
	 * \brief Return the grasp position between the end of the finger tips.
	 * \return the 2x1 grasp position between the end of the finger tips
	*/
  const Eigen::Vector2d& getGraspBottom() const
  {
    return grasp_bottom;
  }
	
	/**
	 * \brief Return the grasp position between the end of the finger tips projected onto the back of the hand.
	 * \return the 2x1 grasp position between the end of the finger tips projected onto the back of the hand
	*/
  const Eigen::Vector2d& getGraspSurface() const
  {
    return grasp_surface;
  }
	
	/**
	 * \brief Return the width of the object contained in the grasp.
	 * \return the width of the object contained in the grasp
	*/
  double getGraspWidth() const
  {
    return grasp_width_;
  }
	
	/**
	 * \brief Return where the back of the hand is.
	 * \return the back of the hand
	*/
  double getBackOfHand() const
  {
    return back_of_hand_;
  }

private:

  double finger_width_; ///< the width of the robot hand fingers
  double hand_outer_diameter_; ///< the maximum aperture of the robot hand
  double hand_depth_; ///< the hand depth (finger length)

  double back_of_hand_; ///< where the back of the hand is (distance from back to position between finger tip ends)

  double grasp_width_; ///< the width of the object contained in the grasp

  Eigen::VectorXd finger_spacing_; ///< the possible finger placements
  Eigen::Array<bool, 1, Eigen::Dynamic> fingers_; ///< indicates which finger placements are collision-free
  Eigen::Array<bool, 1, Eigen::Dynamic> hand_;///< indicates which hand configurations fit the point cloud
  Eigen::MatrixXd points_; ///< the points in the point neighborhood (see FingerHand::setPoints)
  Eigen::Vector2d grasp_bottom; ///< the grasp position between the end of the finger tips
  Eigen::Vector2d grasp_surface; ///< the position between the end of the finger tips projected to the back of the hand
};

#endif
