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

#ifndef LEARNING_H_
#define LEARNING_H_

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <set>
#include <sys/stat.h>
#include <vector>

#include <agile_grasp/rotating_hand.h>

/** Learning class
 *
 * \brief Train and use an SVM to predict antipodal grasps  
 * 
 * This class trains an SVM classifier to predict antipodal grasps. Once trained, it can be used to 
 * predict antipodal grasps. The classifier is trained with HOG features obtained from grasp images. 
 * A grasp image is a 2D image representation of a grasp hypothesis.
 * 
*/
class Learning
{
public:

	/**
	 * \brief Constructor.
	*/
	Learning() :
			num_horizontal_cells_(100), num_vertical_cells_(80), num_threads_(1)
	{
	}
	
	/**
	 * \brief Constructor. Set the number of threads to be used for prediction.
	*/
	Learning(int num_threads) :
			num_horizontal_cells_(100), num_vertical_cells_(80), num_threads_(num_threads)
	{
	}
  
  /** 
	 * \brief Train an SVM classifier to predict antipodal grasps. Uses the same number of positives 
	 * and negatives for each training point cloud.
	 * \param hands_list the set of grasp hypotheses to be used for training
	 * \param sizes the number of grasp hypotheses found for each training point cloud
	 * \param file_name the location and name of the file in which the SVM is stored
	 * \param cam_pos the camera poses
	 * \param max_positive the maximum number of positives examples to be used from each training 
	 * point cloud
	 * \param is_plotting whether the training process is visualized
	*/ 
  void trainBalanced(const std::vector<GraspHypothesis>& hands_list, const std::vector<int>& sizes,
		const std::string& file_name, const Eigen::Matrix3Xd& cam_pos, int max_positive = 1000000000,
		bool is_plotting = false);

	/** 
	 * \brief Train an SVM classifier to predict antipodal grasps.
	 * \param hands_list the set of grasp hypotheses to be used for training
	 * \param sizes the number of grasp hypotheses found for each training point cloud
	 * \param file_name the location and name of the file in which the SVM is stored
	 * \param max_positive the maximum number of positives examples to be used from each training 
	 * point cloud
	 * \param cam_pos the camera poses
	 * \param is_plotting whether the training process is visualized
	*/ 
	void train(const std::vector<GraspHypothesis>& hands_list, const std::vector<int> & sizes,
		const std::string& file_name, const Eigen::Matrix3Xd& cam_pos, int max_positive = 1000000000,
		bool is_plotting = false);

	/** 
	 * \brief Train an SVM classifier to predict antipodal grasps.
	 * \param hands_list the set of grasp hypotheses to be used for training
	 * \param sizes the number of grasp hypotheses found for each training point cloud
	 * \param file_name the location and name of the file in which the SVM is stored
	 * \param is_plotting whether the training process is visualized
	*/ 
	void train(const std::vector<GraspHypothesis>& hands_list, const std::string& file_name,
		const Eigen::Matrix3Xd& cam_pos, bool is_plotting = false);
	
	/** 
	 * \brief Predict antipodal grasps using an SVM classifier.
	 * \param hands_list the set of grasp hypotheses to be tested for being antipodal grasps
	 * \param file_name the location and name of the file in which the SVM is stored
	 * \param cam_pos the camera poses
	 * \param is_plotting whether the training process is visualized
	*/ 
	std::vector<GraspHypothesis> classify(const std::vector<GraspHypothesis>& hands_list,
		const std::string& svm_filename, const Eigen::Matrix3Xd& cam_pos, bool is_plotting = false);

private:
  
  /**
   * \brief Learning instance representing a grasp hypothesis. 
  */
  struct Instance
  {
    Eigen::Matrix3Xd pts; ///< the points from the point cloud that make up the hypothesis
    Eigen::Vector3d binormal; ///< the binormal of the grasp
    Eigen::Vector3d source_to_center; ///< the vector from the center of the grasp to the camera position
    bool label; ///< the label of the instance (true: antipodal, false: not antipodal)
  };
  
  /**
   * \brief Comparator for 2D vectors.
  */
	struct UniqueVectorComparator
	{
    /**
     * \brief Compare two vectors.
     * \param a the first vector to be compared
     * \param b the second vector to be compared
     * \return true if no elements of @p a and @p b are equal, false otherwise 
    */
		bool operator ()(const Eigen::Vector2i& a, const Eigen::Vector2i& b)
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
   * \brief Create a learning instance from a grasp hypothesis.
   * \param h the grasp hypothesis from which the learning instance is created
   * \param cam_pos the camera poses
   * \param cam index of the camera which produced the points in the grasp hypothesis 
   * \return the created learning instance 
  */
  Instance createInstance(const GraspHypothesis& h, 
    const Eigen::Matrix3Xd& cam_pos, int cam = -1);

  /**
   * \brief Convert a set of learning instances to training data readable by the SVM. 
   * \param instances the training instances to be converted
   * \param file_name the location and name of the file in which the SVM is stored
   * \param is_plotting whether the conversion process is visualized
   * \param uses_linear_kernel whether a linear or a quadratic kernel is used in the SVM 
  */
	void convertData(const std::vector<Instance>& instances,
		const std::string& file_name, bool is_plotting = false, 
    bool uses_linear_kernel = false);

  /**
   * \brief Convert a learning instance to a grasp image.
   * \param ins the learning instance to be converted
   * \return the created image
  */
	cv::Mat convertToImage(const Instance& ins);
  
  /**
   * \brief Round a vector's elements down to the closest, smaller integers.
   * \param a the vector whose elements are rounded down
   * \return the vector containing the rounded elements
  */
	Eigen::VectorXi floorVector(const Eigen::VectorXd& a);

	int num_horizontal_cells_; ///< the width of a grasp image
	int num_vertical_cells_; ///<  the height of a grasp image 
	int num_threads_; ///< the number of threads used for prediction
};

#endif /* LEARNING_H_ */
