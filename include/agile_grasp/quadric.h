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

#ifndef QUADRIC_H
#define QUADRIC_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

/** Lapack function to solve the generalized eigenvalue problem */
extern "C" void dggev_(const char* JOBVL, const char* JOBVR, const int* N, const double* A, const int* LDA,
                       const double* B, const int* LDB, double* ALPHAR, double* ALPHAI, double* BETA, double* VL,
                       const int* LDVL, double* VR, const int* LDVR, double* WORK, const int* LWORK, int* INFO);

/** Quadric class
 *
 * \brief Quadratic surface fit and local axes estimation
 * 
 * This class fits a quadratic surface to a point neighborhood and estimates the curvature axis, 
 * normal, and binormal for the surface fitted to a point neighborhood. To fit the quadratic 
 * surface, a method from the LAPACK library is used.
 * 
*/
class Quadric
{
public:

  /**
   * \brief Standard constructor.
  */
  Quadric()
  {
  }

	/**
	 * \brief Constructor.
	 * \param T_cams the camera poses
	 * \param input the input point cloud
	 * \param sample the sample for which the point neighborhood was found
	 * \param is_deterministic_ whether the local axes estimation is deterministic
	*/
  Quadric(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& T_cams,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const Eigen::Vector3d& sample, bool is_deterministic);
  
  /**
	 * \brief Estimate the local axes for the quadric fitted to the point neighborhood.
	 * \param indices the list of point cloud indices that belong to the point neighborhood
	 * \param cam_source the camera source for each point in the point cloud
	*/
  void findTaubinNormalAxis(const std::vector<int> &indices, const Eigen::VectorXi& cam_source);
	
	/**
	 * \brief Fit a quadratic surface to a point neighborhood.
	 * \param indices the list of point cloud indices that belong to the point neighborhood
	*/
	void fitQuadric(const std::vector<int>& indices);

	/**
	 * \brief Set the input point cloud.
	 * \param input the input point cloud
	*/
	void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
	{
		this->input_ = input;
	}
	
	/**
	 * \brief Print a description of the quadric to the system's standard output.
	*/
	void print();
	
	/**
	 * \brief Plot the local axes.
	 * \param viewer_void a pointer to a PCL visualizer
	 * \param id an identifier number for the axes
	*/
	void plotAxes(void* viewer_void, int id) const;
	
	/**
	 * \brief Return the sample for which the point neighborhood was found.
	 * \return the 3x1 sample for which the point neighborhood was found
	*/
	const Eigen::Vector3d& getSample() const
	{
		return sample_;
	}
	
	/**
	 * \brief Return the binormal for the quadric fitted to the point neighborhood.
	 * \return the 3x1 binormal for the quadric fitted to the point neighborhood
	*/
	const Eigen::Vector3d& getBinormal() const
	{
		return binormal_;
	}
	
	/**
	 * \brief Return the curvature axis for the quadric fitted to the point neighborhood.
	 * \return the 3x1 curvature axis for the quadric fitted to the point neighborhood
	*/
	const Eigen::Vector3d& getCurvatureAxis() const
	{
		return curvature_axis_;
	}
	
	/**
	 * \brief Return the normal for the quadric fitted to the point neighborhood.
	 * \return the 3x1 normal for the quadric fitted to the point neighborhood
	*/
	const Eigen::Vector3d& getNormal() const
	{
		return normal_;
	}


private:

	/**
	 * \brief Estimate the average normal axis for the quadric fitted to the point neighborhood.
	 * \param normals the 3xn matrix of normals found for points in the point neighborhood
	*/
	void findAverageNormalAxis(const Eigen::MatrixXd& normals);
	
	/**
	 * \brief Unpack the parameters of the quadric.
	*/
	void unpackQuadric();
	
	/** \brief Solve the generalized Eigen problem A * v(j) = lambda(j) * B * v(j), where v
   * are the Eigen vectors, and lambda are the Eigen values. The eigenvalues are stored as:
   * (lambda(:, 1) + lambda(:, 2)*i)./lambda(:, 3). This method returns true if the Eigen
   * problem is solved successfully.
   * \param A the matrix A in the problem
   * \param B the matrix B in the problem
   * \param v the resultant Eigen vectors
   * \param lambda the resultant Eigen vectors (see above)
   * \return true if the solution process worked properly, false otherwise
   */
  bool solveGeneralizedEigenProblem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& v,
                                    Eigen::MatrixXd& lambda);

  bool is_deterministic_; ///< whether the local axes estimation is deterministic
  Eigen::Matrix3Xd cam_origins_; ///< the camera positions
  Eigen::Vector3d sample_; ///< the sample for which the point neighborhood was found
  int majority_cam_source_; ///< the majority camera source
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_; ///< the input point cloud
  Eigen::Vector3d curvature_axis_, normal_, binormal_; ///< the curvature, normal, and binormal axis
  Eigen::Matrix<double, 10, 1> parameters_; ///< the parameters of the quadric (implicit form)
  Eigen::Vector3d centroid_; ///< the centroid of the quadric
  Eigen::Matrix3d covariance_matrix_; ///< the covariance matrix of the quadric
  double normals_ratio_; ///< the ratio between the normals of the quadric
  
  static const int TAUBIN_MATRICES_SIZE = 10; ///< size of matrices in Taubin Quadric Fitting
};

#endif // PCL_FEATURES_CURVATURE_ESTIMATION_TAUBIN_H_
