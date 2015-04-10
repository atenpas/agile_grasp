#include <agile_grasp/handle.h>

Handle::Handle(const std::vector<GraspHypothesis>& hand_list, const std::vector<int>& inliers) :
		hand_list_(hand_list), inliers_(inliers)
{
	setAxis();
	setDistAlongHandle();
	setGraspVariables();
	setGraspWidth();
}

void Handle::setAxis()
{
	Eigen::Matrix3Xd axis_mat(3, inliers_.size());
	for (int i = 0; i < inliers_.size(); i++)
	{
		axis_mat.col(i) = hand_list_[inliers_[i]].getAxis();
	}

	Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(axis_mat * axis_mat.transpose());
	Eigen::Vector3d eigen_values = eigen_solver.eigenvalues().real();
	Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors().real();
	int max_idx;
	eigen_values.maxCoeff(&max_idx);
	axis_ = eigen_vectors.col(max_idx);
}

void Handle::setDistAlongHandle()
{
	dist_along_handle_.resize(inliers_.size());
	for (int i = 0; i < inliers_.size(); i++)
	{
		dist_along_handle_(i) = axis_.transpose()
				* hand_list_[inliers_[i]].getGraspBottom();
	}
}

void Handle::setGraspVariables()
{
	double center_dist = (dist_along_handle_.maxCoeff() + dist_along_handle_.minCoeff()) / 2.0;
	double min_dist = 10000000;
	int min_idx = -1;
	for (int i = 0; i < dist_along_handle_.size(); i++)
	{
//		std::cout << "dist_along_handle_: " << dist_along_handle_(i) << "\n";
		double dist = fabs(dist_along_handle_(i) - center_dist);
		if (dist < min_dist)
		{
			min_dist = dist;
			min_idx = i;
		}
	}
//	std::cout << "min_idx: " << min_idx << "\n";
	center_ = hand_list_[inliers_[min_idx]].getGraspBottom();
	approach_ = hand_list_[inliers_[min_idx]].getApproach();
	hands_center_ = hand_list_[inliers_[min_idx]].getGraspSurface();
//	std::cout << "center_: " << center_.transpose() << std::endl;
//	std::cout << "approach_: " << approach_.transpose() << std::endl;
//	std::cout << "axis_: " << axis_.transpose() << std::endl;
//	std::cout << "hands_center_: " << hands_center_.transpose() << std::endl;
//	std::cout << std::endl;
}

void Handle::setGraspWidth()
{
	width_ = 0.0;
	for (int i = 0; i < inliers_.size(); i++)
	{
		width_ += hand_list_[inliers_[i]].getGraspWidth();
	}
	width_ /= (double) inliers_.size();
}

