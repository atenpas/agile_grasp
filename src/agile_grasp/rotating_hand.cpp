#include <agile_grasp/rotating_hand.h>


RotatingHand::RotatingHand(const Eigen::VectorXd& camera_origin_left,
	const Eigen::VectorXd& camera_origin_right, const FingerHand& finger_hand, bool tolerant_antipodal,
	int cam_source) :
		finger_hand_(finger_hand), tolerant_antipodal_(tolerant_antipodal), cam_source_(cam_source)
{
	cams_.col(0) = camera_origin_left;
	cams_.col(1) = camera_origin_right;

	// generate hand orientations
	int num_orientations = 8;
	Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(num_orientations + 1, -1.0 * M_PI, M_PI);
	angles_ = angles.block(0, 0, 1, num_orientations);
}


void RotatingHand::transformPoints(const Eigen::Matrix3Xd& points, const Eigen::Vector3d& normal,
	const Eigen::Vector3d& axis, const Eigen::Matrix3Xd& normals, const Eigen::VectorXi& points_cam_source,
	double hand_height)
{
	// rotate points into hand frame
	hand_axis_ = axis;
	frame_ << normal, normal.cross(axis), axis;
	points_ = frame_.transpose() * points;
//  std::cout << "transformPoints(): " << std::endl;
//  std::cout << " points: " << points.rows() << " x " << points.cols() << std::endl;
//  std::cout << " frame_: " << frame_.rows() << " x " << frame_.cols() << std::endl;
//  std::cout << frame_ << std::endl;
//  std::cout << " points_: " << points_.rows() << " x " << points_.cols() << std::endl;

	normals_ = frame_.transpose() * normals;
	points_cam_source_ = points_cam_source;

	// crop points by hand_height
	std::vector<int> indices(points_.size());
	int k = 0;
//  std::cout << "h.pts: " << points_.cols() << ", -1.0 * hand_height: " << -1.0 * hand_height << "\n";
	for (int i = 0; i < points_.cols(); i++)
	{
//    std::cout << "i: " << i << " " << points_.col(i).transpose() << "\n";

		if (points_(2, i) > -1.0 * hand_height && points_(2, i) < hand_height)
		{
//      std::cout << " hand_height i: " << i << " " << points_(2, i) << "\n";
//      indices.push_back(i);
			indices[k] = i;
			k++;
		}
	}
//  indices[k+1] = -1;
//
////  std::cout << "indices.size(): " << indices.size() << std::endl;
//
////  for (int i = 0; i < indices.size(); i++)
////    std::cout << indices[i] << " ";
////  std::cout << "\n";

	Eigen::Matrix3Xd points_cropped(3, k);
	Eigen::Matrix3Xd normals_cropped(3, k);
	Eigen::VectorXi points_cam_source_cropped(k);
	for (int i = 0; i < k; i++)
	{
		points_cropped.col(i) = points_.col(indices[i]);
		normals_cropped.col(i) = normals_.col(indices[i]);
		points_cam_source_cropped(i) = points_cam_source_(indices[i]);
	}

//  std::cout << normals_cropped.col(0) << std::endl;

	points_ = points_cropped;
	normals_ = normals_cropped;
	points_cam_source_ = points_cam_source_cropped;
}


std::vector<GraspHypothesis> RotatingHand::evaluateHand(double init_bite, const Eigen::Vector3d& sample,
	bool use_antipodal)
{
	// initialize hands to zero
	int n = angles_.size();

	std::vector<GraspHypothesis> grasp_list;

	for (int i = 0; i < n; i++)
	{
		// rotate points into <angles_(i)> orientation
		Eigen::Matrix3d rot;
		rot << cos(angles_(i)), -1.0 * sin(angles_(i)), 0.0, sin(angles_(i)), cos(angles_(i)), 0.0, 0.0, 0.0, 1.0;
		Eigen::Matrix3Xd points_rot = rot * points_;
		Eigen::Matrix3Xd normals_rot = rot * normals_;
		Eigen::Vector3d x, y;
		x << 1.0, 0.0, 0.0;
		y << 0.0, 1.0, 0.0;
		Eigen::Vector3d approach = frame_ * rot.transpose() * y;

		// reject hand if it is not within 90 degrees of one of the camera sources
		if (approach.dot(cams_.col(0)) > 0 && approach.dot(cams_.col(1)) > 0)
		{
			continue;
		}

		Eigen::Vector3d binormal = frame_ * rot.transpose() * x;

		// calculate free hand placements
		finger_hand_.setPoints(points_rot);
		finger_hand_.evaluateFingers(init_bite);
		finger_hand_.evaluateHand();

		if (finger_hand_.getHand().sum() > 0)
		{
			// find deepest hand
			finger_hand_.deepenHand(init_bite, finger_hand_.getHandDepth());
			finger_hand_.evaluateGraspParameters(init_bite);

			// save grasp parameters
			Eigen::Vector3d surface, bottom;
			surface << finger_hand_.getGraspSurface(), 0.0;
			bottom << finger_hand_.getGraspBottom(), 0.0;
			surface = frame_ * rot.transpose() * surface;
			bottom = frame_ * rot.transpose() * bottom;

			// save data for learning
			std::vector<int> points_in_box_indices;
			for (int j = 0; j < points_rot.cols(); j++)
			{
				if (points_rot(1, j) < finger_hand_.getBackOfHand() + finger_hand_.getHandDepth())
					points_in_box_indices.push_back(j);
			}

			Eigen::Matrix3Xd points_in_box(3, points_in_box_indices.size());
			Eigen::Matrix3Xd normals_in_box(3, points_in_box_indices.size());
			Eigen::VectorXi cam_source_in_box(points_in_box_indices.size());

			for (int j = 0; j < points_in_box_indices.size(); j++)
			{
				points_in_box.col(j) = points_rot.col(points_in_box_indices[j]) - surface;
				normals_in_box.col(j) = normals_rot.col(points_in_box_indices[j]);
				cam_source_in_box(j) = points_cam_source_(points_in_box_indices[j]);
			}

			std::vector<int> indices_cam1;
			std::vector<int> indices_cam2;
			for (int j = 0; j < points_in_box.cols(); j++)
			{
				if (cam_source_in_box(j) == 0)
					indices_cam1.push_back(j);
				else if (cam_source_in_box(j) == 1)
					indices_cam2.push_back(j);
			}

			surface += sample;
			bottom += sample;

			GraspHypothesis grasp(hand_axis_, approach, binormal, bottom, surface, finger_hand_.getGraspWidth(),
				points_in_box, indices_cam1, indices_cam2, cam_source_);

			if (use_antipodal) // if we need to evaluate whether the grasp is antipodal
			{
				Antipodal antipodal(normals_in_box);
				int antipodal_type = antipodal.evaluateGrasp(20, 20);
				if (antipodal_type == Antipodal::FULL_GRASP)
				{
					grasp.setHalfAntipodal(true);
					grasp.setFullAntipodal(true);
				}
				else if (antipodal_type == Antipodal::HALF_GRASP)
					grasp.setHalfAntipodal(true);
			}

			grasp_list.push_back(grasp);
		}
	}

	return grasp_list;
}
