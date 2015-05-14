#include <agile_grasp/handle_search.h>


std::vector<Handle> HandleSearch::findHandles(const std::vector<GraspHypothesis>& hand_list, int min_inliers,
  double min_length)
{
  double t0 = omp_get_wtime();
  std::vector<Handle> handle_list;
  std::vector<GraspHypothesis> reduced_hand_list(hand_list);

	for (int i = 0; i < hand_list.size(); i++)
	{
    if (reduced_hand_list[i].getGraspWidth() == -1)
      continue;
    
    Eigen::Vector3d iaxis = reduced_hand_list[i].getAxis();
		Eigen::Vector3d ipt = reduced_hand_list[i].getGraspBottom();
		Eigen::Vector3d inormal = reduced_hand_list[i].getApproach();
    std::vector<Eigen::Vector2d> inliers;
    
    for (int j = 0; j < hand_list.size(); j++)
    {
      if (reduced_hand_list[j].getGraspWidth() == -1)
        continue;
      
      Eigen::Vector3d jaxis = reduced_hand_list[j].getAxis();
			Eigen::Vector3d jpt = reduced_hand_list[j].getGraspBottom();
			Eigen::Vector3d jnormal = reduced_hand_list[j].getApproach();
      
      // how good is this match?
			double dist_from_line = ((Eigen::Matrix3d::Identity(3, 3) - iaxis * iaxis.transpose()) * (jpt - ipt)).norm();
			double dist_along_line = iaxis.transpose() * (jpt - ipt);
			Eigen::Vector2d angle_axis;
			angle_axis << safeAcos(iaxis.transpose() * jaxis), M_PI - safeAcos(iaxis.transpose() * jaxis);
			double dist_angle_axis = angle_axis.minCoeff();
			double dist_from_normal = safeAcos(inormal.transpose() * jnormal);
			if (dist_from_line < 0.01 && dist_angle_axis < 0.34 && dist_from_normal < 0.34)
			{
        Eigen::Vector2d inlier;
				inlier << j, dist_along_line;
				inliers.push_back(inlier);
			}
    }
    
    if (inliers.size() < min_inliers)
			continue;

		// shorten handle to a continuous piece
		double handle_gap_threshold = 0.02;
		std::vector<Eigen::Vector2d> inliers_list(inliers.begin(), inliers.end());
		while (!shortenHandle(inliers_list, handle_gap_threshold))
		{	};
    
    if (inliers_list.size() < min_inliers)
      continue;
    
		// find grasps farthest away from i-th grasp 
    double min_dist = 10000000;
		double max_dist = -10000000;
    std::vector<int> in(inliers_list.size());
		for (int k = 0; k < inliers_list.size(); k++)
		{
      in[k] = inliers_list[k](0);
      
			if (inliers_list[k](1) < min_dist)
				min_dist = inliers_list[k](1);
			if (inliers_list[k](1) > max_dist)
				max_dist = inliers_list[k](1);
		}
    
    // handle found
    if (max_dist - min_dist > min_length)
    {
      handle_list.push_back(Handle(hand_list, in));
      std::cout << "handle found with " << in.size() << " inliers\n";
      
      // eliminate hands in this handle from future search
			for (int k = 0; k < in.size(); k++)
			{
				reduced_hand_list[in[k]].setGraspWidth(-1);
			}
    }
  }
  
  std::cout << "Handle Search\n";
  std::cout << " runtime: " << omp_get_wtime() - t0 << " sec\n";
	std::cout << " " << handle_list.size() << " handles found\n"; 
  return handle_list; 
}


bool HandleSearch::shortenHandle(std::vector<Eigen::Vector2d> &inliers, double gap_threshold)
{
	std::sort(inliers.begin(), inliers.end(), HandleSearch::LastElementComparator());
	bool is_done = true;

	for (int i = 0; i < inliers.size() - 1; i++)
	{
		double diff = inliers[i + 1](1) - inliers[i](1);
		if (diff > gap_threshold)
		{
			std::vector<Eigen::Vector2d> out;
			if (inliers[i](2) < 0)
			{
				out.assign(inliers.begin() + i + 1, inliers.end());
				is_done = false;
			}
			else
			{
				out.assign(inliers.begin(), inliers.begin() + i);
			}
			inliers = out;
			break;
		}
	}

	return is_done;
}


double HandleSearch::safeAcos(double x)
{
	if (x < -1.0)
		x = -1.0;
	else if (x > 1.0)
		x = 1.0;
	return acos(x);
}
