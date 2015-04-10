#include <agile_grasp/hand_search.h>


std::vector<GraspHypothesis> HandSearch::findHands(const PointCloud::Ptr cloud, 
  const Eigen::VectorXi& pts_cam_source, const std::vector<int>& indices, 
  const PointCloud::Ptr cloud_plot, bool calculates_antipodal, 
  bool uses_clustering)
{
  // create KdTree for neighborhood search
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	cloud_normals_.resize(3, cloud->size());
	cloud_normals_.setZero(3, cloud->size());

	// calculate normals for all points
	if (calculates_antipodal)
	{
		std::cout << "Calculating normals for all points\n";
		nn_radius_taubin_ = 0.01;
		std::vector<int> indices_cloud(cloud->size());
		for (int i = 0; i < indices_cloud.size(); i++)
			indices_cloud[i] = i;
		findQuadrics(cloud, pts_cam_source, kdtree, indices_cloud);
		nn_radius_taubin_ = 0.03;
	}

	// draw samples from the point cloud uniformly
	std::vector<int> indices_rand;
	Eigen::VectorXi hands_cam_source;
	if (indices.size() == 0)
	{
		double t_rand = omp_get_wtime();
		std::cout << "Generating uniform random indices ...\n";
		indices_rand.resize(num_samples_);
		pcl::RandomSample<pcl::PointXYZ> random_sample;
		random_sample.setInputCloud(cloud);
		random_sample.setSample(num_samples_);
		random_sample.filter(indices_rand);
		hands_cam_source.resize(num_samples_);
		for (int i = 0; i < num_samples_; i++)
			hands_cam_source(i) = pts_cam_source(indices_rand[i]);
		std::cout << " Done in " << omp_get_wtime() - t_rand << " sec\n";
	}
	else
		indices_rand = indices;

	if (plots_samples_)
		plot_.plotSamples(indices_rand, cloud);

	// find quadrics
	std::cout << "Estimating local axes ...\n";
	std::vector<Quadric> quadric_list = findQuadrics(cloud, pts_cam_source, kdtree, indices_rand);
	if (plots_local_axes_)
		plot_.plotLocalAxes(quadric_list, cloud_plot);

	// find hands
	std::cout << "Finding hand poses ...\n";
	std::vector<GraspHypothesis> hand_list = findHands(cloud, pts_cam_source, quadric_list, hands_cam_source, kdtree);
  
  return hand_list;
}


std::vector<Quadric> HandSearch::findQuadrics(const PointCloud::Ptr cloud,
	const Eigen::VectorXi& pts_cam_source, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
	const std::vector<int>& indices)
{
	double t1 = omp_get_wtime();
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > T_cams;
	T_cams.push_back(cam_tf_left_);
	T_cams.push_back(cam_tf_right_);
	std::vector<Quadric> quadric_list(indices.size());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists) num_threads(num_threads_)
#endif
	for (int i = 0; i < indices.size(); i++)
	{
		const pcl::PointXYZ& sample = cloud->points[indices[i]];
//    std::cout << "i: " << i << ", index: " << indices[i] << ", sample: " << sample << std::endl;

		if (kdtree.radiusSearch(sample, nn_radius_taubin_, nn_indices, nn_dists) > 0)
		{
			Eigen::VectorXi nn_cam_source(nn_indices.size());
//      std::cout << " Found " << nn_indices.size() << " neighbors.\n";

			for (int j = 0; j < nn_cam_source.size(); j++)
			{
				nn_cam_source(j) = pts_cam_source(nn_indices[j]);
			}

			Eigen::Vector3d sample_eigen = sample.getVector3fMap().cast<double>();
			Quadric q(T_cams, cloud, sample_eigen, uses_determinstic_normal_estimation_);
			q.fitQuadric(nn_indices);
//      std::cout << " Fitted quadric\n";
			q.findTaubinNormalAxis(nn_indices, nn_cam_source);
//      std::cout << " Found local axes\n";
			quadric_list[i] = q;
			cloud_normals_.col(indices[i]) = q.getNormal();
		}
	}

	double t2 = omp_get_wtime();
	std::cout << "Fitted " << quadric_list.size() << " quadrics in " << t2 - t1 << " sec.\n";

//  quadric_list[0].print(); // debugging
//  plot_.plotLocalAxes(quadric_list, cloud);

	return quadric_list;
}


std::vector<GraspHypothesis> HandSearch::findHands(const PointCloud::Ptr cloud,
	const Eigen::VectorXi& pts_cam_source, const std::vector<Quadric>& quadric_list,
	const Eigen::VectorXi& hands_cam_source, const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree)
{
	double t1 = omp_get_wtime();
	std::vector<int> nn_indices;
	std::vector<float> nn_dists;
	Eigen::Matrix3Xd nn_normals(3, nn_indices.size());
	Eigen::VectorXi nn_cam_source(nn_indices.size());
	Eigen::Matrix3Xd centered_neighborhood(3, nn_indices.size());
	std::vector<RotatingHand> hand_list(quadric_list.size());
//  std::vector<RotatingHand> hand_list;
	double time_eval_hand = 0.0;
	double time_iter = 0.0;
	double time_nn = 0.0;
	double time_tf = 0.0;

	std::vector< std::vector<GraspHypothesis> > grasp_lists(quadric_list.size(), std::vector<GraspHypothesis>(0));

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_normals, nn_cam_source, centered_neighborhood) num_threads(num_threads_)
#endif
	for (std::size_t i = 0; i < quadric_list.size(); i++)
	{
		double timei = omp_get_wtime();
		pcl::PointXYZ sample;
		sample.x = quadric_list[i].getSample()(0);
		sample.y = quadric_list[i].getSample()(1);
		sample.z = quadric_list[i].getSample()(2);
//    std::cout << "i: " << i << ", sample: " << sample << std::endl;

		if (kdtree.radiusSearch(sample, nn_radius_hands_, nn_indices, nn_dists) > 0)
		{
			time_nn += omp_get_wtime() - timei;
			nn_normals.setZero(3, nn_indices.size());
			nn_cam_source.setZero(nn_indices.size());
			centered_neighborhood.setZero(3, nn_indices.size());

			for (int j = 0; j < nn_indices.size(); j++)
			{
				nn_cam_source(j) = pts_cam_source(nn_indices[j]);
				centered_neighborhood.col(j) = (cloud->points[nn_indices[j]].getVector3fMap()
						- sample.getVector3fMap()).cast<double>();
				nn_normals.col(j) = cloud_normals_.col(nn_indices[j]);
			}

			FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

			Eigen::Vector3d sample_eig = sample.getVector3fMap().cast<double>();
			RotatingHand rotating_hand(cam_tf_left_.block<3, 1>(0, 3) - sample_eig,
				cam_tf_right_.block<3, 1>(0, 3) - sample_eig, finger_hand, tolerant_antipodal_, hands_cam_source(i));
			const Quadric& q = quadric_list[i];
			double time_tf1 = omp_get_wtime();
			rotating_hand.transformPoints(centered_neighborhood, q.getNormal(), q.getCurvatureAxis(), nn_normals,
				nn_cam_source, hand_height_);
			time_tf += omp_get_wtime() - time_tf1;
			double time_eval1 = omp_get_wtime();
			std::vector<GraspHypothesis> grasps = rotating_hand.evaluateHand(init_bite_, sample_eig, true);
			time_eval_hand += omp_get_wtime() - time_eval1;

			if (grasps.size() > 0)
			{
				// grasp_list.insert(grasp_list.end(), grasps.begin(), grasps.end());
        grasp_lists[i] = grasps;
			}
		}

		time_iter += omp_get_wtime() - timei;
	}
	time_eval_hand /= quadric_list.size();
	time_nn /= quadric_list.size();
	time_iter /= quadric_list.size();
	time_tf /= quadric_list.size();
	std::cout << " avg time for transforming point neighborhood: " << time_tf << " sec.\n";
	std::cout << " avg time for NN search: " << time_nn << " sec.\n";
	std::cout << " avg time for rotating_hand.evaluate(): " << time_eval_hand << " sec.\n";
	std::cout << " avg time per iteration: " << time_iter << " sec.\n";
  
  std::vector<GraspHypothesis> grasp_list;
  for (std::size_t i = 0; i < grasp_lists.size(); i++)
  {
    // std::cout << i << " " << grasp_lists[i].size() << "\n";
    if (grasp_lists[i].size() > 0)
      grasp_list.insert(grasp_list.end(), grasp_lists[i].begin(), grasp_lists[i].end());
  }

	double t2 = omp_get_wtime();
	std::cout << " Found " << grasp_list.size() << " robot hand poses in " << t2 - t1 << " sec.\n";

	return grasp_list;
}
