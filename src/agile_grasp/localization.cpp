#include <agile_grasp/localization.h>

std::vector<GraspHypothesis> Localization::localizeHands(const PointCloud::Ptr& cloud_in, int size_left,
	const std::vector<int>& indices, bool calculates_antipodal, bool uses_clustering)
{		
	double t0 = omp_get_wtime();
	std::vector<GraspHypothesis> hand_list;
	
	if (size_left == 0 || cloud_in->size() == 0)
	{
		std::cout << "Input cloud is empty!\n";
		std::cout << size_left << std::endl;
		hand_list.resize(0);
		return hand_list;
	}
	
	// set camera source for all points (0 = left, 1 = right)
	std::cout << "Generating camera sources for " << cloud_in->size() << " points ...\n";
	Eigen::VectorXi pts_cam_source(cloud_in->size());
	if (size_left == cloud_in->size())
		pts_cam_source << Eigen::VectorXi::Zero(size_left);
	else
		pts_cam_source << Eigen::VectorXi::Zero(size_left), Eigen::VectorXi::Ones(cloud_in->size() - size_left);
		
	// remove NAN points from the cloud
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, nan_indices);

	// reduce point cloud to workspace
	std::cout << "Filtering workspace ...\n";
	PointCloud::Ptr cloud(new PointCloud);
	filterWorkspace(cloud_in, pts_cam_source, cloud, pts_cam_source);
	std::cout << " " << cloud->size() << " points left\n";

	// store complete cloud for later plotting
	PointCloud::Ptr cloud_plot(new PointCloud);
	*cloud_plot = *cloud;
	*cloud_ = *cloud;

	// voxelize point cloud
	std::cout << "Voxelizing point cloud\n";
	double t1_voxels = omp_get_wtime();
	voxelizeCloud(cloud, pts_cam_source, cloud, pts_cam_source, 0.003);
	double t2_voxels = omp_get_wtime() - t1_voxels;
	std::cout << " Created " << cloud->points.size() << " voxels in " << t2_voxels << " sec\n";

	// plot camera source for each point in the cloud
	if (plots_camera_sources_)
		plot_.plotCameraSource(pts_cam_source, cloud);

	if (uses_clustering)
	{
    std::cout << "Finding point cloud clusters ... \n";
        
		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.01);

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << " Could not estimate a planar model for the given dataset." << std::endl;
			hand_list.resize(0);
			return hand_list;
		}
    
    std::cout << " PointCloud representing the planar component: " << inliers->indices.size()
      << " data points." << std::endl;

		// Extract the nonplanar inliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		std::vector<int> indices_cluster;
		extract.filter(indices_cluster);
		PointCloud::Ptr cloud_cluster(new PointCloud);
		cloud_cluster->points.resize(indices_cluster.size());
		Eigen::VectorXi cluster_cam_source(indices_cluster.size());
		for (int i = 0; i < indices_cluster.size(); i++)
		{
			cloud_cluster->points[i] = cloud->points[indices_cluster[i]];
			cluster_cam_source[i] = pts_cam_source[indices_cluster[i]];
		}
		cloud = cloud_cluster;
		*cloud_plot = *cloud;
		std::cout << " PointCloud representing the non-planar component: " << cloud->points.size()
      << " data points." << std::endl;
	}

	// draw down-sampled and workspace reduced cloud
	cloud_plot = cloud;
  
  // set plotting within handle search on/off  
  bool plots_hands;
  if (plotting_mode_ == PCL_PLOTTING)
		plots_hands = true;
  else
		plots_hands = false;
		
	// find hand configurations
  HandSearch hand_search(finger_width_, hand_outer_diameter_, hand_depth_, hand_height_, init_bite_, num_threads_, 
		num_samples_, plots_hands);
	hand_list = hand_search.findHands(cloud, pts_cam_source, indices, cloud_plot, calculates_antipodal, uses_clustering);

	// remove hands at boundaries of workspace
	if (filters_boundaries_)
  {
    std::cout << "Filtering out hands close to workspace boundaries ...\n";
    hand_list = filterHands(hand_list);
    std::cout << " # hands left: " << hand_list.size() << "\n";
  }

	double t2 = omp_get_wtime();
	std::cout << "Hand localization done in " << t2 - t0 << " sec\n";

	if (plotting_mode_ == PCL_PLOTTING)
	{
		plot_.plotHands(hand_list, cloud_plot, "");
	}
	else if (plotting_mode_ == RVIZ_PLOTTING)
	{
		plot_.plotGraspsRviz(hand_list, visuals_frame_);
	}

	return hand_list;
}

std::vector<GraspHypothesis> Localization::predictAntipodalHands(const std::vector<GraspHypothesis>& hand_list, 
	const std::string& svm_filename)
{
	double t0 = omp_get_wtime();
	std::vector<GraspHypothesis> antipodal_hands;
	Learning learn(num_threads_);
	Eigen::Matrix<double, 3, 2> cams_mat;
	cams_mat.col(0) = cam_tf_left_.block<3, 1>(0, 3);
	cams_mat.col(1) = cam_tf_right_.block<3, 1>(0, 3);
	antipodal_hands = learn.classify(hand_list, svm_filename, cams_mat);
	std::cout << " runtime: " << omp_get_wtime() - t0 << " sec\n";
	std::cout << antipodal_hands.size() << " antipodal hand configurations found\n"; 
  if (plotting_mode_ == PCL_PLOTTING)
		plot_.plotHands(hand_list, antipodal_hands, cloud_, "Antipodal Hands");
	else if (plotting_mode_ == RVIZ_PLOTTING)
		plot_.plotGraspsRviz(antipodal_hands, visuals_frame_, true);
	return antipodal_hands;
}

std::vector<GraspHypothesis> Localization::localizeHands(const std::string& pcd_filename_left,
	const std::string& pcd_filename_right, bool calculates_antipodal, bool uses_clustering)
{
	std::vector<int> indices(0);
	return localizeHands(pcd_filename_left, pcd_filename_right, indices, calculates_antipodal, uses_clustering);
}

std::vector<GraspHypothesis> Localization::localizeHands(const std::string& pcd_filename_left,
	const std::string& pcd_filename_right, const std::vector<int>& indices, bool calculates_antipodal,
	bool uses_clustering)
{
	double t0 = omp_get_wtime();

	// load point clouds
	PointCloud::Ptr cloud_left(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename_left, *cloud_left) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read pcd_filename_left file \n");
		std::vector<GraspHypothesis> hand_list(0);
		return hand_list;
	}
	if (pcd_filename_right.length() > 0)
		std::cout << "Loaded left point cloud with " << cloud_left->width * cloud_left->height << " data points.\n";
	else
		std::cout << "Loaded point cloud with " << cloud_left->width * cloud_left->height << " data points.\n";

	PointCloud::Ptr cloud_right(new PointCloud);
	if (pcd_filename_right.length() > 0)
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename_right, *cloud_right) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read pcd_filename_right file \n");
			std::vector<GraspHypothesis> hand_list(0);
			return hand_list;
		}
		std::cout << "Loaded right point cloud with " << cloud_right->width * cloud_right->height << " data points.\n";
		std::cout << "Loaded both clouds in " << omp_get_wtime() - t0 << " sec\n";
	}
	
	// concatenate point clouds
	std::cout << "Concatenating point clouds ...\n";
	PointCloud::Ptr cloud(new PointCloud);
	*cloud = *cloud_left + *cloud_right;

	return localizeHands(cloud, cloud_left->size(), indices, calculates_antipodal, uses_clustering);
}

void Localization::filterWorkspace(const PointCloud::Ptr& cloud_in, const Eigen::VectorXi& pts_cam_source_in,
	PointCloud::Ptr& cloud_out, Eigen::VectorXi& pts_cam_source_out)
{
	std::vector<int> indices(cloud_in->points.size());
	int c = 0;
	PointCloud::Ptr cloud(new PointCloud);
//  std::cout << "workspace_: " << workspace_.transpose() << "\n";

	for (int i = 0; i < cloud_in->points.size(); i++)
	{
//    std::cout << "i: " << i << "\n";
		const pcl::PointXYZ& p = cloud_in->points[i];
		if (p.x >= workspace_(0) && p.x <= workspace_(1) && p.y >= workspace_(2) && p.y <= workspace_(3)
				&& p.z >= workspace_(4) && p.z <= workspace_(5))
		{
			cloud->points.push_back(p);
			indices[c] = i;
			c++;
		}
	}

	Eigen::VectorXi pts_cam_source(c);
	for (int i = 0; i < pts_cam_source.size(); i++)
	{
		pts_cam_source(i) = pts_cam_source_in(indices[i]);
	}

	cloud_out = cloud;
	pts_cam_source_out = pts_cam_source;
}

void Localization::voxelizeCloud(const PointCloud::Ptr& cloud_in, const Eigen::VectorXi& pts_cam_source_in,
	PointCloud::Ptr& cloud_out, Eigen::VectorXi& pts_cam_source_out, double cell_size)
{
	Eigen::Vector3d min_left, min_right;
	min_left << 10000, 10000, 10000;
	min_right << 10000, 10000, 10000;
	Eigen::Matrix3Xd pts(3, cloud_in->points.size());
	int num_left = 0;
	int num_right = 0;
	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		if (pts_cam_source_in(i) == 0)
		{
			if (cloud_in->points[i].x < min_left(0))
				min_left(0) = cloud_in->points[i].x;
			if (cloud_in->points[i].y < min_left(1))
				min_left(1) = cloud_in->points[i].y;
			if (cloud_in->points[i].z < min_left(2))
				min_left(2) = cloud_in->points[i].z;
			num_left++;
		}
		else if (pts_cam_source_in(i) == 1)
		{
			if (cloud_in->points[i].x < min_right(0))
				min_right(0) = cloud_in->points[i].x;
			if (cloud_in->points[i].y < min_right(1))
				min_right(1) = cloud_in->points[i].y;
			if (cloud_in->points[i].z < min_right(2))
				min_right(2) = cloud_in->points[i].z;
			num_right++;
		}
		pts.col(i) = cloud_in->points[i].getVector3fMap().cast<double>();
	}

	// find the cell that each point falls into
	std::set<Eigen::Vector3i, Localization::UniqueVectorComparator> bins_left;
	std::set<Eigen::Vector3i, Localization::UniqueVectorComparator> bins_right;
	int prev;
	for (int i = 0; i < pts.cols(); i++)
	{
		if (pts_cam_source_in(i) == 0)
		{
			Eigen::Vector3i v = floorVector((pts.col(i) - min_left) / cell_size);
			bins_left.insert(v);
			prev = bins_left.size();
		}
		else if (pts_cam_source_in(i) == 1)
		{
			Eigen::Vector3i v = floorVector((pts.col(i) - min_right) / cell_size);
			bins_right.insert(v);
		}
	}

	// calculate the cell values
	Eigen::Matrix3Xd voxels_left(3, bins_left.size());
	Eigen::Matrix3Xd voxels_right(3, bins_right.size());
	int i = 0;
	for (std::set<Eigen::Vector3i, Localization::UniqueVectorComparator>::iterator it = bins_left.begin();
			it != bins_left.end(); it++)
	{
		voxels_left.col(i) = (*it).cast<double>();
		i++;
	}
	i = 0;
	for (std::set<Eigen::Vector3i, Localization::UniqueVectorComparator>::iterator it = bins_right.begin();
			it != bins_right.end(); it++)
	{
		voxels_right.col(i) = (*it).cast<double>();
		i++;
	}

	voxels_left.row(0) = voxels_left.row(0) * cell_size
			+ Eigen::VectorXd::Ones(voxels_left.cols()) * min_left(0);
	voxels_left.row(1) = voxels_left.row(1) * cell_size
			+ Eigen::VectorXd::Ones(voxels_left.cols()) * min_left(1);
	voxels_left.row(2) = voxels_left.row(2) * cell_size
			+ Eigen::VectorXd::Ones(voxels_left.cols()) * min_left(2);
	voxels_right.row(0) = voxels_right.row(0) * cell_size
			+ Eigen::VectorXd::Ones(voxels_right.cols()) * min_right(0);
	voxels_right.row(1) = voxels_right.row(1) * cell_size
			+ Eigen::VectorXd::Ones(voxels_right.cols()) * min_right(1);
	voxels_right.row(2) = voxels_right.row(2) * cell_size
			+ Eigen::VectorXd::Ones(voxels_right.cols()) * min_right(2);

	PointCloud::Ptr cloud(new PointCloud);
	cloud->resize(bins_left.size() + bins_right.size());
	Eigen::VectorXi pts_cam_source(bins_left.size() + bins_right.size());
	for (int i = 0; i < bins_left.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = voxels_left(0, i);
		p.y = voxels_left(1, i);
		p.z = voxels_left(2, i);
		cloud->points[i] = p;
		pts_cam_source(i) = 0;
	}
	for (int i = 0; i < bins_right.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = voxels_right(0, i);
		p.y = voxels_right(1, i);
		p.z = voxels_right(2, i);
		cloud->points[bins_left.size() + i] = p;
		pts_cam_source(bins_left.size() + i) = 1;
	}

	cloud_out = cloud;
	pts_cam_source_out = pts_cam_source;
}

Eigen::Vector3i Localization::floorVector(const Eigen::Vector3d& a)
{
	Eigen::Vector3i b;
	b << floor(a(0)), floor(a(1)), floor(a(2));
	return b;
}

std::vector<GraspHypothesis> Localization::filterHands(const std::vector<GraspHypothesis>& hand_list)
{
	const double MIN_DIST = 0.02;

	std::vector<GraspHypothesis> filtered_hand_list;

	for (int i = 0; i < hand_list.size(); i++)
	{
		const Eigen::Vector3d& center = hand_list[i].getGraspSurface();
		int k;
		for (k = 0; k < workspace_.size(); k++)
		{
			if (fabs((center(floor(k / 2.0)) - workspace_(k))) < MIN_DIST)
			{
				break;
			}
		}
		if (k == workspace_.size())
		{
			filtered_hand_list.push_back(hand_list[i]);
		}
	}

	return filtered_hand_list;
}

std::vector<Handle> Localization::findHandles(const std::vector<GraspHypothesis>& hand_list, int min_inliers,
	double min_length)
{
	HandleSearch handle_search;
	std::vector<Handle> handles = handle_search.findHandles(hand_list, min_inliers, min_length);
	if (plotting_mode_ == PCL_PLOTTING)
		plot_.plotHandles(handles, cloud_, "Handles");
	else if (plotting_mode_ == RVIZ_PLOTTING)
		plot_.plotHandlesRviz(handles, visuals_frame_);
	return handles;
}
