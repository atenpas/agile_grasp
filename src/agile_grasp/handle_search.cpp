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

void HandleSearch::plotHandles(const std::vector<Handle>& handle_list, const PointCloud::Ptr& cloud,
		std::string str)
{
  std::cout << "Drawing " << handle_list.size() << " handles.\n";
  
	double colors[9][3] = { { 0, 0.4470, 0.7410 }, { 0.8500, 0.3250, 0.0980 }, { 0.9290, 0.6940, 0.1250 }, {
			0.4940, 0.1840, 0.5560 }, { 0.4660, 0.6740, 0.1880 }, { 0.3010, 0.7450, 0.9330 }, { 0.6350, 0.0780,
			0.1840 }, { 0, 0.4470, 0.7410 }, { 0.8500, 0.3250, 0.0980 } };

	//	    0.9290    0.6940    0.1250
	//	    0.4940    0.1840    0.5560
	//	    0.4660    0.6740    0.1880
	//	    0.3010    0.7450    0.9330
	//	    0.6350    0.0780    0.1840
	//	         0    0.4470    0.7410
	//	    0.8500    0.3250    0.0980
	//	    0.9290    0.6940    0.1250
	//	    0.4940    0.1840    0.5560
	//	    0.4660    0.6740    0.1880
	//	    0.3010    0.7450    0.9330
	//	    0.6350    0.0780    0.1840

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;
	pcl::PointCloud<pcl::PointNormal>::Ptr handle_cloud(new pcl::PointCloud<pcl::PointNormal>());

	for (int i = 0; i < handle_list.size(); i++)
	{
		pcl::PointNormal p;
		p.x = handle_list[i].getHandsCenter()(0);
		p.y = handle_list[i].getHandsCenter()(1);
		p.z = handle_list[i].getHandsCenter()(2);
		p.normal[0] = -handle_list[i].getApproach()(0);
		p.normal[1] = -handle_list[i].getApproach()(1);
		p.normal[2] = -handle_list[i].getApproach()(2);
		handle_cloud->points.push_back(p);

		const std::vector<int>& inliers = handle_list[i].getInliers();
		const std::vector<GraspHypothesis>& hand_list = handle_list[i].getHandList();
		pcl::PointCloud<pcl::PointNormal>::Ptr axis_cloud(new pcl::PointCloud<pcl::PointNormal>);

		for (int j = 0; j < inliers.size(); j++)
		{
			pcl::PointNormal p;
			p.x = hand_list[inliers[j]].getGraspSurface()(0);
			p.y = hand_list[inliers[j]].getGraspSurface()(1);
			p.z = hand_list[inliers[j]].getGraspSurface()(2);
			p.normal[0] = -hand_list[inliers[j]].getApproach()(0);
			p.normal[1] = -hand_list[inliers[j]].getApproach()(1);
			p.normal[2] = -hand_list[inliers[j]].getApproach()(2);
			axis_cloud->points.push_back(p);
		}
		clouds.push_back(axis_cloud);
	}

	std::string title = "Handles: " + str;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(title));
	viewer->setBackgroundColor(1, 1, 1);
  //viewer->setPosition(0, 0);
  //viewer->setSize(640, 480);  
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
			"registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,
			"registered point cloud");

	for (int i = 0; i < clouds.size(); i++)
	{
		std::string name = "handle_" + boost::lexical_cast<std::string>(i);
		int ci = i % 6;
//		std::cout << "ci: " << ci << "\n";
		viewer->addPointCloud<pcl::PointNormal>(clouds[i], name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[ci][0],
				colors[ci][1], colors[ci][2], name);
		std::string name2 = "approach_" + boost::lexical_cast<std::string>(i);
		viewer->addPointCloudNormals<pcl::PointNormal>(clouds[i], 1, 0.04, name2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, name2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[ci][0],
				colors[ci][1], colors[ci][2], name2);
	}

	viewer->addPointCloud<pcl::PointNormal>(handle_cloud, "handles");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "handles");
	viewer->addPointCloudNormals<pcl::PointNormal>(handle_cloud, 1, 0.08, "approach");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "approach");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "approach");

	// viewer->addCoordinateSystem(1.0, "", 0);
	viewer->initCameraParameters();
  viewer->setPosition(0, 0);
	viewer->setSize(640, 480);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
  
  viewer->close();

	//	std::vector<pcl::visualization::Camera> cam;
	//
	//	// print the position of the camera
	//	viewer->getCameras(cam);
	//	std::cout << "Cam: " << std::endl << " - pos: (" << cam[0].pos[0] << ", " << cam[0].pos[1] << ", "
	//			<< cam[0].pos[2] << ")" << std::endl << " - view: (" << cam[0].view[0] << ", " << cam[0].view[1] << ", "
	//			<< cam[0].view[2] << ")" << std::endl << " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1]
	//			<< ", " << cam[0].focal[2] << ")" << std::endl;
}

double HandleSearch::safeAcos(double x)
{
	if (x < -1.0)
		x = -1.0;
	else if (x > 1.0)
		x = 1.0;
	return acos(x);
}
