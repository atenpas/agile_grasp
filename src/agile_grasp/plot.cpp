#include <agile_grasp/plot.h>


void Plot::plotHands(const std::vector<GraspHypothesis>& hand_list,
	const std::vector<GraspHypothesis>& antipodal_hand_list, const PointCloud::Ptr& cloud, std::string str,
	bool use_grasp_bottom)
{
	PointCloudNormal::Ptr hands_cloud = createNormalsCloud(hand_list, false, false);
	PointCloudNormal::Ptr antipodal_hands_cloud = createNormalsCloud(antipodal_hand_list, true,
		false);
	plotHandsHelper(hands_cloud, antipodal_hands_cloud, cloud, str, use_grasp_bottom);
}


void Plot::plotHands(const std::vector<GraspHypothesis>& hand_list, const PointCloud::Ptr& cloud,
	std::string str, bool use_grasp_bottom)
{
	PointCloudNormal::Ptr hands_cloud = createNormalsCloud(hand_list, false, false);
	PointCloudNormal::Ptr antipodal_hands_cloud = createNormalsCloud(hand_list, true, false);
	plotHandsHelper(hands_cloud, antipodal_hands_cloud, cloud, str, use_grasp_bottom);
}


void Plot::plotSamples(const std::vector<int>& index_list, const PointCloud::Ptr& cloud)
{
	PointCloud::Ptr samples_cloud(new PointCloud);
	for (int i = 0; i < index_list.size(); i++)
		samples_cloud->points.push_back(cloud->points[index_list[i]]);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer("Samples");  
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
		"registered point cloud");
	viewer->addPointCloud<pcl::PointXYZ>(samples_cloud, "samples cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
		"samples cloud");

	runViewer(viewer);
}


void Plot::plotLocalAxes(const std::vector<Quadric>& quadric_list, const PointCloud::Ptr& cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer("Local Axes");
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
		"registered point cloud");

	for (int i = 0; i < quadric_list.size(); i++)
		quadric_list[i].plotAxes((void*) &viewer, i);

	runViewer(viewer);
}


void Plot::plotCameraSource(const Eigen::VectorXi& pts_cam_source_in, const PointCloud::Ptr& cloud)
{
	PointCloud::Ptr left_cloud(new PointCloud);
	PointCloud::Ptr right_cloud(new PointCloud);

	for (int i = 0; i < pts_cam_source_in.size(); i++)
	{
		if (pts_cam_source_in(i) == 0)
			left_cloud->points.push_back(cloud->points[i]);
		else if (pts_cam_source_in(i) == 1)
			right_cloud->points.push_back(cloud->points[i]);
	}
		
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer("Camera Sources");  
	viewer->addPointCloud<pcl::PointXYZ>(left_cloud, "left point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
		"left point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
		"left point cloud");
	viewer->addPointCloud<pcl::PointXYZ>(right_cloud, "right point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
		"right point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,
		"right point cloud");
	runViewer(viewer);
}


PointCloudNormal::Ptr Plot::createNormalsCloud(
	const std::vector<GraspHypothesis>& hand_list, bool plots_only_antipodal, bool plots_grasp_bottom)
{
	PointCloudNormal::Ptr cloud(new PointCloudNormal);

	for (int i = 0; i < hand_list.size(); i++)
	{
		Eigen::Matrix3Xd grasp_surface = hand_list[i].getGraspSurface();
		Eigen::Matrix3Xd grasp_bottom = hand_list[i].getGraspBottom();
		Eigen::Matrix3Xd hand_approach = hand_list[i].getApproach();

    if (!plots_only_antipodal || (plots_only_antipodal && hand_list[i].isFullAntipodal()))
    {
      pcl::PointNormal p;
      if (!plots_grasp_bottom)
      {
        p.x = grasp_surface(0);
        p.y = grasp_surface(1);
        p.z = grasp_surface(2);
      }
      else
      {
        p.x = grasp_bottom(0);
        p.y = grasp_bottom(1);
        p.z = grasp_bottom(2);
      }
      p.normal[0] = -hand_approach(0);
      p.normal[1] = -hand_approach(1);
      p.normal[2] = -hand_approach(2);
      cloud->points.push_back(p);
    }
	}

	return cloud;
}


void Plot::addCloudNormalsToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
	const PointCloudNormal::Ptr& cloud, double line_width, double* color_cloud,
	double* color_normals, const std::string& cloud_name, const std::string& normals_name)
{
	viewer->addPointCloud<pcl::PointNormal>(cloud, cloud_name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_cloud[0],
		color_cloud[1], color_cloud[2], cloud_name);
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.04, normals_name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width,
		normals_name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_normals[0],
		color_normals[1], color_normals[2], normals_name);
}


void Plot::plotHandsHelper(const PointCloudNormal::Ptr& hands_cloud,
	const PointCloudNormal::Ptr& antipodal_hands_cloud, const PointCloud::Ptr& cloud,
	std::string str, bool use_grasp_bottom)
{
	std::cout << "Drawing " << hands_cloud->size() << " grasps of which " << antipodal_hands_cloud->size()
			<< " are antipodal grasps.\n";

	std::string title = "Hands: " + str;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer(title);  
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
		"registered point cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.8, 0.0,
		"registered point cloud");

	double green[3] = { 0.0, 1.0, 0.0 };
	double cyan[3] = { 0.0, 0.4, 0.8 };
	addCloudNormalsToViewer(viewer, hands_cloud, 2, green, cyan, std::string("hands"),
		std::string("approaches"));

	if (antipodal_hands_cloud->size() > 0)
	{
		double red[3] = { 1.0, 0.0, 0.0 };
		addCloudNormalsToViewer(viewer, antipodal_hands_cloud, 2, green, red, std::string("antipodal_hands"),
			std::string("antipodal_approaches"));
	}

	runViewer(viewer);
}


void Plot::runViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
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


boost::shared_ptr<pcl::visualization::PCLVisualizer> Plot::createViewer(std::string title)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(title));  
  viewer->setBackgroundColor(1, 1, 1); 
  return viewer;
}
