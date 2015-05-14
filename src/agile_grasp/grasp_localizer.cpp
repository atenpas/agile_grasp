#include <agile_grasp/grasp_localizer.h>


GraspLocalizer::GraspLocalizer(ros::NodeHandle& node, const std::string& cloud_topic, 
	const std::string& cloud_frame, int cloud_type, const std::string& svm_file_name, 
	const Parameters& params) 
  : cloud_left_(new PointCloud()), cloud_right_(new PointCloud()), 
  cloud_frame_(cloud_frame), svm_file_name_(svm_file_name), num_clouds_(params.num_clouds_), 
  num_clouds_received_(0), size_left_(0)
{
  // subscribe to input point cloud ROS topic
  if (cloud_type == CLOUD_SIZED)
		cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizer::cloud_sized_callback, this);
	else if (cloud_type == POINT_CLOUD_2)
		cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizer::cloud_callback, this);
  
  // create ROS publisher for grasps
  grasps_pub_ = node.advertise<agile_grasp::Grasps>("grasps", 10);
  
  // create localization object and initialize its parameters
  localization_ = new Localization(params.num_threads_, true, params.plotting_mode_);
  localization_->setCameraTransforms(params.cam_tf_left_, params.cam_tf_right_);
  localization_->setWorkspace(params.workspace_);
  localization_->setNumSamples(params.num_samples_);
  localization_->setFingerWidth(params.finger_width_);
  localization_->setHandOuterDiameter(params.hand_outer_diameter_);
  localization_->setHandDepth(params.hand_depth_);
  localization_->setInitBite(params.init_bite_);
  localization_->setHandHeight(params.hand_height_);
		  
  min_inliers_ = params.min_inliers_;
  
  if (params.plotting_mode_ == 0)
  {
		plots_handles_ = false;
	}		
	else
	{
		plots_handles_ = false;		
		if (params.plotting_mode_ == 2)
			localization_->createVisualsPub(node, params.marker_lifetime_, cloud_frame_);
	}
}


void GraspLocalizer::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (num_clouds_received_ == num_clouds_)
    return;
  
  // get point cloud from topic
  if (cloud_frame_.compare(msg->header.frame_id) != 0 
			&& cloud_frame_.compare("/" + msg->header.frame_id) != 0)
  {
    std::cout << "Input cloud frame " << msg->header.frame_id << " is not equal to parameter " << cloud_frame_ 
			<< std::endl;
    std::exit(EXIT_FAILURE);
  }
  
  if (num_clouds_received_ == 0)
    pcl::fromROSMsg(*msg, *cloud_left_);
  else if (num_clouds_received_ == 1)
    pcl::fromROSMsg(*msg, *cloud_right_);
  std::cout << "Received cloud # " << num_clouds_received_ << " with " << msg->height * msg->width << " points\n";
  num_clouds_received_++;
}


void GraspLocalizer::cloud_sized_callback(const agile_grasp::CloudSized& msg)
{
  // get point cloud from topic
  if (cloud_frame_.compare(msg.cloud.header.frame_id) != 0)
  {
    std::cout << "Input cloud frame " << msg.cloud.header.frame_id << " is not equal to parameter "
      << cloud_frame_ << std::endl;
    std::exit(EXIT_FAILURE);
  }
  
  pcl::fromROSMsg(msg.cloud, *cloud_left_);
  size_left_ = msg.size_left.data;
  std::cout << "Received cloud with size_left: " << size_left_ << std::endl;
  num_clouds_received_ = 1;
}


void GraspLocalizer::localizeGrasps()
{
  ros::Rate rate(1);
  std::vector<int> indices(0);
  
  while (ros::ok())
  {
    // wait for point clouds to arrive
    if (num_clouds_received_ == num_clouds_)
    {
      // localize grasps
      if (num_clouds_ > 1)
      {
        PointCloud::Ptr cloud(new PointCloud());
        *cloud = *cloud_left_ + *cloud_right_;
        hands_ = localization_->localizeHands(cloud, cloud_left_->size(), indices, false, false);
      }
      else
      {
        hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
			}
      
      antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);
      handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);
      
      // publish handles
      grasps_pub_.publish(createGraspsMsg(handles_));
      ros::Duration(1.0).sleep();
      
      // publish hands contained in handles
      grasps_pub_.publish(createGraspsMsgFromHands(handles_));
      ros::Duration(1.0).sleep();
      
      // reset
      num_clouds_received_ = 0;
    }
    
    ros::spinOnce();
    rate.sleep();
  }
}


agile_grasp::Grasps GraspLocalizer::createGraspsMsg(const std::vector<GraspHypothesis>& hands)
{
  agile_grasp::Grasps msg;
  
  for (int i = 0; i < hands.size(); i++)
	{
  	msg.grasps.push_back(createGraspMsg(hands[i]));
  }
  
  msg.header.stamp = ros::Time::now();  
  return msg;
}


agile_grasp::Grasp GraspLocalizer::createGraspMsg(const GraspHypothesis& hand)
{
  agile_grasp::Grasp msg;
  tf::vectorEigenToMsg(hand.getGraspBottom(), msg.center);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getGraspSurface(), msg.surface_center);
  msg.width.data = hand.getGraspWidth();
  return msg;
}


agile_grasp::Grasps GraspLocalizer::createGraspsMsgFromHands(const std::vector<Handle>& handles)
{
  agile_grasp::Grasps msg;  
  for (int i = 0; i < handles.size(); i++)
  {
    const std::vector<GraspHypothesis>& hands = handles[i].getHandList();
    const std::vector<int>& inliers = handles[i].getInliers();
    
    for (int j = 0; j < inliers.size(); j++)
    {
      msg.grasps.push_back(createGraspMsg(hands[inliers[j]]));
    }
  }
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " hands\n";
  return msg;
}


agile_grasp::Grasps GraspLocalizer::createGraspsMsg(const std::vector<Handle>& handles)
{
  agile_grasp::Grasps msg;  
  for (int i = 0; i < handles.size(); i++)
    msg.grasps.push_back(createGraspMsg(handles[i]));  
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " handles\n";
  return msg;
}


agile_grasp::Grasp GraspLocalizer::createGraspMsg(const Handle& handle)
{
  agile_grasp::Grasp msg;
  tf::vectorEigenToMsg(handle.getCenter(), msg.center);
  tf::vectorEigenToMsg(handle.getAxis(), msg.axis);
  tf::vectorEigenToMsg(handle.getApproach(), msg.approach);
  tf::vectorEigenToMsg(handle.getHandsCenter(), msg.surface_center);
  msg.width.data = handle.getWidth();
  return msg;
}
