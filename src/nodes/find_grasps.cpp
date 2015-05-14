#include <ros/ros.h>

#include <agile_grasp/Grasp.h>
#include <agile_grasp/grasp_localizer.h>


const std::string CLOUD_TOPIC = "input_cloud";
const std::string CLOUD_FRAME = "camera_rgb_optical_frame";
const std::string SVM_FILE_NAME = "/home/baxter/svm";
const int NUM_THREADS = 1;
const int NUM_SAMPLES = 1000;
const int NUM_CLOUDS = 2;
const double FINGER_WIDTH = 0.01;
const double HAND_OUTER_DIAMETER = 0.09;
const double HAND_DEPTH = 0.06;
const double INIT_BITE = 0.01;
const double HAND_HEIGHT = 0.02;
const double WORKSPACE[6] = {0.65, 0.9, -0.2, 0.07, -0.3, 1.0};
const int MIN_HANDLE_INLIERS = 3;
const int CLOUD_TYPE = 0;
const std::string CLOUD_TYPES[2] = {"sensor_msgs/PointCloud2", "grasp_affordances/CloudSized"};
const std::string PLOT_MODES[3] = {"none", "pcl", "rviz"};


int main(int argc, char** argv)
{
  // initialize ROS
  ros::init(argc, argv, "find_grasps");
  ros::NodeHandle node("~");
  
  GraspLocalizer::Parameters params;
  
  // camera transforms (poses)
  Eigen::Matrix4d base_tf, sqrt_tf;
  base_tf <<  0, 0.445417, 0.895323, 0.215, 
              1, 0, 0, -0.015, 
              0, 0.895323, -0.445417, 0.23, 
              0, 0, 0, 1;
  sqrt_tf <<  0.9366, -0.0162, 0.3500, -0.2863, 
              0.0151, 0.9999, 0.0058, 0.0058, 
              -0.3501, -0.0002, 0.9367, 0.0554, 
              0, 0, 0, 1;
  params.cam_tf_left_ = base_tf * sqrt_tf.inverse();
  params.cam_tf_right_ = base_tf * sqrt_tf;

  // read ROS parameters
  std::string cloud_topic;
  std::string cloud_frame;
  std::string svm_file_name;
  std::vector<double> workspace;
  std::vector<double> camera_pose;
  int cloud_type;
  node.param("cloud_topic", cloud_topic, CLOUD_TOPIC);
  node.param("cloud_frame", cloud_frame, CLOUD_FRAME);
  node.param("cloud_type", cloud_type, CLOUD_TYPE);
  node.param("svm_file_name", svm_file_name, SVM_FILE_NAME);
  node.param("num_threads", params.num_threads_, NUM_THREADS);
  node.param("num_samples", params.num_samples_, NUM_SAMPLES); 
  node.param("num_clouds", params.num_clouds_, NUM_CLOUDS);
  node.param("finger_width", params.finger_width_, FINGER_WIDTH);
  node.param("hand_outer_diameter", params.hand_outer_diameter_, HAND_OUTER_DIAMETER);
  node.param("hand_depth", params.hand_depth_, HAND_DEPTH);
  node.param("init_bite", params.init_bite_, INIT_BITE);
  node.param("hand_height", params.hand_height_, HAND_HEIGHT);
  node.param("min_inliers", params.min_inliers_, MIN_HANDLE_INLIERS);
  node.getParam("workspace", workspace);
  node.getParam("camera_pose", camera_pose);
  node.param("plotting", params.plotting_mode_, 0);
  node.param("marker_lifetime", params.marker_lifetime_, 0.0);
  
  Eigen::Matrix4d R;
  for (int i=0; i < R.rows(); i++)
    R.row(i) << camera_pose[i*R.cols()], camera_pose[i*R.cols() + 1], camera_pose[i*R.cols() + 2], camera_pose[i*R.cols() + 3];  
    
  Eigen::VectorXd ws(6);
  ws << workspace[0], workspace[1], workspace[2], workspace[3], workspace[4], workspace[5];
  params.workspace_ = ws;
  
  std::cout << "-- Parameters --\n";
  std::cout << " Input\n";
  std::cout << "  cloud_topic: " << cloud_topic << "\n";
  std::cout << "  cloud_frame: " << cloud_frame << "\n";
  std::cout << "  cloud_type: " << CLOUD_TYPES[cloud_type] << "\n";
  std::cout << " Hand Search\n";  
  std::cout << "  workspace: " << ws.transpose() << "\n";
  std::cout << "  num_samples: " << params.num_samples_ << "\n";
  std::cout << "  num_threads: " << params.num_threads_ << "\n";
  std::cout << "  num_clouds: " << params.num_clouds_ << "\n";  
  std::cout << "  camera pose:\n" << R << std::endl;
  std::cout << " Robot Hand Model\n";
  std::cout << "  finger_width: " << params.finger_width_ << "\n";
  std::cout << "  hand_outer_diameter: " << params.hand_outer_diameter_ << "\n";
  std::cout << "  hand_depth: " << params.finger_width_ << "\n";
  std::cout << "  init_bite: " << params.finger_width_ << "\n";
  std::cout << "  hand_height: " << params.finger_width_ << "\n";
  std::cout << " Antipodal Grasps Prediction\n";
  std::cout << "  svm_file_name: " << svm_file_name << "\n";
  std::cout << " Handle Search\n";
  std::cout << "  min_inliers: " << params.min_inliers_ << "\n";
  std::cout << " Visualization\n";
  std::cout << "  plot_mode: " << PLOT_MODES[params.plotting_mode_] << "\n";
  std::cout << "  marker_lifetime: " << params.marker_lifetime_ << "\n";
  
  GraspLocalizer loc(node, cloud_topic, cloud_frame, cloud_type, svm_file_name, params);
  loc.localizeGrasps();
  
	return 0;
}
