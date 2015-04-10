#include <agile_grasp/quadric.h>
#include <agile_grasp/rotating_hand.h>
#include <agile_grasp/finger_hand.h>
#include <agile_grasp/grasp_hypothesis.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  double taubin_radius = 0.03; // radius of curvature-estimation neighborhood
  double hand_radius = 0.08; // radius of hand configuration neighborhood

//  std::string file = "/home/andreas/data/mlaffordance/round21l_reg.pcd";
  std::string file = "/home/andreas/data/mlaffordance/training/rect31l_reg.pcd";
  PointCloud::Ptr cloud(new PointCloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read input PCD file\n");
    return (-1);
  }
  
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
    new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

//  int sample_index = 0;
  int sample_index = 731;
  if (cloud->isOrganized())
  {
    organized_neighbor->setInputCloud(cloud);
    organized_neighbor->radiusSearch(cloud->points[sample_index], taubin_radius, nn_indices, nn_dists);
  }
  else
  {
    tree->setInputCloud(cloud);
    tree->radiusSearch(cloud->points[sample_index], taubin_radius, nn_indices, nn_dists);
  }
 
  std::cout << "Found point neighborhood for sample " << sample_index << "\n";

  Eigen::Matrix4d T_base, T_sqrt;
  T_base << 0, 0.445417, 0.895323, 0.22, 1, 0, 0, -0.02, 0, 0.895323, -0.445417, 0.24, 0, 0, 0, 1;

  T_sqrt << 0.9366, -0.0162, 0.3500, -0.2863, 0.0151, 0.9999, 0.0058, 0.0058, -0.3501, -0.0002, 0.9367, 0.0554, 0, 0, 0, 1;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > T_cams;
  T_cams.push_back(T_base * T_sqrt.inverse());
  std::cout << T_cams[0] << std::endl;

  // fit quadric
  Eigen::Vector3d sample = cloud->points[sample_index].getVector3fMap().cast<double>();
  Quadric quadric(T_cams, cloud, sample, true);
  quadric.fitQuadric(nn_indices);

  Eigen::MatrixXi cam_source = Eigen::MatrixXi::Zero(1, cloud->points.size());
  quadric.findTaubinNormalAxis(nn_indices, cam_source);

  quadric.print();

  // fit hand
  tree->radiusSearch(cloud->points[sample_index], hand_radius, nn_indices, nn_dists);

  Eigen::VectorXi pts_cam_source = Eigen::VectorXi::Ones(1, cloud->size());

  Eigen::Matrix3Xd nn_normals(3, nn_indices.size());
  Eigen::VectorXi nn_cam_source(nn_indices.size());
  Eigen::Matrix3Xd centered_neighborhood(3, nn_indices.size());
  nn_normals.setZero();

  for (int j = 0; j < nn_indices.size(); j++)
  {
    nn_cam_source(j) = pts_cam_source(nn_indices[j]);
    centered_neighborhood.col(j) = cloud->points[nn_indices[j]].getVector3fMap().cast<double>() - sample;
  }

  double finger_width_ = 0.01;
  double hand_outer_diameter_ = 0.09;
  double hand_depth_ = 0.06;

  FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

  double hand_height_ = 0.02;
  double init_bite_ = 0.01;

  RotatingHand rotating_hand(T_cams[0].block(0, 3, 3, 1) - sample, 
    T_cams[0].block(0, 3, 3, 1) - sample, finger_hand, true, pts_cam_source(sample_index));
  rotating_hand.transformPoints(centered_neighborhood, quadric.getNormal(), quadric.getCurvatureAxis(), nn_normals, nn_cam_source,
                                hand_height_);
  std::vector<GraspHypothesis> h = rotating_hand.evaluateHand(init_bite_, sample, 1);
  for (int i = 0; i < h.size(); i++)
  {
  	std::cout << "-- orientation " << i << " --\n";
		h[i].print();
	}

  // std::cout << "\n";
  // rotating_hand.evaluateHand(init_bite_, sample, 1);
  // rotating_hand.print();

  return 0;
}
