#include <agile_grasp/quadric.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct VectorComparator
{
  bool operator ()(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
  {
  	return a(0) != b(0) || a(1) != b(1) || a(2) != b(2);
  }
};

int main(int argc, char** argv)
{
  double taubin_radius = 0.03; // radius of curvature-estimation neighborhood

  std::string file = "/home/andreas/data/mlaffordance/round21l_reg.pcd";
  PointCloud::Ptr cloud(new PointCloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read input PCD file\n");
    return (-1);
  }

  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr organized_neighbor(
      new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  std::vector<int> nn_outer_indices;
  std::vector<float> nn_outer_dists;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

  int sample_index = 0;
  if (cloud->isOrganized())
  {
    organized_neighbor->setInputCloud(cloud);
    organized_neighbor->radiusSearch(cloud->points[sample_index], taubin_radius, nn_outer_indices, nn_outer_dists);
  }
  else
  {
    tree->setInputCloud(cloud);
    tree->radiusSearch(cloud->points[sample_index], taubin_radius, nn_outer_indices, nn_outer_dists);
  }

  Eigen::Matrix4d T_base, T_sqrt;
  T_base << 0, 0.445417, 0.895323, 0.22, 1, 0, 0, -0.02, 0, 0.895323, -0.445417, 0.24, 0, 0, 0, 1;

  T_sqrt << 0.9366, -0.0162, 0.3500, -0.2863, 0.0151, 0.9999, 0.0058, 0.0058, -0.3501, -0.0002, 0.9367, 0.0554, 0, 0, 0, 1;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > T_cams;
  T_cams.push_back(T_base * T_sqrt.inverse());

  Eigen::Vector3d sample = cloud->points[sample_index].getVector3fMap().cast<double>();
  Quadric quadric(T_cams, cloud, sample, true);
  quadric.fitQuadric(nn_outer_indices);

  Eigen::MatrixXi cam_source = Eigen::MatrixXi::Zero(1, cloud->points.size());
  quadric.findTaubinNormalAxis(nn_outer_indices, cam_source);

  quadric.print();

  std::set<Eigen::Vector3i, VectorComparator> s;
  Eigen::Matrix3i M;
  M << 1,1,1,2,3,4,1,1,1;
  for (int i=0; i < M.rows(); i++)
    s.insert(M.row(i));
  std::cout << "M:" << std::endl;
  std::cout << M << std::endl;
  Eigen::Matrix<int, Eigen::Dynamic, 3> N(s.size(), 3);
  int i = 0;
  for (std::set<Eigen::Vector3i, VectorComparator>::iterator it=s.begin(); it!=s.end(); ++it)
  {
    N.row(i) = *it;
    i++;
  }
  std::cout << "N:" << std::endl;
  std::cout << N << std::endl;

  return 0;
}
