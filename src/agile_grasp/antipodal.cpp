#include <agile_grasp/antipodal.h>

const int Antipodal::NO_GRASP = 0; // normals point not toward any finger
const int Antipodal::HALF_GRASP = 1; // normals point towards one finger
const int Antipodal::FULL_GRASP = 2; // normals point towards both fingers

Antipodal::Antipodal(const Eigen::Matrix3Xd& normals) : normals_(normals)
{

}

int Antipodal::evaluateGrasp(double thresh_half, double thresh_full)
{
  int num_thresh = 6;
  int grasp = 0;
  double cos_thresh = cos(thresh_half * M_PI / 180.0);
  int numl = 0;
  int numr = 0;
  Eigen::Vector3d l, r;
  l << -1, 0, 0;
  r << 1, 0, 0;
  bool is_half_grasp = false;
  bool is_full_grasp = false;

  // check whether this is a half grasp
  for (int i = 0; i < normals_.cols(); i++)
  {
    if (l.dot(normals_.col(i)) > cos_thresh)
    {
      numl++;
      if (numl > num_thresh)
      {
        is_half_grasp = true;
        break;
      }
    }

    if (r.dot(normals_.col(i)) > cos_thresh)
    {
      numr++;
      if (numr > num_thresh)
      {
        is_half_grasp = true;
        break;
      }
    }
  }

  // check whether this is a full grasp
  cos_thresh = cos(thresh_full * M_PI / 180.0);
  numl = 0;
  numr = 0;
//  std::cout << "cos_thresh: " << cos_thresh << std::endl;
  for (int i = 0; i < normals_.cols(); i++)
  {
//  	std::cout << "normals_.col(" << i << "): " << normals_.col(i) << ", ldot: " << l.dot(normals_.col(i)) << std::endl;
    if (l.dot(normals_.col(i)) > cos_thresh)
    {
      numl++;
//      std::cout << "numl: " << numl << std::endl;
      if (numl > num_thresh && numr > num_thresh)
      {
        is_full_grasp = true;
        break;
      }
    }

    if (r.dot(normals_.col(i)) > cos_thresh)
    {
      numr++;
//      std::cout << "numr: " << numr << std::endl;
      if (numl > num_thresh && numr > num_thresh)
      {
        is_full_grasp = true;
        break;
      }
    }
  }

  if (is_full_grasp)
    return FULL_GRASP;
  else if (is_half_grasp)
    return HALF_GRASP;

  return NO_GRASP;
}
