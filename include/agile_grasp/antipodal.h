#ifndef ANTIPODAL_H_
#define ANTIPODAL_H_

#include <Eigen/Dense>
#include <iostream>

class Antipodal
{
public:

  Antipodal(const Eigen::Matrix3Xd& normals);

  int evaluateGrasp(double thresh_half, double thresh_full);

  static const int NO_GRASP; // normals point not toward any finger
  static const int HALF_GRASP; // normals point towards one finger
  static const int FULL_GRASP; // normals point towards both fingers

private:

  Eigen::Matrix3Xd normals_;
};

#endif /* ANTIPODAL_H_ */
