#include <agile_grasp/localization.h>

#include <Eigen/Dense>

#include <stdlib.h>

int main(int argc, char** argv)
{
  if (argc > 1)
  {
    std::string file_name = argv[1];
    std::string file_name_left = file_name + "l_reg.pcd";
    std::string file_name_right = file_name + "r_reg.pcd";

    int num_samples = 1000;
    if (argc > 2)
      num_samples = atoi(argv[2]);

    double taubin_radius = 0.03;
    double hand_radius = 0.08;

    Eigen::Matrix4d base_tf, sqrt_tf;

    base_tf << 0, 0.445417, 0.895323, 0.21, 1, 0, 0, -0.02, 0, 0.895323, -0.445417, 0.24, 0, 0, 0, 1;

    sqrt_tf << 0.9366, -0.0162, 0.3500, -0.2863, 0.0151, 0.9999, 0.0058, 0.0058, -0.3501, -0.0002, 0.9367, 0.0554, 0, 0, 0, 1;

    Eigen::VectorXd workspace(6);
    workspace << 0.4, 0.7, -0.02, 0.06, -0.2, 10;

    Localization loc;
    loc.setCameraTransforms(base_tf * sqrt_tf.inverse(), base_tf * sqrt_tf);
    loc.setWorkspace(workspace);
    loc.setNumSamples(num_samples);
    loc.setNeighborhoodRadiusTaubin(taubin_radius);
    loc.setNeighborhoodRadiusHands(hand_radius);
    loc.setFingerWidth(0.01);
    loc.setHandOuterDiameter(0.09);
    loc.setHandDepth(0.06);
    loc.setInitBite(0.01);
    loc.setHandHeight(0.02);
    std::cout << "Localizing hands ...\n";

//    std::vector<int> indices(1);
//    indices[0] = 731;
//    indices[0] = 4507;
//    indices[0] = 4445;
//    indices[0] = 2254;
//    indices[0] = 3716;
    std::vector<int> indices(5);
    indices[0] = 731;
    indices[1] = 4507;
    indices[2] = 4445;
    indices[3] = 2254;
    indices[4] = 3716;
    loc.localizeHands(file_name_left, file_name_right, indices, false);

//		std::vector<int> indices(0);
//		loc.localizeHands(file_name_left, file_name_right, indices, "",  false);

    std::cout << "back to main\n";
    return 0;
  }

  std::cout << "No PCD filename given!\n";
  std::cout << "Usage: test_local_axes filename [num_samples]\n";
  return (-1);
}
