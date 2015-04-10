#include <agile_grasp/learning.h>
#include <agile_grasp/localization.h>

int main(int argc, char** argv)
{
  if (argc > 1)
  {
    std::string file_name = argv[1];
    std::string file_name_left = file_name + "l_reg.pcd";
    std::string file_name_right = file_name + "r_reg.pcd";

    int num_samples = 400;
    if (argc > 2)
      num_samples = atoi(argv[2]);

    int num_threads = 1;
    if (argc > 3)
      num_threads = atoi(argv[3]);

    double taubin_radius = 0.03;
    double hand_radius = 0.08;

    Eigen::Matrix4d base_tf, sqrt_tf;

    base_tf << 0, 0.445417, 0.895323, 0.21, 1, 0, 0, -0.02, 0, 0.895323, -0.445417, 0.24, 0, 0, 0, 1;

    sqrt_tf << 0.9366, -0.0162, 0.3500, -0.2863, 0.0151, 0.9999, 0.0058, 0.0058, -0.3501, -0.0002, 0.9367, 0.0554, 0, 0, 0, 1;

    Eigen::VectorXd workspace(6);
    workspace << 0.4, 0.7, -0.02, 0.06, -0.2, 10;

    Localization loc(num_threads, false, true);
    loc.setCameraTransforms(base_tf * sqrt_tf.inverse(), base_tf * sqrt_tf);
    loc.setWorkspace(workspace);
    loc.setNumSamples(num_samples);
    loc.setNeighborhoodRadiusTaubin(taubin_radius);
    loc.setNeighborhoodRadiusHands(hand_radius);
    loc.setFingerWidth(0.01);
    loc.setHandOuterDiameter(0.09);
    loc.setHandDepth(0.06);
    loc.setInitBite(0.015);
    loc.setHandHeight(0.02);
    std::cout << "Localizing hands ...\n";

    // test with fixed set of indices
    std::vector<int> indices(5);
    indices[0] = 731;
    indices[1] = 4507;
    indices[2] = 4445;
    indices[3] = 2254;
    indices[4] = 3716;
//    std::vector<RotatingHand> hand_list = loc.localizeHands(file_name_left, file_name_right, indices, "", true);
//    Learning learn;
//    learn.train(hand_list);
//    loc.localizeHands(file_name_left, file_name_right, indices, "svm");

    // test with randomly sampled indices
    std::vector<GraspHypothesis> hand_list = loc.localizeHands(file_name_left, file_name_right, true);
    Learning learn;
    Eigen::Matrix<double,3,2> cam_pos;
    cam_pos.col(0) = loc.getCameraTransform(true).block<3,1>(0,3);
    cam_pos.col(1) = loc.getCameraTransform(false).block<3,1>(0,3);
    learn.train(hand_list, "svm", cam_pos);
    hand_list = loc.localizeHands(file_name_left, file_name_right, indices);
    loc.predictAntipodalHands(hand_list, "svm");

    return 0;
  }

  std::cout << "No PCD filename given!\n";
  std::cout << "Usage: learning_test filename [num_samples]\n";
  return (-1);
}

