#include <agile_grasp/learning.h>
#include <agile_grasp/localization.h>

int main(int argc, char** argv)
{
  if (argc > 2)
  {
    // read PCD filename from command line
    std::string pcd_file_name = argv[1];
    std::string file_name_left;
    std::string file_name_right;
        
    if (pcd_file_name.find(".pcd") == std::string::npos)
    {
			file_name_left = pcd_file_name + "l_reg.pcd";
			file_name_right = pcd_file_name + "r_reg.pcd";
		}
		else
		{
			file_name_left = pcd_file_name;
			file_name_right = "";
		}

    // read SVM filename from command line
    std::string svm_file_name = argv[2];

		/* read number of samples, number of threads and min handle inliers
		 * from command line */
    int num_samples = 400;
    if (argc > 3)
      num_samples = atoi(argv[3]);

    int num_threads = 1;
    if (argc > 4)
      num_threads = atoi(argv[4]);
    
    int min_inliers = 3;
    if (argc > 5)
      min_inliers = atoi(argv[5]);

    double taubin_radius = 0.03;
    double hand_radius = 0.08;
	
		// camera poses for 2-camera Baxter setup
    Eigen::Matrix4d base_tf, sqrt_tf;

    base_tf << 0, 0.445417, 0.895323, 0.215, 
               1, 0, 0, -0.015, 
               0, 0.895323, -0.445417, 0.23, 
               0, 0, 0, 1;

    sqrt_tf <<   0.9366,  -0.0162,  0.3500, -0.2863, 
                 0.0151,   0.9999,   0.0058,   0.0058, 
                -0.3501, -0.0002, 0.9367, 0.0554, 
                 0,        0,      0,      1;

    // workspace dimensions
    Eigen::VectorXd workspace(6);
    //workspace << 0.4, 0.7, -0.02, 0.06, -0.2, 10;
    //workspace << 0.4, 1.0, -0.3, 0.3, -0.2, 2;
    //workspace << 0.55, 0.9, -0.35, 0.2, -0.2, 2;
    //workspace << 0.6, 0.8, -0.25, 0.1, -0.3, 2;
    // workspace << 0.55, 0.95, -0.25, 0.07, -0.3, 1;
    workspace << -10, 10, -10, 10, -10, 10;
    // workspace << -10, 10, -10, 10, 0.55, 0.95;

		// set-up parameters for the hand search
    Localization loc(num_threads, true, true);
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

    // test with fixed set of indices
//    std::vector<int> indices(5);
//    indices[0] = 731;
//    indices[1] = 4507;
//    indices[2] = 4445;
//    indices[3] = 2254;
//    indices[4] = 3716;
//    std::vector<RotatingHand> hand_list = loc.localizeHands(file_name_left, file_name_right, indices, svm_file_name);

//    // test with randomly sampled indices
    std::vector<GraspHypothesis> hands = loc.localizeHands(file_name_left, file_name_right);
    std::vector<GraspHypothesis> antipodal_hands = loc.predictAntipodalHands(hands, svm_file_name);
    std::vector<Handle> handles = loc.findHandles(antipodal_hands, min_inliers, 0.005);

    return 0;
  }

  std::cout << "No PCD filename given!\n";
  std::cout << "Usage: test_svm pcd_filename svm_filename [num_samples] [num_threads] [min_handle_inliers]\n";
  std::cout << "Localize grasps in a *.pcd file using a trained SVM.\n";
  
  return (-1);
}

