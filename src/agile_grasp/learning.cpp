#include <agile_grasp/learning.h>

void Learning::trainBalanced(const std::vector<GraspHypothesis>& hands_list, const std::vector<int> & sizes,
	const std::string& file_name, const Eigen::Matrix3Xd& cam_pos, int max_positive, bool is_plotting)
{
	std::vector<int> positives;
	std::vector<int> negatives;
  std::vector<int> indices_selected;
  std::vector<int> positives_sub;
	int k = 0;
  
  for (int i = 0; i < hands_list.size(); i++)
	{
    if (hands_list[i].isFullAntipodal())
      positives_sub.push_back(i);
    else if (!hands_list[i].isHalfAntipodal())
      negatives.push_back(i);
    
    if (i == sizes[k])
    {
      if (positives_sub.size() <= max_positive)
			{
				positives.insert(positives.end(), positives_sub.begin(), positives_sub.end());
			}
			else // select <max_positive> positive examples randomly
			{
				std::set<int> indices;
				while (indices.size() < max_positive)
					indices.insert(indices.end(), std::rand() % positives_sub.size());
        
        std::cout << positives.size() << " positive examples found\n";
        std::cout << " randomly selected indices:";

				for (std::set<int>::iterator it = indices.begin(); it != indices.end(); it++)
				{
          std::cout << " " << *it;
					positives.push_back(positives_sub[*it]);
				}
        
        std::cout << std::endl;
			}
      
      positives_sub.resize(0);
      k++;
    }
  }
  
  indices_selected.insert(indices_selected.end(), positives.begin(), positives.end());
  std::set<int> indices;
  while (indices.size() < positives.size())
    indices.insert(indices.end(), std::rand() % negatives.size());

  for (std::set<int>::iterator it = indices.begin(); it != indices.end(); it++)
  {
    indices_selected.push_back(negatives[*it]);
  }
  
  std::cout << "size(positives): " << positives.size() << std::endl;
  std::cout << "indices_selected.size: " << indices_selected.size() << std::endl;

  std::vector<Instance> instances;
	for (int i = 0; i < indices_selected.size(); i++)
	{
    int idx = indices_selected[i];
    instances.push_back(createInstance(hands_list[idx], cam_pos));
      
    // add instances for simulated left and right camera
    instances.push_back(createInstance(hands_list[idx], cam_pos, 0));
    instances.push_back(createInstance(hands_list[idx], cam_pos, 1));
  }

	std::cout << "Converting " << instances.size() << " training examples (grasps) to images\n";
	convertData(instances, file_name, is_plotting);
}

void Learning::train(const std::vector<GraspHypothesis>& hands_list, const std::vector<int> & sizes,
	const std::string& file_name, const Eigen::Matrix3Xd& cam_pos, int max_positive, bool is_plotting)
{  
  std::vector<int> positives;
  std::vector<Instance> instances;
	int k = 0;

	for (int i = 0; i < hands_list.size(); i++)
	{
		if (hands_list[i].isFullAntipodal())
		{
			positives.push_back(i);
		}
		else if (!hands_list[i].isHalfAntipodal())
		{
			instances.push_back(createInstance(hands_list[i], cam_pos));
      
      // add instances for simulated left and right camera
      instances.push_back(createInstance(hands_list[i], cam_pos, 0));
      instances.push_back(createInstance(hands_list[i], cam_pos, 1));
		}

		if (i == sizes[k])
		{
			// std::cout << " i: " << i << " " << sizes[k] << std::endl;
			if (positives.size() <= max_positive)
			{
				for (int j = 0; j < positives.size(); j++)
        {
          instances.push_back(createInstance(hands_list[positives[j]], cam_pos));
      
          // add instances for simulated left and right camera
          instances.push_back(createInstance(hands_list[positives[j]], cam_pos, 0));
          instances.push_back(createInstance(hands_list[positives[j]], cam_pos, 1));
        }
			}
			else // select <max_positive> positive examples randomly
			{
				std::set<int> indices;
				while (indices.size() < max_positive)
					indices.insert(indices.end(), std::rand() % positives.size());
        
        std::cout << positives.size() << " positive examples found\n";
        std::cout << " randomly selected indices:";

				for (std::set<int>::iterator it = indices.begin(); it != indices.end(); it++)
				{
					std::cout << " " << *it;
					instances.push_back(createInstance(hands_list[positives[*it]], cam_pos));
      
          // add instances for simulated left and right camera
          instances.push_back(createInstance(hands_list[positives[*it]], cam_pos, 0));
          instances.push_back(createInstance(hands_list[positives[*it]], cam_pos, 1));
				}
        
        std::cout << std::endl;
			}
			
      positives.resize(0);
			k++;
		}
	}

	std::cout << "Converting " << instances.size() << " training examples (grasps) to images\n";
	convertData(instances, file_name, is_plotting);
}

void Learning::train(const std::vector<GraspHypothesis>& hands_list, const std::string& file_name,
	const Eigen::Matrix3Xd& cam_pos, bool is_plotting)
{
	std::vector<Instance> instances;

	for (int i = 0; i < hands_list.size(); i++)
	{
		// do not use grasp if it's half-antipodal
		if (!hands_list[i].isHalfAntipodal() || hands_list[i].isFullAntipodal())
		{
      instances.push_back(createInstance(hands_list[i], cam_pos));
      
      // add instances for simulated left and right camera
      instances.push_back(createInstance(hands_list[i], cam_pos, 0));
      instances.push_back(createInstance(hands_list[i], cam_pos, 1));
		}
	}

	std::cout << "Converting " << instances.size() << " training examples (grasps) to images\n";
	convertData(instances, file_name, is_plotting);
}

std::vector<GraspHypothesis> Learning::classify(const std::vector<GraspHypothesis>& hands_list,
	const std::string& svm_filename, const Eigen::Matrix3Xd& cam_pos, bool is_plotting)
{
	std::cout << "Predicting ...\n";
  std::vector<GraspHypothesis> antipodal_hands(0);
	
	// check if SVM file exists
	ifstream f(svm_filename.c_str());
	if (!f.good()) 
	{
		f.close();
		std::cout << " Error: File " << svm_filename << " does not exist!\n";
		return antipodal_hands;
	}
		
	// load the SVM model from the file
	CvSVM svm;
	double t0 = omp_get_wtime();
	try
	{
		svm.load(svm_filename.c_str());
	}
	catch (cv::Exception& e)
	{
		std::cout << " Exception: " << e.msg << "\n";
		return antipodal_hands;
	}
	std::cout << " time for loading SVM: " << omp_get_wtime() - t0 << "\n";
  std::cout << " # of support vectors: " << svm.get_support_vector_count() << "\n";
	cv::HOGDescriptor hog;
	hog.winSize = cv::Size(64, 64);
	std::vector<bool> is_antipodal(hands_list.size());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
	for (int i = 0; i < hands_list.size(); i++)
	{
		const Eigen::Vector3d& source = cam_pos.col(hands_list[i].getCamSource());
		Eigen::Vector3d source_to_center = hands_list[i].getGraspSurface() - source;

		// convert grasp to image
		cv::Mat image = convertToImage(createInstance(hands_list[i], cam_pos));

		if (is_plotting)
		{
			cv::namedWindow("Grasp Image", cv::WINDOW_NORMAL); // Create a window for display.
			cv::imshow("Grasp Image", image); // Show our image inside it.
			cv::waitKey(0); // Wait for a keystroke in the window
		}

		// extract HOG features from image
		//    hog.cellSize = cv::Size(8,8);
		std::vector<float> descriptors;
		std::vector<cv::Point> locations;
		hog.compute(image, descriptors, cv::Size(32, 32), cv::Size(0, 0), locations);

		cv::Mat features(1, hog.getDescriptorSize() * 2, CV_32FC1);
		for (int k = 0; k < descriptors.size(); k++)
			features.at<float>(k) = descriptors[k];
		float prediction = svm.predict(features);
		if (prediction == 1)
		{
			GraspHypothesis grasp = hands_list[i];
			grasp.setFullAntipodal(true);
      is_antipodal[i] = true;
		}
    else
      is_antipodal[i] = false;
	}

  for (int i = 0; i < is_antipodal.size(); i++)
  {
    if (is_antipodal[i])
    {
      antipodal_hands.push_back(hands_list[i]);
      antipodal_hands[antipodal_hands.size()-1].setFullAntipodal(true);
    }
  }

  std::cout << " " << antipodal_hands.size() << " antipodal grasps found.\n";
	return antipodal_hands;
}

void Learning::convertData(const std::vector<Instance>& instances, 
  const std::string& file_name, bool is_plotting, 
  bool uses_linear_kernel)
{
	std::vector<cv::Mat> image_list;
	cv::HOGDescriptor hog;
	hog.winSize = cv::Size(64, 64);
	cv::Mat features(instances.size(), hog.getDescriptorSize() * 2, CV_32FC1);
	cv::Mat labels(instances.size(), 1, CV_32FC1);
	int num_positives = 0;

	for (int i = 0; i < instances.size(); i++)
	{
		// convert grasp to image
//		std::cout << "i: " << i << "\n";
		cv::Mat image = convertToImage(instances[i]);
//		std::cout << " Converted grasp data to image\n";
		image_list.push_back(image);

		// visualize grasp image
		if (is_plotting)
		{
			cv::namedWindow("Grasp Image", cv::WINDOW_NORMAL); // Create a window for display.
			cv::imshow("Grasp Image", image); // Show our image inside it.
			cv::waitKey(0); // Wait for a keystroke in the window
		}

		// extract HOG features from image
//    hog.cellSize = cv::Size(8,8);
		std::vector<float> descriptors;
		std::vector<cv::Point> locations;
//		std::cout << "HOG descriptor size is " << hog.getDescriptorSize() << std::endl;
		hog.compute(image, descriptors, cv::Size(32, 32), cv::Size(0, 0), locations);
//		std::cout << "HOG descriptor size is " << hog.getDescriptorSize() << std::endl;
//		std::cout << "# descriptors = " << descriptors.size() << std::endl;
		for (int j = 0; j < features.cols; ++j)
		{
			features.at<float>(i, j) = descriptors[j];
		}
		if (instances[i].label == 1)
		{
			labels.at<float>(i, 0) = 1.0;
			num_positives++;
		}
		else
			labels.at<float>(i, 0) = -1.0;
	}

  // train the SVM
	CvSVMParams params;
  // cv::Mat weights(1, 2, CV_32FC1);
  // weights.at<float>(0,0) = 0.9;
  // weights.at<float>(0,1) = 0.1;
  // CvMat cv1_weights = weights;  
  // params.class_weights = &cv1_weights;
	params.svm_type = CvSVM::C_SVC;
  if (uses_linear_kernel)
    params.kernel_type = CvSVM::LINEAR;
  else
  {
    params.kernel_type = CvSVM::POLY;
    params.degree = 2;
	}
  CvSVM svm;
	svm.train(features, labels, cv::Mat(), cv::Mat(), params);
	svm.save(file_name.c_str());
	std::cout << "# training examples: " << features.rows << " (# positives: " << num_positives
			<< ", # negatives: " << features.rows - num_positives << ")\n";
	std::cout << "Saved trained SVM as " << file_name << "\n";
}

cv::Mat Learning::convertToImage(const Instance& ins)
{
	const double HORIZONTAL_LIMITS[2] = { -0.05, 0.05 };
	const double VERTICAL_LIMITS[2] = { 0.0, 0.08 };
	double cell_size = (HORIZONTAL_LIMITS[1] - HORIZONTAL_LIMITS[0]) / (double) num_horizontal_cells_;

	Eigen::VectorXi horizontal_cells(ins.pts.cols());
	Eigen::VectorXi vertical_cells(ins.pts.cols());

  // reverse x-direction to keep orientation consistent
	if (ins.binormal.dot(ins.source_to_center) > 0)
		horizontal_cells = floorVector((ins.pts.row(0).array() - HORIZONTAL_LIMITS[0]) / cell_size);
	else
		horizontal_cells = floorVector((-ins.pts.row(0).array() - HORIZONTAL_LIMITS[0]) / cell_size);

	vertical_cells = floorVector((ins.pts.row(1).array() - VERTICAL_LIMITS[0]) / cell_size);

	std::set<Eigen::Vector2i, UniqueVectorComparator> cells;
	for (int i = 0; i < ins.pts.cols(); i++)
	{
		Eigen::Vector2i c;
		c << horizontal_cells(i), vertical_cells(i);
		cells.insert(c);
	}

	Eigen::Matrix2Xi cells_mat(2, cells.size());
	int i = 0;
	cv::Mat image(num_vertical_cells_, num_horizontal_cells_, CV_8UC1);
	image.setTo(0);

	for (std::set<Eigen::Vector2i, UniqueVectorComparator>::iterator it = cells.begin(); it != cells.end();
			it++)
	{
		Eigen::Vector2i c = *it;
		cells_mat(0, i) = std::max(0, c(0));
		cells_mat(1, i) = std::max(0, c(1));

		c = cells_mat.col(i);
		cells_mat(0, i) = std::min(num_horizontal_cells_ - 1, c(0));
		cells_mat(1, i) = std::min(num_vertical_cells_ - 1, c(1));
		image.at<uchar>(image.rows - 1 - cells_mat(1, i), cells_mat(0, i)) = 255;
		i++;
	}

	return image;
}

Eigen::VectorXi Learning::floorVector(const Eigen::VectorXd& a)
{
	Eigen::VectorXi b(a.size());
	for (int i = 0; i < b.size(); i++)
		b(i) = floor(a(i));
	return b;
}

Learning::Instance Learning::createInstance(const GraspHypothesis& h, const Eigen::Matrix3Xd& cam_pos, int cam)
{
  Instance ins;
  ins.binormal = h.getBinormal();
  ins.label = h.isFullAntipodal();
  
  // calculate camera position to center vector
  const Eigen::Vector3d& source = cam_pos.col(h.getCamSource());
  ins.source_to_center = h.getGraspSurface() - source;
  
  if (cam == -1)
  {
    ins.pts = h.getPointsForLearning();
  }
  else
  {
    const std::vector<int>& indices_cam = (cam == 0) ? h.getIndicesPointsForLearningCam1() : h.getIndicesPointsForLearningCam2();
    ins.pts.resize(3, indices_cam.size());
    for (int i = 0; i < indices_cam.size(); i++)
    {
      ins.pts.col(i) = h.getPointsForLearning().col(indices_cam[i]);
    } 
  }

  return ins;
}
