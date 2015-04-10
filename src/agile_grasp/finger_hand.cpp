#include <agile_grasp/finger_hand.h>

FingerHand::FingerHand(double finger_width, double hand_outer_diameter,
		double hand_depth) :
		finger_width_(finger_width), hand_outer_diameter_(hand_outer_diameter), hand_depth_(
				hand_depth)
{
	int n = 10; // number of finger placements to consider over a single hand diameter

	Eigen::VectorXd fs_half;
	fs_half.setLinSpaced(n, 0.0, hand_outer_diameter - finger_width);
	finger_spacing_.resize(2 * fs_half.size());
	finger_spacing_
			<< (fs_half.array() - hand_outer_diameter_ + finger_width_).matrix(), fs_half;
	fingers_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, 2 * n, false);
//  std::cout << "fs_half: " << fs_half.transpose() << std::endl;
//  std::cout << "finger spacing: " << finger_spacing_.transpose() << std::endl;
}

void FingerHand::evaluateFingers(double bite)
{
	back_of_hand_ = -1.0 * (hand_depth_ - bite);
//  std::cout << "back_of_hand_: " << back_of_hand_ << std::endl;

	fingers_.setConstant(false);

	// crop points at bite
	std::vector<int> cropped_indices;
	for (int i = 0; i < points_.cols(); i++)
	{
		if (points_(1, i) < bite)
		{
			cropped_indices.push_back(i);

			// Check that the hand would be able to extend by <bite> onto the object without causing back of hand
			// to collide with points.
			if (points_(1, i) < back_of_hand_)
			{
				return;
			}
		}
	}
//  std::cout << "     -A- " << " " << cropped_indices.size() << " " << points_.cols() << "\n";
	Eigen::MatrixXd cropped_points(points_.rows(), cropped_indices.size());
	for (int i = 0; i < cropped_indices.size(); i++)
	{
		cropped_points.col(i) = points_.col(cropped_indices[i]);
	}
//  std::cout << "     -C-\n";

	// identify free gaps
	int m = finger_spacing_.size();
//  std::cout << "m: " << m << ", finger_spacing_: " << finger_spacing_.transpose() << " " << finger_spacing_.size() << std::endl;
	for (int i = 0; i < m; i++)
	{
//    std::cout << "  i: " << i << std::endl;
		// int num_in_gap = (cropped_points.array() > finger_spacing_(i)
		// && cropped_points.array() < finger_spacing_(i) + finger_width_).count();
		int num_in_gap = 0;
		for (int j = 0; j < cropped_indices.size(); j++)
		{
			if (cropped_points(0, j) > finger_spacing_(i)
					&& cropped_points(0, j) < finger_spacing_(i) + finger_width_)
				num_in_gap++;
		}
//    std::cout << "i: " << i << ", numInGap: " << num_in_gap << " finger_width_: " << finger_width_ << " finger_spacing_: " << finger_spacing_(i) << std::endl;

		if (num_in_gap == 0)
		{
			int sum;

			if (i <= m / 2)
			{
//        std::cout << " i <= m/2 " << finger_spacing_(i) << "\n";
//        Eigen::Array<bool, 1, Eigen::Dynamic>
				sum = (cropped_points.row(0).array()
						> finger_spacing_(i) + finger_width_).count();
			}
			else
			{
//        std::cout << " i > m/2\n";
				sum = (cropped_points.row(0).array() < finger_spacing_(i)).count();
			}

//      std::cout << "                 i: " << i<< " sum: " << sum << " cropped_points.array(): " << cropped_points.array().rows() << " x " << cropped_points.array().cols() << std::endl;

			if (sum > 0)
			{
//        std::cout << "      sum larger 0\n";
				fingers_(i) = true;
//        std::cout << fingers_ << "\n";
			}
		}
//    std::cout << " -----\n";
	}
//  std::cout << "     -E-\n";
//  std::cout << " fingers_: " << fingers_ << std::endl;
}

void FingerHand::evaluateHand()
{
//  std::cout << "evaluateHand()\n";
	int n = fingers_.size() / 2;
//  std::cout << " n: " << n;
	// hand_ = fingers_.block(0, 0, 1, n) && fingers_.block(0, n, 1, n);
	hand_.resize(1, n);
	for (int i = 0; i < n; i++)
	{
		if (fingers_(i) == 1 && fingers_(n + i) == 1)
			hand_(i) = 1;
		else
			hand_(i) = 0;
	}
//  std::cout << " hand_: " << hand_ << std::endl;
}

void FingerHand::evaluateGraspParameters(double bite)
{
//  double hor_pos = (hand_outer_diameter_ / 2.0)
//      + (finger_spacing_.block(0, 0, 1, hand_.cols()).array() * hand_.cast<double>()).sum() / hand_.sum();
//  std::cout << "hand_: " << hand_ << ", hand_.sum(): " << hand_.sum() << std::endl;
//  std::cout << "finger_spacing_: " << finger_spacing_.block(0, 0, 1, hand_.cols()).array() << std::endl;
//  std::cout << "hand_.cast<double>(): " << hand_.cast<double>() << std::endl;
//  std::cout << finger_spacing_.block(0, 0, 1, hand_.cols()).array() * hand_.cast<double>() << std::endl;
//  std::cout << (finger_spacing_.block(0, 0, 1, hand_.cols()).array() * hand_.cast<double>()).sum() << std::endl;

	double fs_sum = 0.0;
	for (int i = 0; i < hand_.size(); i++)
	{
		fs_sum += finger_spacing_(i) * hand_(i);
	}
	double hor_pos = (hand_outer_diameter_ / 2.0) + (fs_sum / hand_.sum());
//	std::cout << "fs_sum: " << fs_sum << ", hor_pos: " << hor_pos << ", hand_: "
//			<< hand_ << "\n";
	grasp_bottom << hor_pos, points_.row(1).maxCoeff();
	grasp_surface << hor_pos, points_.row(1).minCoeff();
//  std::cout << "  grasp_surface:\n" << grasp_surface << std::endl;
//  std::cout << "  grasp_bottom:\n" << grasp_bottom << std::endl;
//  std::cout << " EGP(1)\n";
//  std::cout << " hand_: " << hand_ << ", " << hand_.sum() << std::endl;

	// calculate handWidth. First find eroded hand. Then take points contained within two fingers of that hand.
	std::vector<int> hand_idx;
	for (int i = 0; i < hand_.cols(); i++)
	{
		if (hand_(i) == true)
			hand_idx.push_back(i);
	}
//  std::cout << hand_idx.size() << std::endl;
	int hand_eroded_idx = hand_idx[hand_idx.size() / 2];
//  std::cout << "hand_eroded_idx: " << hand_eroded_idx << std::endl;
	double left_finger_pos = finger_spacing_(hand_eroded_idx);
	double right_finger_pos = finger_spacing_(hand_.cols() + hand_eroded_idx);
	double max = -100000.0;
	double min = 100000.0;
//  std::cout << " EGP(2) left_finger_pos: " << left_finger_pos << " right_finger_pos: " << right_finger_pos << std::endl;
	for (int i = 0; i < points_.cols(); i++)
	{
		if (points_(1, i) < bite && points_(0, i) > left_finger_pos
				&& points_(0, i) < right_finger_pos)
		{
			if (points_(0, i) < min)
				min = points_(0, i);
			if (points_(0, i) > max)
				max = points_(0, i);
		}
	}
//  std::cout << " EGP(3) max: " << max << " min: " << min << std::endl;

	grasp_width_ = max - min;
}

void FingerHand::deepenHand(double init_deepness, double max_deepness)
{
//  std::cout << "deepenHand begin: " << hand_ << std::endl;
//	std::cout << "deepenHand begin. hand_: " << hand_ << ", fingers_: "
//			<< fingers_ << std::endl;
	std::vector<int> hand_idx;

	for (int i = 0; i < hand_.cols(); i++)
	{
		if (hand_(i) == true)
			hand_idx.push_back(i);
	}

	if (hand_idx.size() == 0)
		return;

	// choose middle hand
	int hand_eroded_idx = hand_idx[ceil(hand_idx.size() / 2.0) - 1]; // middle index
	Eigen::Array<bool, 1, Eigen::Dynamic> hand_eroded = Eigen::Array<bool, 1,	
		Eigen::Dynamic>::Constant(hand_.cols(), false);
	hand_eroded(hand_eroded_idx) = true;
//	std::cout << "hand_eroded_idx: " << hand_eroded_idx << ", hand_eroded: "
//			<< hand_eroded << ", hand_idx.size() / 2: " << hand_idx.size() / 2
//			<< std::endl;

	// attempt to deepen hand
	double deepen_step_size = 0.005;
	FingerHand new_hand = *this;
	FingerHand last_new_hand = new_hand;
//  std::cout << "new_hand.hand_: " << new_hand.getHand() << std::endl;
//  std::cout << "d = " << init_deepness + deepen_step_size << ", max_deepness: " << max_deepness << std::endl;
	for (double d = init_deepness + deepen_step_size; d <= max_deepness; d +=	deepen_step_size)
	{
//    std::cout << "d(): " << d << std::endl;
		new_hand.evaluateFingers(d);
		new_hand.evaluateHand();
//		std::cout << "d: " << d << ", new_hand.hand_: " << new_hand.getHand()
//				<< std::endl;
//		std::cout << "hand_eroded: " << hand_eroded << std::endl;
//    std::cout << " => last_new_hand.hand_: " << last_new_hand.getHand() << std::endl;
//     if ((new_hand.hand_ && hand_eroded).count() == 0) // hand is so deep that it does not exist any longer
//       break;
		bool is_too_deep = true;
		for (int i = 0; i < hand_eroded.cols(); i++)
		{
			if (new_hand.hand_(i) == 1 && hand_eroded(i) == 1) // hand is not so deep that it does not exist any longer
			{
				is_too_deep = false;
				break;
			}
		}
		if (is_too_deep)
			break;
		last_new_hand = new_hand;
//    std::cout << "d[]: " << d << ", last_new_hand.hand: " << last_new_hand.getHand() << ", new_hand.hand: " << new_hand.getHand() << std::endl;
	}
	*this = last_new_hand; // recover most deep hand
	hand_ = hand_eroded;
//	std::cout << "deepenHand done. hand_: " << hand_ << ", fingers_: " << fingers_
//			<< std::endl;
}
