#include <agile_grasp/grasp_hypothesis.h>

void GraspHypothesis::print()
{
	std::cout << "axis: " << axis_.transpose() << std::endl;
	std::cout << "approach: " << approach_.transpose() << std::endl;
	std::cout << "binormal: " << binormal_.transpose() << std::endl;
	std::cout << "grasp width: " << grasp_width_ << std::endl;
	std::cout << "grasp surface: " << grasp_surface_.transpose() << std::endl;
	std::cout << "grasp bottom: " << grasp_bottom_.transpose() << std::endl;
}
