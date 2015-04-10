#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  cv::Mat image(80, 100, CV_8UC1);
  image.setTo(0);
  int r = 26;
  int c = 78;
  image.at<uchar>(r,c) = 255;
  image.at<uchar>(r+2,c) = 255;
  cv::namedWindow("Display window", cv::WINDOW_NORMAL); // Create a window for display.
  cv::imshow("Display window", image); // Show our image inside it.
  cv::waitKey(0); // Wait for a keystroke in the window
  return 0;
}
