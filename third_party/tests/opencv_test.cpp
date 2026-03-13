#include <opencv2/core.hpp>
#include <iostream>
#include <cassert>

int main() {
    cv::Mat A = cv::Mat::eye(3, 3, CV_32F);
    std::cout << "OpenCV Matrix:\n" << A << std::endl;
    
    assert(A.rows == 3);
    assert(A.cols == 3);
    assert(A.at<float>(0,0) == 1.0f);
    
    return 0;
}
