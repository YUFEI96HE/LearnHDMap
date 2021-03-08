#include "image_preprocessor.h"


ImagePreprocessor::ImagePreprocessor(/* args */)
{
}

ImagePreprocessor::~ImagePreprocessor()
{
}

int ImagePreprocessor::UndistortImage(cv::Mat &src_img, cv::Mat &dest_img)
{
    cv::Mat intrinsic = cv::Mat(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_64F);

    intrinsic.at<double>(0, 0) = 1399.56;
    intrinsic.at<double>(0, 1) = 0.0;
    intrinsic.at<double>(0, 2) = 1049.69;
    intrinsic.at<double>(1, 0) = 0.0;
    intrinsic.at<double>(1, 1) = 1399.56;
    intrinsic.at<double>(1, 2) = 540.251;
    intrinsic.at<double>(2, 0) = 0.0;
    intrinsic.at<double>(2, 1) = 0.0;
    intrinsic.at<double>(2, 2) = 1.0;

    distCoeffs.at<double>(0, 0) = -0.178554;
    distCoeffs.at<double>(1, 0) = 0.0307435;
    distCoeffs.at<double>(2, 0) = 0.0;
    distCoeffs.at<double>(3, 0) = 0.0;
    distCoeffs.at<double>(4, 0) = 0.0;

    cv::undistort(src_img, dest_img, intrinsic, distCoeffs);

    return OK;
}
