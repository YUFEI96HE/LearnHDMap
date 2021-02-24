/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-02-02 14:56
 * 
 */

#ifndef IMAEG_PREPROCESSOR_H
#define IMAEG_PREPROCESSOR_H

#include <stdio.h>
#include <iostream>
#include "fstream"
#include "string"
#include <vector>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


class ImagePreprocessor
{
private:
    /* data */
    const int OK = 0;
    const int FAILED = -1;

public:

    int height;
    int width;
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;


    int UndistortImage(cv::Mat &src_img, cv::Mat &dest_img);

    ImagePreprocessor(/* args */);
    ~ImagePreprocessor();
};



#endif