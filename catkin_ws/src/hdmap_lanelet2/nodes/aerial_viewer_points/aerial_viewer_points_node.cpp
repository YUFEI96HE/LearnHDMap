/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-02-22 10:53
 * 
 */

#include "aerial_viewer_points.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aerial_viewer_points");

    AerialViewerPoints points_viewer;

    ros::spin();

    return 0;
}
