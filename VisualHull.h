//
// Created by kino on 18/10/12.
//

#ifndef COM_VISUALHULL_H
#define COM_VISUALHULL_H

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>


class VisualHull {
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr visualizeVH(bool *voxel);
        void cutVoxel(const cv::Mat &img, const cv::Mat &cameraMat, const cv::Mat &disCo,
                      const cv::Mat &rvec, const cv::Mat &tvec, bool *voxel);
        pcl::PointCloud<pcl::PointXYZ>::Ptr visuHull(cv::Mat cameraMat[], cv::Mat disCo[], cv::Mat rvec[],
                                                     cv::Mat tvec[], int USED_CAMERA[], std::string SUBNAME, std::string category, int frame);
        cv::Mat csvread(std::string filename);
};


#endif //COM_VISUALHULL_H
