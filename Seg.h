//
// Created by kino on 18/10/12.
//

#ifndef COM_SEG_H
#define COM_SEG_H
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include "json.hpp"
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>


class Seg {
    public:
        std::vector<pcl::PointXYZ> obtainKeyPoints(cv::Mat cameraMat[], cv::Mat rvec[],
                                                   cv::Mat tvec[], int USED_CAMERA[],
                                                   std::string category, int USED_CAMERANUM, int frame);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> weighting(pcl::PointCloud<pcl::PointXYZ>::Ptr VHpoint, std::vector<pcl::PointXYZ> OPP,
                                                                   double* CoM, double* onlyVH, double* onlyOP, double* beforeWeight,
                                                                   std::string subname, std::string category, int frame);
};


#endif //COM_SEG_H
