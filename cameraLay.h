//
// Created by kino on 18/10/22.
//

#ifndef COM_CAMERALAY_H
#define COM_CAMERALAY_H

#endif //COM_CAMERALAY_H

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

class CameraLay {
    public:
        double outb(int n);
        cv::Mat outb();
        double outd(int n);
        cv::Mat outd();
        CameraLay(const cv::Mat inpb, const cv::Mat inpd);
        cv::Mat b;
        cv::Mat d;
};