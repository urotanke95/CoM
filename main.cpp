#include "VisualHull.h"
#include "Seg.h"

const int USED_CAMERANUM = 4;

int main() {
    cv::Mat cameraMat[4];
    cv::Mat rvec[4];
    cv::Mat tvec[4];
    cv::Mat disCo[4];

    VisualHull vh;
    Seg seg;

    //segmentation or densepose
    std::string subname = "densepose";

    //Either of fi, fo, ki, ko
    std::string category = "fi";

    //which frame?
    int frame = 3;

    //Define the filename of point cloud to save
    std::string filename = "/home/kino/Documents/Kaichisan0517/result_" + subname + "_" + category + "_" + std::to_string(frame) + ".pcd";

    int usedCam[4];

    //Read csv of camera parameters
    for (int j = 0; j < USED_CAMERANUM; j++) {
        cameraMat[j] = vh.csvread("/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/cameraMat.csv");
        rvec[j] = vh.csvread("/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/rvec.csv");
        tvec[j] = vh.csvread("/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/tvec.csv");
        disCo[j] = vh.csvread("/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/disCo.csv");
        usedCam[j] = j;
    }

    double CoM[3], onlyVH[3], onlyOP[3], beforeWeight[3];

//----------------- OpenPose -------------------------
    std::cout << "OpenPose" << std::endl;
    std::vector<pcl::PointXYZ> OPPvec =
            seg.obtainKeyPoints(cameraMat, rvec, tvec, usedCam, category, USED_CAMERANUM, frame);
    std::cout << "3d pos" << std::endl;
    //for (int i = 0; i < OPPvec.size(); i++) std::cout << OPPvec.at(i) << std::endl;
//----------------- Weighting -------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr vhPoint = vh.visuHull(cameraMat, disCo, rvec, tvec, usedCam, subname, category, frame);
    std::cout << "Weighting" << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = seg.weighting(vhPoint, OPPvec, CoM, onlyVH, onlyOP, beforeWeight, subname, category, frame);

    //Generate visual hull and save it as point cloud
    pcl::io::savePCDFileASCII(filename, *vhPoint);
    std::cout << CoM[0] << ", " << CoM[1] << ", " << CoM[2] << std::endl;

    return 0;
}