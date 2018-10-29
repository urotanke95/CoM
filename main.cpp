#include "VisualHull.h"
#include "Seg.h"

const int USED_CAMERANUM = 4;

int main() {
    cv::Mat cameraMat[USED_CAMERANUM];
    cv::Mat rvec[USED_CAMERANUM];
    cv::Mat tvec[USED_CAMERANUM];
    cv::Mat disCo[USED_CAMERANUM];

    VisualHull vh;
    Seg seg;

    //segmentation or densepose
    std::vector<std::string> subname = {"segmentation", "densepose"};

    //Either of fi, fo, ki, ko
    std::vector<std::string> category = {"fi", "fo", "ki", "ko"};

    //How many frames in the category?
    int frame = 48;

    for (int k = 0; k < subname.size(); k++) {
        for (int l = 0; l < category.size(); l++) {

            std::cout << subname.at(k) << ", " << category.at(l) << std::endl;
            std::string csvfile =
                    "/home/kino/Documents/Kaichisan0517/segmentation-synclo/eval/" + category.at(l) + "/" + subname.at(k) + ".csv";
            std::ofstream wfile;
            wfile.open(csvfile, std::ios::app);
            if (!wfile) {
                std::cout << "cannnot open csv" << std::endl;
                exit(1);
            }

            for (int i = 0; i < frame; i++) {

                //Define the filename of point cloud to save
                std::string filename =
                        "/home/kino/Documents/Kaichisan0517/segmentation-synclo/eval/" + category.at(l) + "/" + subname.at(k) + "_" + std::to_string(i) + ".pcd";

                int usedCam[USED_CAMERANUM];

                //Read csv of camera parameters
                for (int j = 0; j < USED_CAMERANUM; j++) {
                    cameraMat[j] = vh.csvread(
                            "/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) +
                            "/cameraMat.csv");
                    rvec[j] = vh.csvread(
                            "/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/rvec.csv");
                    tvec[j] = vh.csvread(
                            "/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/tvec.csv");
                    disCo[j] = vh.csvread(
                            "/home/kino/Documents/Kaichisan0517/calibration/cam" + std::to_string(j) + "/disCo.csv");
                    usedCam[j] = j;
                }

                double CoM[3], onlyVH[3], onlyOP[3], beforeWeight[3];

                //----------------- OpenPose -------------------------
                std::cout << "OpenPose" << std::endl;
                std::vector<pcl::PointXYZ> OPPvec =
                        seg.obtainKeyPoints(cameraMat, rvec, tvec, usedCam, category.at(l), USED_CAMERANUM, i);
                std::cout << "3d pos" << std::endl;
                //for (int i = 0; i < OPPvec.size(); i++) std::cout << OPPvec.at(i) << std::endl;
                //----------------- Weighting -------------------------

                pcl::PointCloud<pcl::PointXYZ>::Ptr vhPoint = vh.visuHull(cameraMat, disCo, rvec, tvec, usedCam,
                                                                          subname.at(k),
                                                                          category.at(l), i);
                std::cout << "Weighting" << std::endl;
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = seg.weighting(vhPoint, OPPvec, CoM,
                                                                                              onlyVH,
                                                                                              onlyOP, beforeWeight,
                                                                                              subname.at(k),
                                                                                              category.at(l), i);

                //----------------- Generate VH ------------------------
                pcl::io::savePCDFileASCII(filename, *vhPoint);
                std::cout << CoM[0] << ", " << CoM[1] << ", " << CoM[2] << std::endl;
                wfile << CoM[0] << "," << CoM[1] << ", " << CoM[2] << std::endl;
            }

            wfile.close();

        }
    }
    return 0;
}