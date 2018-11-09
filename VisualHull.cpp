//
// Created by kino on 18/10/12.
//

#include <QVTKWin32Header.h>
#include "VisualHull.h"

const double ORI_X = -80.0;
const double ORI_Y = -20.0;
const double ORI_Z = 0.0;
const double BOX_X = 120.0;
const double BOX_Y = 160.0;
const double BOX_Z = -190.0;
const int DIVIDE = 150;

const int USED_CAMERANUM = 4;


/**
 * Visual Hull点群の生成
 * @param voxel ボクセルの集合
 * @return visuHull Constructed visual hull
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr VisualHull::visualizeVH(bool *voxel)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr visualHull(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < (int)pow(DIVIDE,3 ); i++) {
        if (voxel[i]) {
            pcl::PointXYZ point;
            point.z = (i / (int) pow(DIVIDE, 2)) * (BOX_Z / DIVIDE) + ORI_Z;
            point.y = ((i % (int) pow(DIVIDE, 2)) / DIVIDE) * (BOX_Y / DIVIDE) + ORI_Y;
            point.x = (i % DIVIDE) * (BOX_X / DIVIDE) + ORI_X;
            visualHull->points.push_back(point);
        }
    }
    visualHull->width = visualHull->points.size();
    visualHull->height = 1;
    return visualHull;
}

/**
 * マスク画像を見て，ボクセル空間中の各ボクセルについてが領域内に入っていなければ削る
 * @param img 画像
 * @param cameraMat カメラパラメータ
 * @param disCo 歪みパラメータ
 * @param rvec 回転ベクトル
 * @param tvec 移動ベクトル
 * @param voxel ボクセルの集合
 */
void VisualHull::cutVoxel(const cv::Mat &img, const cv::Mat &cameraMat, const cv::Mat &disCo,
              const cv::Mat &rvec, const cv::Mat &tvec, bool *voxel)
{
    for (int i = 0; i < (int)pow(DIVIDE, 3); i++) {
        if (voxel[i] == 0) {
            continue;
        } else {
            double z = (i / (int)pow(DIVIDE, 2)) * (BOX_Z / DIVIDE) + ORI_Z;
            double y =
                    ((i % (int)pow(DIVIDE, 2)) / DIVIDE) * (BOX_Y / DIVIDE) +
                    ORI_Y;
            double x = (i % DIVIDE) * (BOX_X / DIVIDE) + ORI_X;

            cv::Mat it = (cv::Mat_<double>(1, 3) << x, y, z);
            cv::Mat projected;
            cv::projectPoints(it, rvec, tvec, cameraMat, disCo, projected);
            cv::Point pp(projected.at<double>(0, 0),
                         projected.at<double>(0, 1));

            if (pp.x < 0 || pp.y < 0 || pp.x >= img.cols || pp.y >= img.rows) {
                voxel[i] = false;
            } else if (img.at<unsigned char>(pp.y, pp.x) == 0) {
                voxel[i] = false;
            }
        }
    }
    //	cv::imshow("test", test);
    //	cv::waitKey(0);
    //	std::cout << "pvmat" << pvmat << std::endl;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr VisualHull::visuHull(cv::Mat cameraMat[], cv::Mat disCo[], cv::Mat rvec[],
                                             cv::Mat tvec[], int USED_CAMERA[], std::string SUBNAME, std::string category, int frame)
{
    std::string folder =
            "/home/kino/Documents/Kaichisan0517/segmentation-synclo/" + SUBNAME + "/";
    std::string syntax;
    if (SUBNAME == "segmentation")
        syntax = ".jpg";
    else if (SUBNAME == "densepose")
        syntax = "_IUV.png";

    bool *voxel = new bool[(int)pow(DIVIDE, 3)];
    for (int i = 0; i < (int)pow(DIVIDE, 3); i++)
        voxel[i] = true;

    for (int i = 0; i < USED_CAMERANUM; i++) {
        int idx = USED_CAMERA[i];
        std::string filename =
                folder + category + "_cam" + std::to_string(i) + "/im_" + std::to_string(frame) + syntax;
        cv::Mat src = cv::imread(filename, 0);
        cutVoxel(src, cameraMat[idx], disCo[idx], rvec[idx], tvec[idx], voxel);

        // cv::Mat ds;
        // cv::resize(src, ds, cv::Size(), 0.5, 0.5);
        // cv::imshow("visuhull", ds);
        // cv::waitKey(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp = visualizeVH(voxel);
    delete[] voxel;
    return temp;
}

cv::Mat VisualHull::csvread(std::string filename) {
    std::ifstream inputfile(filename);
    std::string current_line;
// vector allows you to add data without knowing the exact size beforehand
    std::vector< std::vector<double> > all_data;
// Start reading lines as long as there are lines in the file
    while(getline(inputfile, current_line)){
        // Now inside each line we need to seperate the cols
        std::vector<double> values;
        std::stringstream temp(current_line);
        std::string single_value;
        while(getline(temp,single_value,',')){
            // convert the string element to a integer value
            values.push_back(std::stod(single_value));
        }
        // add the row to the complete data vector
        all_data.push_back(values);
    }

// Now add all the data into a Mat element
    cv::Mat vect = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64FC1);
// Loop over vectors and add the data
    for(int rows = 0; rows < (int)all_data.size(); rows++){
        for(int cols= 0; cols< (int)all_data[0].size(); cols++){
            vect.at<double>(rows,cols) = all_data[rows][cols];
        }
    }
    return vect;
}