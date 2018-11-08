//
// Created by kino on 18/10/12.
//

#include "Seg.h"
#include "VisualHull.h"
#include "cameraLay.h"

const int USED_CAMERANUM = 4;
//const int KEYPOINT_NUM = 18;
const int SEGMENTATION = 9;
const std::string POSTURE = "bat";

cv::Mat computeLayCenter(std::vector<CameraLay *> *lays)
{
    static int countOPcenter = 0;
    int size = lays->size();
    if (size < 2) {
        std::cout << "no joint: " << countOPcenter << std::endl;
        // exit(1);
        return (cv::Mat_<double>(3, 1) << -10000.0, 0.0, 0.0);
    }
    cv::Mat left = cv::Mat::zeros(size + 3, size + 3, CV_64F);
    cv::Mat right = cv::Mat::zeros(size + 3, 1, CV_64F);
    double b[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < size; i++) {
        cv::Mat temp0 = (*lays)[i]->outd().t() * (*lays)[i]->outd();
        left.at<double>(i, i) =
                temp0.at<double>(0, 0) - 1.0; //-1.0 is homogeneous coordinates
        left.at<double>(size, i) = -(*lays)[i]->outd(0);
        left.at<double>(i, size) = -(*lays)[i]->outd(0);
        left.at<double>(size + 1, i) = -(*lays)[i]->outd(1);
        left.at<double>(i, size + 1) = -(*lays)[i]->outd(1);
        left.at<double>(size + 2, i) = -(*lays)[i]->outd(2);
        left.at<double>(i, size + 2) = -(*lays)[i]->outd(2);
        cv::Mat temp1 = (*lays)[i]->outd().t() * (*lays)[i]->outb();
        right.at<double>(i, 0) = -(temp1.at<double>(0, 0) - 1.0);
        b[0] += (*lays)[i]->outb(0);
        b[1] += (*lays)[i]->outb(1);
        b[2] += (*lays)[i]->outb(2);
    }
    for (int i = 0; i < 3; i++) {
        left.at<double>(size + i, size + i) = (double)size;
        right.at<double>(size + i, 0) = b[i];
    }
    //	cv::Mat temp00 = (*lays)[0]->outd().t() * (*lays)[0]->outd();
    //	std::cout << "temp" << temp00 << std::endl;
    //   std::cout << (*lays)[0]->outd().t() << std::endl;
    //   std::cout << (*lays)[0]->outd() << std::endl;
    //    std::cout << "left" << left << std::endl;
    //    std::cout << "right" << right << std::endl;

    cv::Mat ans;
    cv::solve(left, right, ans);
    cv::Mat ret_ans =
            (cv::Mat_<double>(3, 1) << ans.at<double>(size, 0),
                    ans.at<double>(size + 1, 0), ans.at<double>(size + 2, 0));
    countOPcenter++;
    return ret_ans;
}

double distancePtoL(pcl::PointXYZ jointA, pcl::PointXYZ jointB,
                    pcl::PointXYZ pointX)
{
    double vx = jointB.x - jointA.x, vy = jointB.y - jointA.y,
            vz = jointB.z - jointA.z;
    double t = (vx * (pointX.x - jointA.x) + vy * (pointX.y - jointA.y) +
                vz * (pointX.z - jointA.z)) /
               (vx * vx + vy * vy + vz * vz);
    if (t < 0 || 1 < t)
        return 9999999;
    else {
        double disdis = std::pow((jointA.x + t * vx) - pointX.x, 2.0) +
                        std::pow((jointA.y + t * vy) - pointX.y, 2.0) +
                        std::pow((jointA.z + t * vz) - pointX.z, 2.0);
        return std::sqrt(disdis);
    }
}

double distancePtoLforThins(pcl::PointXYZ jointA, pcl::PointXYZ jointB,
                            pcl::PointXYZ pointX)
{
    double vx = jointB.x - jointA.x, vy = jointB.y - jointA.y,
            vz = jointB.z - jointA.z;
    double t = (vx * (pointX.x - jointA.x) + vy * (pointX.y - jointA.y) +
                vz * (pointX.z - jointA.z)) /
               (vx * vx + vy * vy + vz * vz);
    if (t < 0.2 || 1 < t)
        return 9999999;
    else {
        double disdis = std::pow((jointA.x + t * vx) - pointX.x, 2.0) +
                        std::pow((jointA.y + t * vy) - pointX.y, 2.0) +
                        std::pow((jointA.z + t * vz) - pointX.z, 2.0);
        return std::sqrt(disdis);
    }
}

double distancePtoLforHands(pcl::PointXYZ jointA, pcl::PointXYZ jointB,
                            pcl::PointXYZ pointX)
{
    double vx = jointB.x - jointA.x, vy = jointB.y - jointA.y,
            vz = jointB.z - jointA.z;
    double t = (vx * (pointX.x - jointA.x) + vy * (pointX.y - jointA.y) +
                vz * (pointX.z - jointA.z)) /
               (vx * vx + vy * vy + vz * vz);
    if (t < 1 || 2.0 < t)
        return 9999999;
    else {
        double disdis = std::pow((jointA.x + t * vx) - pointX.x, 2.0) +
                        std::pow((jointA.y + t * vy) - pointX.y, 2.0) +
                        std::pow((jointA.z + t * vz) - pointX.z, 2.0);
        return std::sqrt(disdis);
    }
}

double distancePtoLforBody(pcl::PointXYZ jointA, pcl::PointXYZ jointB,
                           pcl::PointXYZ pointX)
{
    double vx = jointB.x - jointA.x, vy = jointB.y - jointA.y,
            vz = jointB.z - jointA.z;
    double t = (vx * (pointX.x - jointA.x) + vy * (pointX.y - jointA.y) +
                vz * (pointX.z - jointA.z)) /
               (vx * vx + vy * vy + vz * vz);
    if (t < -0.4 || 1.4 < t)
        return 9999999;
    else {
        double disdis = std::pow((jointA.x + t * vx) - pointX.x, 2.0) +
                        std::pow((jointA.y + t * vy) - pointX.y, 2.0) +
                        std::pow((jointA.z + t * vz) - pointX.z, 2.0);
        return std::sqrt(disdis);
    }
}

double distancePtoP(pcl::PointXYZ A, pcl::PointXYZ B)
{
    double distan = std::pow(A.x - B.x, 2.0) + std::pow(A.y - B.y, 2.0) +
                    std::pow(A.z - B.z, 2.0);
    return std::sqrt(distan);
}

double distancePtoPforHead(pcl::PointXYZ A, pcl::PointXYZ B)
{
    double distan = std::pow(A.x - B.x, 2.0) + std::pow(A.y - B.y, 2.0) +
                    std::pow(A.z + 5.0 - B.z, 2.0);
    return std::sqrt(distan);
}

void middlePointforKineCoM(pcl::PointXYZ jointA, pcl::PointXYZ jointB,
                           pcl::PointXYZ* each)
{
    each->x = (jointA.x + jointB.x) / 2.0;
    each->y = (jointA.y + jointB.y) / 2.0;
    each->z = (jointA.z + jointB.z) / 2.0;
}

void computeKinematicCoM(std::vector<pcl::PointXYZ> OPP, std::string filename,
                         double* onlyOP)
{
    // 0-8, head,shoulder,hand,foot,backarm,frontarm,thigh,shin,body
    double kineCoM[3] = {0.0, 0.0, 0.0};
    pcl::PointXYZ eachKineCoM[SEGMENTATION];
    pcl::PointXYZ temp[2];
    middlePointforKineCoM(OPP[16], OPP[17], &eachKineCoM[0]);
    middlePointforKineCoM(OPP[2], OPP[5], &eachKineCoM[1]);
    middlePointforKineCoM(OPP[4], OPP[7], &eachKineCoM[2]);
    middlePointforKineCoM(OPP[10], OPP[13], &eachKineCoM[3]);

    middlePointforKineCoM(OPP[2], OPP[3], &temp[0]);
    middlePointforKineCoM(OPP[5], OPP[6], &temp[1]);
    middlePointforKineCoM(temp[0], temp[1], &eachKineCoM[4]);

    middlePointforKineCoM(OPP[3], OPP[4], &temp[0]);
    middlePointforKineCoM(OPP[6], OPP[7], &temp[1]);
    middlePointforKineCoM(temp[0], temp[1], &eachKineCoM[5]);

    middlePointforKineCoM(OPP[8], OPP[9], &temp[0]);
    middlePointforKineCoM(OPP[11], OPP[12], &temp[1]);
    middlePointforKineCoM(temp[0], temp[1], &eachKineCoM[6]);

    middlePointforKineCoM(OPP[9], OPP[10], &temp[0]);
    middlePointforKineCoM(OPP[12], OPP[13], &temp[1]);
    middlePointforKineCoM(temp[0], temp[1], &eachKineCoM[7]);

    middlePointforKineCoM(OPP[8], OPP[11], &temp[0]);
    middlePointforKineCoM(temp[0], OPP[1], &eachKineCoM[8]);

    double percentage;
    for (int i = 0; i < SEGMENTATION; i++) {
        switch (i) {
            case 0:
                percentage = 0.0792;
                break;
            case 1:
                percentage = 0.1054;
                break;
            case 2:
                percentage = 0.01224;
                break;
            case 3:
                percentage = 0.02862;
                break;
            case 4:
                percentage = 0.0528;
                break;
            case 5:
                percentage = 0.03062;
                break;
            case 6:
                percentage = 0.20016;
                break;
            case 7:
                percentage = 0.09224;
                break;
            case 8:
                percentage = 0.381;
                break;
        }
        kineCoM[0] += eachKineCoM[i].x * percentage;
        kineCoM[1] += eachKineCoM[i].y * percentage;
        kineCoM[2] += eachKineCoM[i].z * percentage;
    }
    /**
    std::ofstream wfile;
    wfile.open(filename, std::ios::app);
    if (!wfile) {
        std::cout << "cannnot open csv" << std::endl;
        exit(1);
    } else {
         wfile << kineCoM[0] * 10.0 << "," << kineCoM[1] * 10.0 << "," <<
         255.0
              << "," << 0 << "," << 255.0 << std::endl;
        std::cout << "CoM write" << std::endl;
        wfile << kineCoM[0] << "," << kineCoM[1] << "," << kineCoM[2]
              << std::endl;
        for (int i = 0; i < 3; i++)
            onlyOP[i] = kineCoM[i] * 10.0;
    }
     */
}

void computeCoM(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud,
                double* CoM, double* beforeWeight, std::string filename)
{
    double all = 0.0;

    double noweightCoM[3];
    double eachCoM[SEGMENTATION][3];
    bool sizenot0[SEGMENTATION];
    for (int i = 0; i < SEGMENTATION; i++) {
        if (segmentCloud[i]->size() == 0)
            sizenot0[i] = false;
        else
            sizenot0[i] = true;

        double segsize = segmentCloud[i]->size();
        double segCoM[3] = {0.0, 0.0, 0.0};
        if (sizenot0[i]) {
            for (auto& point : *segmentCloud[i]) {
                segCoM[0] += point.x;
                segCoM[1] += point.y;
                segCoM[2] += point.z;
            }
            for (int j = 0; j < 3; j++) {
                eachCoM[i][j] = segCoM[j] / segsize;
            }
            all += segsize;
        }
    }
    double noweightCoM_cp[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < SEGMENTATION; i++) {
        if (sizenot0[i]) {
            for (auto& point : *segmentCloud[i]) {
                noweightCoM_cp[0] += point.x;
                noweightCoM_cp[1] += point.y;
                noweightCoM_cp[2] += point.z;
            }
        }
    }
    // std::cout << "koreda" << std::endl;
    // for (int j = 0; j < 3; j++) {
    //    noweightCoM_cp[j] /= all;
    //    std::cout << noweightCoM_cp[j] << std::endl;
    //}

    // weighting for each
    // 0-8, head,shoulder,hand,foot,backarm,frontarm,thigh,shin,body
    double percentage;
    for (int i = 0; i < SEGMENTATION; i++) {
        if (sizenot0[i]) {
            switch (i) {
                case 0:
                    percentage = 0.0792;
                    break;
                case 1:
                    percentage = 0.1054;
                    break;
                case 2:
                    percentage = 0.01224;
                    break;
                case 3:
                    percentage = 0.02862;
                    break;
                case 4:
                    percentage = 0.0528;
                    break;
                case 5:
                    percentage = 0.03062;
                    break;
                case 6:
                    percentage = 0.20016;
                    break;
                case 7:
                    percentage = 0.09224;
                    break;
                case 8:
                    percentage = 0.381;
                    break;
            }
            for (int j = 0; j < 3; j++) {
                noweightCoM[j] += eachCoM[i][j] * segmentCloud[i]->size() / all;
                CoM[j] += eachCoM[i][j] * percentage;
            }
        }
    }
    /**
    std::ofstream wfile;
    wfile.open(filename, std::ios::app);
    if (!wfile) {
        std::cout << "cannnot open csv" << std::endl;
        exit(1);
    } else {
    //    wfile << noweightCoM[0] * 10.0 << "," << noweightCoM[1] * 10.0 << ","
    //         << 0 << "," << 0 << "," << 255.0 << std::endl;
    //    wfile << CoM[0] * 10.0 << "," << CoM[1] * 10.0 << "," << 0 << ","
    //          << 255.0 << "," << 255.0 << std::endl;
        for (int i = 0; i < 3; i++)
            beforeWeight[i] = noweightCoM[i];
    }
    */
}

void split(std::string input, char delimiter, std::vector<std::string>* result)
{
    std::istringstream stream(input);
    std::string field;
    while (getline(stream, field, delimiter)) {
        result->push_back(field);
    }
}

std::vector<pcl::PointXYZ> Seg::obtainKeyPoints(cv::Mat cameraMat[], cv::Mat rvec[],
                                                cv::Mat tvec[], int USED_CAMERA[],
                                                std::string category, int USED_CAMERANUM, int frame) {
    std::vector<cv::Mat> openPoint;
    for (int i = 0; i < USED_CAMERANUM; i++) {
        cv::Mat tempOP(18, 2, CV_64FC1);
        std::string csv_file = "/home/kino/Documents/Kaichisan0517/segmentation-synclo/op/" + category + "/cam" + std::to_string(i) + "/manualcsv/" + std::to_string(frame) + ".csv";
        std::ifstream ifs(csv_file);
        std::string line;
        std::vector<std::string> strvec;
        while (std::getline(ifs, line)) {
            split(line, ',', &strvec);
        }

        for (int j = 0; j < 18; j++) {
            for (int k = 0; k < 2; k++) {
                tempOP.at<double>(j, k) = std::stod(strvec[k + 2 * j]);
            }
        }
        /** for reading json
        std::string json_file =
                "/home/kino/Documents/Kaichisan0517/segmentation-synclo/openpose_keypoint/" + category + "_cam" + std::to_string(i) + "/im_0_keypoints.json";
        std::ifstream people_file(json_file, std::ifstream::binary);
        nlohmann::json people;
        people_file >> people;
        nlohmann::json pos = people["people"][0]["pose_keypoints"];
        int counter = -1;
        for (int j = 0; j < 18; j++) {
            for (int k = 0; k < 2; k++) {
                counter++;
                if (counter % 3 == 2) continue;
                tempOP.at<double>(j, k) = pos[counter];
            }
        }
        */
        openPoint.push_back(tempOP);
    }

    cv::Mat cpose = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
    cv::Mat rmat[USED_CAMERANUM];
    std::vector<cv::Mat> vmat;
    cv::Mat invpose[USED_CAMERANUM];
    for (int i = 0; i < USED_CAMERANUM; i++) {
        int idx = USED_CAMERA[i];
        cv::Rodrigues(rvec[idx], rmat[idx]);
        cv::Mat tempvmat =
                (cv::Mat_<double>(4, 4) << rmat[idx].at<double>(0, 0),
                        rmat[idx].at<double>(0, 1), rmat[idx].at<double>(0, 2),
                        tvec[idx].at<double>(0, 0), rmat[idx].at<double>(1, 0),
                        rmat[idx].at<double>(1, 1), rmat[idx].at<double>(1, 2),
                        tvec[idx].at<double>(1, 0), rmat[idx].at<double>(2, 0),
                        rmat[idx].at<double>(2, 1), rmat[idx].at<double>(2, 2),
                        tvec[idx].at<double>(2, 0), 0.0, 0.0, 0.0, 1.0);
        vmat.push_back(tempvmat);
        invpose[i] = tempvmat.inv() * cpose;
    }

    std::vector<cv::Mat> answers;
    for (int i = 0; i < 18; i++) {
        std::vector<CameraLay *> lays;
        for (int j = 0; j < USED_CAMERANUM; j++) {
            int jdx = USED_CAMERA[j];
            if (openPoint[j].at<double>(i, 0) > 0.0 &&
                openPoint[j].at<double>(i, 1) > 0.0) {
                double f = cameraMat[jdx].at<double>(0, 0);
                double cx = cameraMat[jdx].at<double>(0, 2);
                double cy = cameraMat[jdx].at<double>(1, 2);
                cv::Mat cdire = (cv::Mat_<double>(4, 1)
                        << (openPoint[j].at<double>(i, 0) - cx),
                        (openPoint[j].at<double>(i, 1) - cy), f, 1.0);
                cv::Mat invdiretemp = vmat[j].inv() * cdire;
                cv::Mat invdire = invdiretemp - invpose[j];
                CameraLay *lay = new CameraLay(invpose[j], invdire);
                lays.push_back(lay);
            }
        }
        cv::Mat ans = computeLayCenter(&lays);
        answers.push_back(ans);
    }
    std::cout << "fin opPoint" << std::endl;

    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
    //      new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointXYZ> toretu;
    //    std::vector<pcl::PointXYZ> ansposi;
    for (auto &po : answers) {
        //std::cout << po << std::endl;
        pcl::PointXYZ point;
        point.x = po.at<double>(0, 0);
        point.y = po.at<double>(1, 0);
        point.z = po.at<double>(2, 0);
        toretu.push_back(point);
    }
    return toretu;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Seg::weighting(
        pcl::PointCloud<pcl::PointXYZ>::Ptr VHpoint, std::vector<pcl::PointXYZ> OPP,
        double* CoM, double* onlyVH, double* onlyOP, double* beforeWeight,
        std::string subname, std::string category, int frame) {
    //----head:0, shoulder:2,5, hand:4,7, foot:10,13, elbow:3,6, knee:9,12,
    // hip:8,11, body:1,8,11, 6parts and 10points-------
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud0(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud1(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud2(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud3(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud4(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud5(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud6(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud7(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud8(
            new pcl::PointCloud<pcl::PointXYZ>);

    double eachAllPoints[3] = {0.0, 0.0, 0.0};

    double VHsize = VHpoint->size();
    if (POSTURE == "sit") {
        // if (POSTURE == "s") {
        for (uint i = 0; i < VHsize; i++) {
            eachAllPoints[0] += VHpoint->at(i).x;
            eachAllPoints[1] += VHpoint->at(i).y;
            eachAllPoints[2] += VHpoint->at(i).z;
            // head
            if (distancePtoPforHead(VHpoint->at(i), OPP[16]) < 11.0 ||
                distancePtoPforHead(VHpoint->at(i), OPP[17]) < 11.0 ||
                distancePtoP(VHpoint->at(i), OPP[14]) < 12.0 ||
                distancePtoP(VHpoint->at(i), OPP[15]) < 12.0) {
                tempCloud0->push_back(VHpoint->at(i));
                continue;
            }
            // shoulder
            if (distancePtoP(VHpoint->at(i), OPP[2]) < 12.0 ||
                distancePtoP(VHpoint->at(i), OPP[5]) < 12.0) {
                tempCloud1->push_back(VHpoint->at(i));
                continue;
            }
            // back arm
            if (distancePtoL(OPP[2], OPP[3], VHpoint->at(i)) < 8.0 ||
                distancePtoL(OPP[5], OPP[6], VHpoint->at(i)) < 8.0) {
                tempCloud4->push_back(VHpoint->at(i));
                continue;
            }
            // front arm
            if (distancePtoL(OPP[3], OPP[4], VHpoint->at(i)) < 7.0 ||
                distancePtoL(OPP[6], OPP[7], VHpoint->at(i)) < 7.0) {
                tempCloud5->push_back(VHpoint->at(i));
                continue;
            }
            // shin
            if (distancePtoL(OPP[9], OPP[10], VHpoint->at(i)) < 7.0 ||
                distancePtoL(OPP[12], OPP[13], VHpoint->at(i)) < 7.0) {
                tempCloud7->push_back(VHpoint->at(i));
                continue;
            }
            // thigh
            if (distancePtoLforThins(OPP[8], OPP[9], VHpoint->at(i)) < 13.0 ||
                distancePtoLforThins(OPP[11], OPP[12], VHpoint->at(i)) < 13.0 ||
                distancePtoP(VHpoint->at(i), OPP[8]) < 13.0 ||
                distancePtoP(VHpoint->at(i), OPP[11]) < 13.0) {
                tempCloud6->push_back(VHpoint->at(i));
                continue;
            }
            // hand
            if (distancePtoLforHands(OPP[3], OPP[4], VHpoint->at(i)) < 20.0 ||
                distancePtoLforHands(OPP[6], OPP[7], VHpoint->at(i)) < 20.0) {
                tempCloud2->push_back(VHpoint->at(i));
                continue;
            }
            // if (distancePtoP(VHpoint->at(i), OPP[4]) < 10.0 ||
            //    distancePtoP(VHpoint->at(i), OPP[7]) < 10.0) {
            //    tempCloud2->push_back(VHpoint->at(i));
            //    continue;
            //}
            // foot
            if (distancePtoP(VHpoint->at(i), OPP[10]) < 16.0 ||
                distancePtoP(VHpoint->at(i), OPP[13]) < 16.0) {
                tempCloud3->push_back(VHpoint->at(i));
                continue;
            }
            // body
            if (distancePtoLforBody(OPP[1], OPP[8], VHpoint->at(i)) < 16.0 ||
                distancePtoLforBody(OPP[1], OPP[11], VHpoint->at(i)) < 16.0 ||
                distancePtoLforBody(OPP[8], OPP[11], VHpoint->at(i)) < 16.0) {
                tempCloud8->push_back(VHpoint->at(i));
                continue;
            }
        }
    } else {
        for (uint i = 0; i < VHsize; i++) {
            eachAllPoints[0] += VHpoint->at(i).x;
            eachAllPoints[1] += VHpoint->at(i).y;
            eachAllPoints[2] += VHpoint->at(i).z;
            // head
            if (distancePtoPforHead(VHpoint->at(i), OPP[16]) < 14.0 ||
                distancePtoPforHead(VHpoint->at(i), OPP[17]) < 14.0 ||
                distancePtoP(VHpoint->at(i), OPP[14]) < 14.0 ||
                distancePtoP(VHpoint->at(i), OPP[15]) < 14.0) {
                tempCloud0->push_back(VHpoint->at(i));
                continue;
            }
            // hand
            if (distancePtoLforHands(OPP[3], OPP[4], VHpoint->at(i)) < 6.0 ||
                distancePtoLforHands(OPP[6], OPP[7], VHpoint->at(i)) < 6.0) {
                tempCloud2->push_back(VHpoint->at(i));
                continue;
            }
            // shoulder
            if (distancePtoP(VHpoint->at(i), OPP[2]) < 11.0 ||
                distancePtoP(VHpoint->at(i), OPP[5]) < 11.0) {
                tempCloud1->push_back(VHpoint->at(i));
                continue;
            }
            // back arm
            if (distancePtoL(OPP[2], OPP[3], VHpoint->at(i)) < 7.0 ||
                distancePtoL(OPP[5], OPP[6], VHpoint->at(i)) < 7.0) {
                tempCloud4->push_back(VHpoint->at(i));
                continue;
            }
            // front arm
            if (distancePtoL(OPP[3], OPP[4], VHpoint->at(i)) < 6.0 ||
                distancePtoL(OPP[6], OPP[7], VHpoint->at(i)) < 6.0) {
                tempCloud5->push_back(VHpoint->at(i));
                continue;
            }
            // thigh
            if (distancePtoLforThins(OPP[8], OPP[9], VHpoint->at(i)) < 13.0 ||
                distancePtoLforThins(OPP[11], OPP[12], VHpoint->at(i)) < 13.0) {
                tempCloud6->push_back(VHpoint->at(i));
                continue;
            }
            // shin
            if (distancePtoL(OPP[9], OPP[10], VHpoint->at(i)) < 10.0 ||
                distancePtoL(OPP[12], OPP[13], VHpoint->at(i)) < 10.0) {
                tempCloud7->push_back(VHpoint->at(i));
                continue;
            }
            // hand
            // if (distancePtoLforHands(OPP[3], OPP[4], VHpoint->at(i))
            // < 6.0 ||
            //    distancePtoLforHands(OPP[6], OPP[7], VHpoint->at(i))
            //    < 6.0) { tempCloud2->push_back(VHpoint->at(i)); continue;
            //}
            // if (distancePtoP(VHpoint->at(i), OPP[4]) < 10.0 ||
            //    distancePtoP(VHpoint->at(i), OPP[7]) < 10.0) {
            //    tempCloud2->push_back(VHpoint->at(i));
            //    continue;
            //}
            // foot
            if (distancePtoP(VHpoint->at(i), OPP[10]) < 16.0 ||
                distancePtoP(VHpoint->at(i), OPP[13]) < 16.0) {
                tempCloud3->push_back(VHpoint->at(i));
                continue;
            }
            // body
            if (distancePtoLforBody(OPP[1], OPP[8], VHpoint->at(i)) < 16.0 ||
                distancePtoLforBody(OPP[1], OPP[11], VHpoint->at(i)) < 16.0 ||
                distancePtoLforBody(OPP[8], OPP[11], VHpoint->at(i)) < 16.0) {
                tempCloud8->push_back(VHpoint->at(i));
                continue;
            }
        }
    }
    segmentCloud.push_back(tempCloud0);
    segmentCloud.push_back(tempCloud1);

    segmentCloud.push_back(tempCloud2);
    segmentCloud.push_back(tempCloud3);
    segmentCloud.push_back(tempCloud4);
    segmentCloud.push_back(tempCloud5);
    segmentCloud.push_back(tempCloud6);
    segmentCloud.push_back(tempCloud7);
    segmentCloud.push_back(tempCloud8);
    std::string segpcd = "/home/kino/Documents/Kaichisan0517/segmentation-synclo/eval/" + category + "/" + subname + "_" + std::to_string(frame) + "_seg.pcd";
    pcl::PointCloud<pcl::PointXYZRGB> seg;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud0_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud1_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud2_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud3_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud4_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud5_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud6_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud7_;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud8_;
    pcl::copyPointCloud(*tempCloud0, tempCloud0_);
    pcl::copyPointCloud(*tempCloud1, tempCloud1_);
    pcl::copyPointCloud(*tempCloud2, tempCloud2_);
    pcl::copyPointCloud(*tempCloud3, tempCloud3_);
    pcl::copyPointCloud(*tempCloud4, tempCloud4_);
    pcl::copyPointCloud(*tempCloud5, tempCloud5_);
    pcl::copyPointCloud(*tempCloud6, tempCloud6_);
    pcl::copyPointCloud(*tempCloud7, tempCloud7_);
    pcl::copyPointCloud(*tempCloud8, tempCloud8_);

    for (int i = 0; i < tempCloud0_.size(); i++) {
        tempCloud0_[i].r = 255; tempCloud0_[i].b = 0; tempCloud0_[i].g = 0; tempCloud0_[i].a = 0;
    }
    for (int i = 0; i < tempCloud1_.size(); i++) {
        tempCloud1_[i].r = 0; tempCloud1_[i].b = 255; tempCloud1_[i].g = 0; tempCloud1_[i].a = 0;
    }
    for (int i = 0; i < tempCloud2_.size(); i++) {
        tempCloud2_[i].r = 0; tempCloud2_[i].b = 0; tempCloud2_[i].g = 255; tempCloud2_[i].a = 0;
    }
    for (int i = 0; i < tempCloud3_.size(); i++) {
        tempCloud3_[i].r = 255; tempCloud3_[i].b = 255; tempCloud3_[i].g = 0; tempCloud3_[i].a = 0;
    }
    for (int i = 0; i < tempCloud4_.size(); i++) {
        tempCloud4_[i].r = 255; tempCloud4_[i].b = 0; tempCloud4_[i].g = 255; tempCloud4_[i].a = 0;
    }
    for (int i = 0; i < tempCloud5_.size(); i++) {
        tempCloud5_[i].r = 0; tempCloud5_[i].b = 255; tempCloud5_[i].g = 255; tempCloud5_[i].a = 0;
    }
    for (int i = 0; i < tempCloud6_.size(); i++) {
        tempCloud6_[i].r = 0; tempCloud6_[i].b = 0; tempCloud6_[i].g = 0; tempCloud6_[i].a = 0;
    }
    for (int i = 0; i < tempCloud7_.size(); i++) {
        tempCloud7_[i].r = 255; tempCloud7_[i].b = 255; tempCloud7_[i].g = 255; tempCloud7_[i].a = 0;
    }
    for (int i = 0; i < tempCloud8_.size(); i++) {
        tempCloud8_[i].r = 50; tempCloud8_[i].b = 50; tempCloud8_[i].g = 50; tempCloud8_[i].a = 0;
    }

    seg = tempCloud0_;
    seg += tempCloud1_ + tempCloud2_;
    seg += tempCloud3_+ tempCloud4_;
    seg += tempCloud5_ + tempCloud6_;
    seg += tempCloud7_ + tempCloud8_;
    pcl::io::savePCDFileASCII(segpcd, seg);

    std::cout << "each part" << std::endl;
    for (int i = 0; i < SEGMENTATION; i++) {
        std::cout << i << ": " << segmentCloud[i]->size() << std::endl;
    }

    std::string filename = "/home/kino/Documents/Kaichisan0517/segmentation-synclo/eval/" + category + "/" + subname + "_" + std::to_string(frame) + "_gnu.csv";
    //std::ofstream wfile;
    //wfile.open(filename, std::ios::out);
    //wfile << eachAllPoints[0] * 10.0 / VHsize << ","
    //      << eachAllPoints[1] * 10.0 / VHsize << "," << 0 << "," << 255.0 << ","
    //      << 0 << std::endl;
    for (int i = 0; i < 3; i++)
        onlyVH[i] = eachAllPoints[i] * 10.0 / VHsize;

    double tempcom[3] = {0.0, 0.0, 0.0}, tempKine[3] = {0.0, 0.0, 0.0};
    computeKinematicCoM(OPP, filename, onlyOP);
    computeCoM(segmentCloud, tempcom, tempKine, filename);
    for (int i = 0; i < 3; i++) {
        CoM[i] = tempcom[i];// * 10.0;
        beforeWeight[i] = tempKine[i];// * 10.0;
    }

    return segmentCloud;
}