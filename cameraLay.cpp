//
// Created by kino on 18/10/22.
//

#include "cameraLay.h"

double CameraLay::outb(int n)
{
    return b.at<double>(n, 0);
}

cv::Mat CameraLay::outb()
{
    return b;
}

double CameraLay::outd(int n)
{
    return d.at<double>(n, 0);
}
cv::Mat CameraLay::outd()
{
    return d;
}

CameraLay::CameraLay(const cv::Mat inpb, const cv::Mat inpd)
{
    b = inpb;
    d = inpd;
}