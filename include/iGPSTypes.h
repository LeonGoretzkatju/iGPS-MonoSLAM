/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2021 Ze Yang
*/


#ifndef IGPSTYPES_H
#define IGPSTYPES_H

#include<vector>
#include<utility>
#include<opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

namespace IGPS
{

class Point
{
public:
    Point(const float &p_x, const float &p_y, const float &p_z,
             const double &timestamp): p(p_x,p_y,p_z), t(timestamp){}
    Point(const cv::Point3f position, const double &timestamp):
                                       p(position),    t(timestamp){}
public:
    cv::Point3f p;
    double t;
};

}

} //namespace ORB_SLAM3

#endif // IGPSTYPES_H
