#ifndef REFERENCE_PATH_H
#define REFERENCE_PATH_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include "cubic_spline.hpp"

class ReferencePath
{
public:
    ReferencePath(const std::vector<double>& x, const std::vector<double>& y, const double ds) : spline_(x, y)
    {
        x_.reserve(spline_.s.size());
        y_.reserve(spline_.s.size());
        yaw_.reserve(spline_.s.size());

        for(float s=0; s<spline_.s.back(); s+=ds)
        {
            std::array<double, 2> point = spline_.calc_position(s);

            x_.push_back(point[0]);
            y_.push_back(point[1]);
            yaw_.push_back(spline_.calc_yaw(s));
        }
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    Spline2D spline_;
};

class ReferenceTrajectoryPath
{
public:
    ReferenceTrajectoryPath(const std::vector<double>& x, 
                            const std::vector<double>& y, 
                            const std::vector<double>& v,
                            const double ds) : spline_(x, y, v)
    {
        x_.reserve(spline_.s.size());
        y_.reserve(spline_.s.size());
        yaw_.reserve(spline_.s.size());

        for(float s=0; s<spline_.s.back(); s+=ds)
        {
            std::array<double, 3> point = spline_.calc_point(s);

            x_.push_back(point[0]);
            y_.push_back(point[1]);
            v_.push_back(point[2]);
            yaw_.push_back(spline_.calc_yaw(s));
        }
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> v_;
    std::vector<double> yaw_;
    Spline3D spline_;
};

class Reference3DTrajectoryPath
{
public:
    Reference3DTrajectoryPath(const std::vector<double>& x, 
                              const std::vector<double>& y, 
                              const std::vector<double>& z, 
                              const std::vector<double>& v,
                              const double ds) : spline_(x, y, z, v)
    {
        x_.reserve(spline_.s.size());
        y_.reserve(spline_.s.size());
        z_.reserve(spline_.s.size());
        v_.reserve(spline_.s.size());
        yaw_.reserve(spline_.s.size());

        for(float s=0; s<spline_.s.back(); s+=ds)
        {
            std::array<double, 4> point = spline_.calc_trajectory_point(s);

            x_.push_back(point[0]);
            y_.push_back(point[1]);
            z_.push_back(point[2]);
            v_.push_back(point[3]);
            yaw_.push_back(spline_.calc_yaw(s));
        }
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    std::vector<double> v_;
    std::vector<double> yaw_;
    Spline4D spline_;
};


#endif //REFERENCE_PATH_H
