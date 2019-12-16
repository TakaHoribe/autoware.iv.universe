/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NDT_SLAM_PCL_H
#define NDT_SLAM_PCL_H

#include "ndt/base.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
// #include "pcl_registration/ndt.h"

template <class PointSource, class PointTarget>
class NdtSlamPCL : public NdtSlamBase<PointSource, PointTarget>
{
  public:
    NdtSlamPCL();
    ~NdtSlamPCL() = default;

    void align(pcl::PointCloud<PointSource> &output, const Eigen::Matrix4f& guess) override;
    void setInputTarget(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) override;
    void setInputSource(const boost::shared_ptr<pcl::PointCloud<PointSource>> &scan_ptr) override;

    void setMaximumIterations(int max_iter) override;
    void setResolution(float res) override;
    void setStepSize(double step_size) override;
    void setTransformationEpsilon(double trans_eps) override;

    int getMaximumIterations() override;
    float getResolution() const override;
    double getStepSize() const override;
    double getTransformationEpsilon() override;
    double getTransformationProbability() const override;
    double getFitnessScore() override;

    Eigen::Matrix<double, 6, 6> getHessian() const override;

    boost::shared_ptr<pcl::search::KdTree<PointTarget>> getSearchMethodTarget() const override;

  private:
    boost::shared_ptr<pcl::NormalDistributionsTransform<PointSource, PointTarget>> ndt_ptr_;
};

#endif
