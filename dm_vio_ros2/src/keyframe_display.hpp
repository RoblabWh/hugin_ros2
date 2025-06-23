/**
 * This file is part of DSO, written by Jakob Engel.
 * It has been modified by Lukas von Stumberg for the inclusion in DM-VIO (http://vision.in.tum.de/dm-vio).
 *
 * Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Eigen/Core>
#include "util/NumType.h"
#include "util/settings.h"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <sstream>
#include <fstream>

namespace dso
{
  class CalibHessian;
  class FrameHessian;
  class FrameShell;

  namespace IOWrap
  {

    template <int ppp>
    struct InputPointSparse
    {
      float u;
      float v;
      float idepth;
      float idepth_hessian;
      float relObsBaseline;
      int numGoodRes;
      unsigned char color[ppp];
      unsigned char status;
    };

    // stores a pointcloud associated to a Keyframe.
    class KeyFrameDisplay
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      KeyFrameDisplay(Settings *settings, const std::string &name, const std::string &frame_odom);
      ~KeyFrameDisplay();

      // copies points from KF over to internal buffer,
      // keeping some additional information so we can render it differently.
      void setFromKF(FrameHessian *fh, CalibHessian *HCalib);

      // copies points from KF over to internal buffer,
      // keeping some additional information so we can render it differently.
      void setFromF(FrameShell *fs, CalibHessian *HCalib);

      void setFromPose(const SE3 &pose, CalibHessian *HCalib);

      // renders cam & pointcloud.
      std::unique_ptr<visualization_msgs::msg::Marker> drawCam(const rclcpp::Time &stamp, const float *color = nullptr, float lineWidth = 0.01, float sizeFactor = 0.05, int id = -1);
      std::unique_ptr<visualization_msgs::msg::Marker> drawPointcloud(const rclcpp::Time &stamp, double pointSize = 0.01, float scaledTH = 0.001, float absTH = 0.001, int mode = 1, float minBS = 0.1, int sparsity = 1);

      int id;
      SE3 camToWorld;
      std::string name, frame_odom;

      inline bool operator<(const KeyFrameDisplay &other) const
      {
        return (id < other.id);
      }

    private:
      float fx, fy, cx, cy;
      float fxi, fyi, cxi, cyi;
      int width, height;

      float my_scaledTH, my_absTH, my_scale;
      int my_sparsifyFactor;
      int my_displayMode;
      float my_minRelBS;

      int numSparsePoints;
      int numSparseBufferSize;
      InputPointSparse<MAX_RES_PER_POINT> *originalInputSparse;

      Settings *settings;
    };

  }
}
