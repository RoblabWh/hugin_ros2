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

#include "keyframe_display.hpp"
#include "util/settings.h"
#include "util/FrameShell.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"

namespace dso
{
  namespace IOWrap
  {

    KeyFrameDisplay::KeyFrameDisplay(Settings *settings, const std::string &name, const std::string &frame_odom)
        : name(name), frame_odom(frame_odom), settings(settings)
    {
      originalInputSparse = 0;
      numSparseBufferSize = 0;
      numSparsePoints = 0;

      id = 0;
      camToWorld = SE3();

      my_scaledTH = 0.001;
      my_absTH = 0.001;
      my_displayMode = 1;
      my_minRelBS = 0.1;
      my_sparsifyFactor = 1;
    }

    void KeyFrameDisplay::setFromF(FrameShell *frame, CalibHessian *HCalib)
    {
      id = frame->id;
      fx = HCalib->fxl();
      fy = HCalib->fyl();
      cx = HCalib->cxl();
      cy = HCalib->cyl();
      width = settings->calibG.wG[0];
      height = settings->calibG.hG[0];
      fxi = 1 / fx;
      fyi = 1 / fy;
      cxi = -cx / fx;
      cyi = -cy / fy;
      camToWorld = frame->camToWorld;
    }

    void KeyFrameDisplay::setFromPose(const SE3 &pose, CalibHessian *HCalib)
    {
      id = 0;
      fx = HCalib->fxl();
      fy = HCalib->fyl();
      cx = HCalib->cxl();
      cy = HCalib->cyl();
      width = settings->calibG.wG[0];
      height = settings->calibG.hG[0];
      fxi = 1 / fx;
      fyi = 1 / fy;
      cxi = -cx / fx;
      cyi = -cy / fy;
      camToWorld = pose;
    }

    void KeyFrameDisplay::setFromKF(FrameHessian *fh, CalibHessian *HCalib)
    {
      setFromF(fh->shell, HCalib);

      // add all traces, inlier and outlier points.
      int npoints = fh->immaturePoints.size() +
                    fh->pointHessians.size() +
                    fh->pointHessiansMarginalized.size() +
                    fh->pointHessiansOut.size();

      if (numSparseBufferSize < npoints)
      {
        if (originalInputSparse != 0)
          delete originalInputSparse;
        numSparseBufferSize = npoints + 100;
        originalInputSparse = new InputPointSparse<MAX_RES_PER_POINT>[numSparseBufferSize];
      }

      InputPointSparse<MAX_RES_PER_POINT> *pc = originalInputSparse;
      numSparsePoints = 0;
      for (ImmaturePoint *p : fh->immaturePoints)
      {
        for (int i = 0; i < patternNum; i++)
          pc[numSparsePoints].color[i] = p->color[i];

        pc[numSparsePoints].u = p->u;
        pc[numSparsePoints].v = p->v;
        pc[numSparsePoints].idepth = (p->idepth_max + p->idepth_min) * 0.5f;
        pc[numSparsePoints].idepth_hessian = 1000;
        pc[numSparsePoints].relObsBaseline = 0;
        pc[numSparsePoints].numGoodRes = 1;
        pc[numSparsePoints].status = 0;
        numSparsePoints++;
      }

      for (PointHessian *p : fh->pointHessians)
      {
        for (int i = 0; i < patternNum; i++)
          pc[numSparsePoints].color[i] = p->color[i];
        pc[numSparsePoints].u = p->u;
        pc[numSparsePoints].v = p->v;
        pc[numSparsePoints].idepth = p->idepth_scaled;
        pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
        pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
        pc[numSparsePoints].numGoodRes = 0;
        pc[numSparsePoints].status = 1;

        numSparsePoints++;
      }

      for (PointHessian *p : fh->pointHessiansMarginalized)
      {
        for (int i = 0; i < patternNum; i++)
          pc[numSparsePoints].color[i] = p->color[i];
        pc[numSparsePoints].u = p->u;
        pc[numSparsePoints].v = p->v;
        pc[numSparsePoints].idepth = p->idepth_scaled;
        pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
        pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
        pc[numSparsePoints].numGoodRes = 0;
        pc[numSparsePoints].status = 2;
        numSparsePoints++;
      }

      for (PointHessian *p : fh->pointHessiansOut)
      {
        for (int i = 0; i < patternNum; i++)
          pc[numSparsePoints].color[i] = p->color[i];
        pc[numSparsePoints].u = p->u;
        pc[numSparsePoints].v = p->v;
        pc[numSparsePoints].idepth = p->idepth_scaled;
        pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
        pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
        pc[numSparsePoints].numGoodRes = 0;
        pc[numSparsePoints].status = 3;
        numSparsePoints++;
      }
      assert(numSparsePoints <= npoints);

      camToWorld = fh->PRE_camToWorld;
    }

    KeyFrameDisplay::~KeyFrameDisplay()
    {
      if (originalInputSparse != 0)
        delete[] originalInputSparse;
    }

    std::unique_ptr<visualization_msgs::msg::Marker> KeyFrameDisplay::drawPointcloud(const rclcpp::Time &stamp, double pointSize, float scaledTH, float absTH, int mode, float minBS, int sparsity)
    {
      my_scaledTH = scaledTH;
      my_absTH = absTH;
      my_displayMode = mode;
      my_minRelBS = minBS;
      my_sparsifyFactor = sparsity;

      // if there are no vertices, done!
      if (numSparsePoints == 0)
        return nullptr;

      // make data
      auto msg = std::make_unique<visualization_msgs::msg::Marker>();
      msg->header.frame_id = frame_odom;
      msg->header.stamp = stamp;
      msg->ns = "pointcloud";
      msg->id = id;
      msg->type = visualization_msgs::msg::Marker::POINTS;
      msg->action = visualization_msgs::msg::Marker::MODIFY;

      const auto transCamToWorld = camToWorld.translation();
      const auto rotCamToWorld = camToWorld.unit_quaternion();
      msg->pose.position.x = transCamToWorld.x();
      msg->pose.position.y = transCamToWorld.y();
      msg->pose.position.z = transCamToWorld.z();
      msg->pose.orientation.x = rotCamToWorld.x();
      msg->pose.orientation.y = rotCamToWorld.y();
      msg->pose.orientation.z = rotCamToWorld.z();
      msg->pose.orientation.w = rotCamToWorld.w();

      msg->scale.x = pointSize;
      msg->scale.y = pointSize;

      int numPoints = 0;
      for (int i = 0; i < numSparsePoints; i++)
      {
        /* display modes:
         * my_displayMode==0 - all pts, color-coded
         * my_displayMode==1 - normal points
         * my_displayMode==2 - active only
         * my_displayMode==3 - nothing
         */

        if (my_displayMode == 1 && originalInputSparse[i].status != 1 && originalInputSparse[i].status != 2)
          continue;
        if (my_displayMode == 2 && originalInputSparse[i].status != 1)
          continue;
        if (my_displayMode > 2)
          continue;

        if (originalInputSparse[i].idepth < 0)
          continue;

        float depth = (1.0f / originalInputSparse[i].idepth);
        float depth4 = depth * depth;
        depth4 *= depth4;
        float var = (1.0f / (originalInputSparse[i].idepth_hessian + 0.01));

        if (var * depth4 > my_scaledTH)
          continue;

        if (var > my_absTH)
          continue;

        if (originalInputSparse[i].relObsBaseline < my_minRelBS)
          continue;

        for (int pnt = 0; pnt < patternNum; pnt++)
        {
          if (my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0)
            continue;
          int dx = patternP[pnt][0];
          int dy = patternP[pnt][1];

          auto &point = msg->points.emplace_back();
          point.x = ((originalInputSparse[i].u + dx) * fxi + cxi) * depth;
          point.y = ((originalInputSparse[i].v + dy) * fyi + cyi) * depth;
          point.z = depth * (1 + 2 * fxi * (rand() / (float)RAND_MAX - 0.5f));

          auto &color = msg->colors.emplace_back();
          color.a = 1.0;
          if (my_displayMode == 0)
          {
            if (originalInputSparse[i].status == 0)
            {
              color.r = 0.0;
              color.g = 1.0;
              color.b = 1.0;
            }
            else if (originalInputSparse[i].status == 1)
            {
              color.r = 0.0;
              color.g = 1.0;
              color.b = 0.0;
            }
            else if (originalInputSparse[i].status == 2)
            {
              color.r = 0.0;
              color.g = 0.0;
              color.b = 1.0;
            }
            else if (originalInputSparse[i].status == 3)
            {
              color.r = 1.0;
              color.g = 0.0;
              color.b = 0.0;
            }
            else
            {
              color.r = 1.0;
              color.g = 1.0;
              color.b = 1.0;
            }
          }
          else
          {
            float c = originalInputSparse[i].color[pnt] / 255.0f;
            color.r = c;
            color.g = c;
            color.b = c;
          }
          numPoints++;

          assert(numPoints <= numSparsePoints * patternNum);
        }
      }

      return std::move(msg);
    }

    std::unique_ptr<visualization_msgs::msg::Marker> KeyFrameDisplay::drawCam(const rclcpp::Time &stamp, const float *color, float lineWidth, float sizeFactor, int id)
    {
      if (width == 0)
        return nullptr;

      const float sz = sizeFactor;

      auto msg = std::make_unique<visualization_msgs::msg::Marker>();
      msg->header.frame_id = frame_odom;
      msg->header.stamp = stamp;
      msg->ns = name;
      msg->id = id < 0 ? this->id : id;
      msg->type = visualization_msgs::msg::Marker::LINE_LIST;
      msg->action = visualization_msgs::msg::Marker::MODIFY;

      if (color == nullptr)
      {
        msg->color.r = 1.0;
        msg->color.g = 0.0;
        msg->color.b = 0.0;
      }
      else
      {
        msg->color.r = color[0];
        msg->color.g = color[1];
        msg->color.b = color[2];
      }
      msg->color.a = 1.0;
      msg->scale.x = lineWidth;

      const auto transCamToWorld = camToWorld.translation();
      const auto rotCamToWorld = camToWorld.unit_quaternion();
      msg->pose.position.x = transCamToWorld.x();
      msg->pose.position.y = transCamToWorld.y();
      msg->pose.position.z = transCamToWorld.z();
      msg->pose.orientation.x = rotCamToWorld.x();
      msg->pose.orientation.y = rotCamToWorld.y();
      msg->pose.orientation.z = rotCamToWorld.z();
      msg->pose.orientation.w = rotCamToWorld.w();

      geometry_msgs::msg::Point center, top_left, top_right, bottom_left, bottom_right;
      const float left = sz * (0 - cx) / fx;
      const float right = sz * (width - 1 - cx) / fx;
      const float top = sz * (0 - cy) / fy;
      const float bottom = sz * (height - 1 - cy) / fy;

      center.x = 0;
      center.y = 0;
      center.z = 0;
      top_left.x = left;
      top_left.y = top;
      top_left.z = sz;
      bottom_left.x = left;
      bottom_left.y = bottom;
      bottom_left.z = sz;
      top_right.x = right;
      top_right.y = top;
      top_right.z = sz;
      bottom_right.x = right;
      bottom_right.y = bottom;
      bottom_right.z = sz;

      msg->points.resize(16);
      msg->points[0] = center;
      msg->points[1] = top_left;
      msg->points[2] = center;
      msg->points[3] = bottom_left;
      msg->points[4] = center;
      msg->points[5] = bottom_right;
      msg->points[6] = center;
      msg->points[7] = top_right;
      msg->points[8] = top_left;
      msg->points[9] = top_right;
      msg->points[10] = top_right;
      msg->points[11] = bottom_right;
      msg->points[12] = bottom_right;
      msg->points[13] = bottom_left;
      msg->points[14] = bottom_left;
      msg->points[15] = top_left;

      return std::move(msg);
    }

  }
}
