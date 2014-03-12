// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_MATCHER_H_
#define SVO_MATCHER_H_

#include <svo/global.h>

namespace vk {
  class AbstractCamera;
}

namespace svo {

class Point;
class Frame;
class Feature;

/// Patch-matcher for reprojection-matching and epipolar search in triangulation.
namespace matcher {

const int g_halfpatch_size = 4; // do not change, because of sse instructions alignment.
const int g_patch_size = 8;     // do not change, because of sse instructions alignment.

/// Manages the aligned memory of a patch. Aligned for SIMD instructions.
struct PatchData
{
  const int patch_size;
  uint8_t*  patch;
  uint8_t*  patch_with_border;
  PatchData(int patch_size);
  ~PatchData();
};

/// Warp a patch from the reference view to the current view.
namespace warp {

void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref);

int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level);

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int level_cur,
    PatchData& ref_patch);

} // namespace warp

// TODO take function from vikit
bool depthFromTriangulation(
    const SE3& T_cur_ref,
    const Vector3d& f_ref,
    const Vector3d& f_cur,
    double& depth);

/// Find a match by directly applying subpix refinement.
/// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
bool findMatchDirect(
    const Point& pt,
    const Frame& frame,
    Vector2d& px_cur,
    int& level_cur);

/// Find a match by searching along the epipolar line without using any features.
bool findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    const bool align_1d,
    double& depth,
    double& h_inv);

} // namespace matcher
} // namespace svo

#endif // SVO_MATCHER_H_
