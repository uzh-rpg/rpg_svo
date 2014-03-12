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

#include <cstdlib>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/aligned_mem.h>
#include <vikit/patch_score.h>
#include <svo/matcher.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/feature_alignment.h>

namespace svo {
namespace matcher {

PatchData::PatchData(int patch_size) :
    patch_size(patch_size)
{
  patch = vk::aligned_mem::aligned_alloc<uint8_t>(patch_size*patch_size, 16);
  patch_with_border = vk::aligned_mem::aligned_alloc<uint8_t>((patch_size+2)*(patch_size+2), 16);
}

PatchData::~PatchData()
{
  free(patch);
  free(patch_with_border);
}

namespace warp {

void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const int halfpatch_size = 5;
  const Vector3d xyz_ref(f_ref*depth_ref);
  Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
  Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
  const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level)
{
  // Compute patch level in other image
  int level_cur = 0;
  double D = A_cur_ref.determinant();
  while(D > 3.0 && level_cur < max_level)
  {
    level_cur += 1;
    D *= 0.25;
  }
  return level_cur;
}

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int level_cur,
    PatchData& ref_patch)
{
  const int halfpatch_size = ref_patch.patch_size/2+1;
  const int patch_size = halfpatch_size*2 ;
  Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  // Perform the Warp on a larger patch
  uint8_t* ref_patch_ptr = ref_patch.patch_with_border;
  const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x, ++ref_patch_ptr)
    {
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<level_cur);
      const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        *ref_patch_ptr = 0;
      else
        *ref_patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }

  ref_patch_ptr = ref_patch.patch;
  for(int y=1; y<g_patch_size+1; ++y, ref_patch_ptr += g_patch_size)
  {
    uint8_t* ref_patch_border_ptr = ref_patch.patch_with_border + y*(g_patch_size+2) + 1;
    for(int x=0; x<g_patch_size; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

} // namespace warp

bool
findMatchDirect(
    const Point& pt,
    const Frame& cur_frame,
    Vector2d& px_cur,
    int& level_cur)
{
  Feature* ref_ftr;
  if(!pt.getCloseViewObs(cur_frame.pos(), ref_ftr))
    return false;

  if(!ref_ftr->frame->cam_->isInFrame(
      ref_ftr->px.cast<int>()/(1<<ref_ftr->level), g_halfpatch_size+2, ref_ftr->level))
    return false;

  // warp affine
  PatchData ref_patch(g_patch_size);
  Matrix2d A_cur_ref;
  warp::getWarpMatrixAffine(
      *ref_ftr->frame->cam_, *cur_frame.cam_, ref_ftr->px, ref_ftr->f,
      (ref_ftr->frame->pos() - pt.pos_).norm(), cur_frame.T_f_w_ * ref_ftr->frame->T_f_w_.inverse(), ref_ftr->level, A_cur_ref);
  level_cur = warp::getBestSearchLevel(A_cur_ref, Config::nPyrLevels()-1);
  warp::warpAffine(
      A_cur_ref, ref_ftr->frame->img_pyr_[ref_ftr->level], ref_ftr->px,  ref_ftr->level, level_cur, ref_patch);

  // px_cur should be set
  Vector2d px_scaled(px_cur/(1<<level_cur));
  bool success = feature_alignment::align2D(
      cur_frame.img_pyr_[level_cur], ref_patch.patch_with_border, ref_patch.patch, 10, px_scaled);
  px_cur = px_scaled * (1<<level_cur);
  return success;
}

bool
depthFromTriangulation(
    const SE3& T_search_ref,
    const Vector3d& f_ref,
    const Vector3d& f_cur,
    double& depth)
{
  Matrix<double,3,2> A; A << T_search_ref.rotation_matrix() * f_ref, f_cur;
  const Matrix2d AtA = A.transpose()*A;
  if(AtA.determinant() < 0.000001)
    return false;
  const Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
  depth = fabs(depth2[0]);
  return true;
}

bool
findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    const bool align_1d,
    double& depth,
    double& h_inv)
{
  typedef vk::patch_score::ZMSSD<g_halfpatch_size> PatchScore;

  PatchData ref_patch(g_patch_size);
  SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();
  int zmssd_best = PatchScore::threshold();
  Vector2d uv_best;

  // Compute start and end of epipolar line in old_kf for match search, on unit plane!
  Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min));
  Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max));
  Vector2d epi_dir = A - B;

  // Compute affine warp matrix
  Matrix2d A_cur_ref;
  warp::getWarpMatrixAffine(
      *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref);
  int level_cur = warp::getBestSearchLevel(A_cur_ref, Config::nPyrLevels()-1);

  // Find length of search range on epipolar line
  Vector2d px_A(cur_frame.cam_->world2cam(A));
  Vector2d px_B(cur_frame.cam_->world2cam(B));
  double px_length = (px_A-px_B).norm() / (1<<level_cur);

#ifdef DEBUG_MATCHER_EPISEARCH
    printf("px_length = %f\n", px_length);
    cv::Mat img_rgb = cv::Mat(cur_frame.img_pyr_.at(level_cur).size(), CV_8UC3);
    cv::cvtColor(cur_frame.img_pyr_.at(level_cur), img_rgb, CV_GRAY2RGB);
#endif

  // Warp reference patch at ref_level
  warp::warpAffine(
      A_cur_ref, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,  ref_ftr.level, level_cur, ref_patch);

  //printf("px_length = %f\n", px_length);
  if(px_length < 2.0)
  {
    Vector2d px = (px_A+px_B)/2.0;
    Vector2d px_scaled(px/(1<<level_cur));
    bool res;
    if(align_1d)
      res = feature_alignment::align1D(
          cur_frame.img_pyr_[level_cur], (px_A-px_B).cast<float>().normalized(),
          ref_patch.patch_with_border, ref_patch.patch, 10, px_scaled, h_inv);
    else
      res = feature_alignment::align2D(
          cur_frame.img_pyr_[level_cur], ref_patch.patch_with_border,
          ref_patch.patch, 10, px_scaled);
    if(res)
    {
      px = px_scaled*(1<<level_cur);
      if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px), depth))
        return true;
    }
    return false;
  }

  size_t n_steps = px_length/0.7; // one step per pixel
  Vector2d step = epi_dir/n_steps;

  if(n_steps > 1000)
  {
    printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
           n_steps, px_length, d_min, d_max);
    return false;
  }

  // for matching, precompute sum and sum2 of warped reference patch
  int pixel_sum = 0;
  int pixel_sum_square = 0;
  PatchScore patch_score(ref_patch.patch);

  // now we sample along the epipolar line
  Vector2d uv = B-step;
  Vector2i last_checked_pxi(0,0);
  ++n_steps;
  for(size_t i=0; i<n_steps; ++i, uv+=step)
  {
    Vector2d px(cur_frame.cam_->world2cam(uv));
    Vector2i pxi(px[0]/(1<<level_cur)+0.5,
                 px[1]/(1<<level_cur)+0.5); // +0.5 to round to closest int

#ifdef DEBUG_MATCHER_EPISEARCH
    cv::circle(img_rgb, cv::Point2f(pxi[0], pxi[1]), 1, cv::Scalar(0,0,255), 1);
#endif

    if(pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if(!cur_frame.cam_->isInFrame(pxi, g_patch_size, level_cur))
      continue;

    // TODO interpolation would probably be a good idea
    uint8_t* cur_patch_ptr = cur_frame.img_pyr_[level_cur].data + (pxi[1]-g_halfpatch_size)*cur_frame.img_pyr_[level_cur].cols + (pxi[0]-g_halfpatch_size);
    int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[level_cur].cols);

    if(zmssd < zmssd_best) {
      zmssd_best = zmssd;
      uv_best = uv;
    }
  }

#ifdef DEBUG_MATCHER_EPISEARCH
  if(zmssd_best < g_zmssd_thresh) {
    Vector2d px(cur_frame.cam_->world2cam(uv_best));
    Vector2i pxi(px[0]/(1<<level_cur)+0.5, px[1]/(1<<level_cur)+0.5);
    cv::circle(img_rgb, cv::Point2f(px[0]/(1<<level_cur), px[1]/(1<<level_cur)), 8, cv::Scalar(255,0,0), 2);
  }
  cv::imshow("cur_img", img_rgb);
  cv::Mat ref_img_rgb = cv::Mat(ref_frame.img_pyr_.at(ref_ftr.level).size(), CV_8UC3);
  cv::cvtColor(ref_frame.img_pyr_.at(ref_ftr.level), ref_img_rgb, CV_GRAY2RGB);
  cv::circle(ref_img_rgb, cv::Point2f(ref_ftr.px[0]/(1<<ref_ftr.level), ref_ftr.px[1]/(1<<ref_ftr.level)), 8, cv::Scalar(255,0,0), 2);
  cv::imshow("ref_img", ref_img_rgb);
  uint8_t* cur_patch_ptr = cur_frame.img_pyr_[level_cur].data + (pxi[1]-g_halfpatch_size)*cur_frame.img_pyr_[level_cur].cols + (pxi[0]-g_halfpatch_size);
  cv::Mat cur_patch(g_patch_size,g_patch_size,CV_8U,cur_patch_ptr);
  cv::imshow("cur_patch", cur_patch);
  cv::waitKey(0);
#endif

  if(zmssd_best < PatchScore::threshold())
  {
//    if(px_length < 4.0)
    {
      Vector2d px(cur_frame.cam_->world2cam(uv_best));
      Vector2d px_scaled(px/(1<<level_cur));
      bool res;
      if(align_1d)
        res = feature_alignment::align1D(
            cur_frame.img_pyr_[level_cur], (px_A-px_B).cast<float>().normalized(),
            ref_patch.patch_with_border, ref_patch.patch, 10, px_scaled, h_inv);
      else
        res = feature_alignment::align2D(
            cur_frame.img_pyr_[level_cur], ref_patch.patch_with_border,
            ref_patch.patch, 10, px_scaled);
      if(res)
      {
        px = px_scaled*(1<<level_cur);
        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px), depth))
          return true;
      }
      return false;
    }

    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
      return true;
  }
  return false;
}

} // namespace matcher
} // namespace svo
