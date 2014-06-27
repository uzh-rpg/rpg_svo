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

#include <algorithm>
#include <svo/sparse_img_align.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/config.h>
#include <svo/point.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>

namespace svo {

SparseImgAlign::SparseImgAlign(
    int max_level, int min_level, int n_iter,
    Method method, bool display, bool verbose) :
        display_(display),
        max_level_(max_level),
        min_level_(min_level)
{
  n_iter_ = n_iter;
  n_iter_init_ = n_iter_;
  method_ = method;
  verbose_ = verbose;
  eps_ = 0.000001;
}

size_t SparseImgAlign::run(FramePtr ref_frame, FramePtr cur_frame)
{
  reset();

  if(ref_frame->fts_.empty())
  {
    SVO_WARN_STREAM("SparseImgAlign: no features to track!");
    return 0;
  }

  ref_frame_ = ref_frame;
  cur_frame_ = cur_frame;
  ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F);
  jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows*patch_area_);
  visible_fts_.resize(ref_patch_cache_.rows, false); // TODO: should it be reset at each level?

  SE3 T_cur_from_ref(cur_frame_->T_f_w_ * ref_frame_->T_f_w_.inverse());

  for(level_=max_level_; level_>=min_level_; --level_)
  {
    mu_ = 0.1;
    jacobian_cache_.setZero();
    have_ref_patch_cache_ = false;
    if(verbose_)
      printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
    optimize(T_cur_from_ref);
  }
  cur_frame_->T_f_w_ = T_cur_from_ref * ref_frame_->T_f_w_;



  return n_meas_/patch_area_;
}

Matrix<double, 6, 6> SparseImgAlign::getFisherInformation()
{
  double sigma_i_sq = 5e-4*255*255; // image noise
  Matrix<double,6,6> I = H_/sigma_i_sq;
  return I;
}

void SparseImgAlign::precomputeReferencePatches()
{
  const int border = patch_halfsize_+1;
  const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_);
  const int stride = ref_img.cols;
  const float scale = 1.0f/(1<<level_);
  const Vector3d ref_pos = ref_frame_->pos();
  const double focal_length = ref_frame_->cam_->errorMultiplier2();
  size_t feature_counter = 0;
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  for(auto it=ref_frame_->fts_.begin(), ite=ref_frame_->fts_.end();
      it!=ite; ++it, ++feature_counter, ++visiblity_it)
  {
    // check if reference with patch size is within image
    const float u_ref = (*it)->px[0]*scale;
    const float v_ref = (*it)->px[1]*scale;
    const int u_ref_i = floorf(u_ref);
    const int v_ref_i = floorf(v_ref);
    if((*it)->point == NULL || u_ref_i-border < 0 || v_ref_i-border < 0 || u_ref_i+border >= ref_img.cols || v_ref_i+border >= ref_img.rows)
      continue;
    *visiblity_it = true;

    // cannot just take the 3d points coordinate because of the reprojection errors in the reference image!!!
    const double depth(((*it)->point->pos_ - ref_pos).norm());
    const Vector3d xyz_ref((*it)->f*depth);

    // evaluate projection jacobian
    Matrix<double,2,6> frame_jac;
    Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

    // compute bilateral interpolation weights for reference image
    const float subpix_u_ref = u_ref-u_ref_i;
    const float subpix_v_ref = v_ref-v_ref_i;
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    size_t pixel_counter = 0;
    float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i+y-patch_halfsize_)*stride + (u_ref_i-patch_halfsize_);
      for(int x=0; x<patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
      {
        // precompute interpolated reference patch color
        *cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];

        // we use the inverse compositional: thereby we can take the gradient always at the same position
        // get gradient of warped image (~gradient at warped position)
        float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + w_ref_bl*ref_img_ptr[stride+1] + w_ref_br*ref_img_ptr[stride+2])
                          -(w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + w_ref_bl*ref_img_ptr[stride-1] + w_ref_br*ref_img_ptr[stride]));
        float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1+stride] + w_ref_bl*ref_img_ptr[stride*2] + w_ref_br*ref_img_ptr[stride*2+1])
                          -(w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1-stride] + w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));

        // cache the jacobian
        jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) =
            (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1<<level_));
      }
    }
  }
  have_ref_patch_cache_ = true;
}

double SparseImgAlign::computeResiduals(
    const SE3& T_cur_from_ref,
    bool linearize_system,
    bool compute_weight_scale)
{
  // Warp the (cur)rent image such that it aligns with the (ref)erence image
  const cv::Mat& cur_img = cur_frame_->img_pyr_.at(level_);

  if(linearize_system && display_)
    resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

  if(have_ref_patch_cache_ == false)
    precomputeReferencePatches();

  // compute the weights on the first iteration
  std::vector<float> errors;
  if(compute_weight_scale)
    errors.reserve(visible_fts_.size());
  const int stride = cur_img.cols;
  const int border = patch_halfsize_+1;
  const float scale = 1.0f/(1<<level_);
  const Vector3d ref_pos(ref_frame_->pos());
  float chi2 = 0.0;
  size_t feature_counter = 0; // is used to compute the index of the cached jacobian
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  for(auto it=ref_frame_->fts_.begin(); it!=ref_frame_->fts_.end();
      ++it, ++feature_counter, ++visiblity_it)
  {
    // check if feature is within image
    if(!*visiblity_it)
      continue;

    // compute pixel location in cur img
    const double depth = ((*it)->point->pos_ - ref_pos).norm();
    const Vector3d xyz_ref((*it)->f*depth);
    const Vector3d xyz_cur(T_cur_from_ref * xyz_ref);
    const Vector2f uv_cur_pyr(cur_frame_->cam_->world2cam(xyz_cur).cast<float>() * scale);
    const float u_cur = uv_cur_pyr[0];
    const float v_cur = uv_cur_pyr[1];
    const int u_cur_i = floorf(u_cur);
    const int v_cur_i = floorf(v_cur);

    // check if projection is within the image
    if(u_cur_i < 0 || v_cur_i < 0 || u_cur_i-border < 0 || v_cur_i-border < 0 || u_cur_i+border >= cur_img.cols || v_cur_i+border >= cur_img.rows)
      continue;

    // compute bilateral interpolation weights for the current image
    const float subpix_u_cur = u_cur-u_cur_i;
    const float subpix_v_cur = v_cur-v_cur_i;
    const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
    const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
    const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
    const float w_cur_br = subpix_u_cur * subpix_v_cur;
    float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    size_t pixel_counter = 0; // is used to compute the index of the cached jacobian
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* cur_img_ptr = (uint8_t*) cur_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

      for(int x=0; x<patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
      {
        // compute residual
        const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride+1];
        const float res = intensity_cur - (*ref_patch_cache_ptr);

        // used to compute scale for robust cost
        if(compute_weight_scale)
          errors.push_back(fabsf(res));

        // robustification
        float weight = 1.0;
        if(use_weights_) {
          weight = weight_function_->value(res/scale_);
        }

        chi2 += res*res*weight;
        n_meas_++;

        if(linearize_system)
        {
          // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
          const Vector6d J(jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter));
          H_.noalias() += J*J.transpose()*weight;
          Jres_.noalias() -= J*res*weight;
          if(display_)
            resimg_.at<float>((int) v_cur+y-patch_halfsize_, (int) u_cur+x-patch_halfsize_) = res/255.0;
        }
      }
    }
  }

  // compute the weights on the first iteration
  if(compute_weight_scale && iter_ == 0)
    scale_ = scale_estimator_->compute(errors);

  return chi2/n_meas_;
}

int SparseImgAlign::solve()
{
  x_ = H_.ldlt().solve(Jres_);
  if((bool) std::isnan((double) x_[0]))
    return 0;
  return 1;
}

void SparseImgAlign::update(
    const ModelType& T_curold_from_ref,
    ModelType& T_curnew_from_ref)
{
  T_curnew_from_ref =  T_curold_from_ref * SE3::exp(-x_);
}

void SparseImgAlign::startIteration()
{}

void SparseImgAlign::finishIteration()
{
  if(display_)
  {
    cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);
    cv::imshow("residuals", resimg_*10);
    cv::waitKey(0);
  }
}

} // namespace svo

