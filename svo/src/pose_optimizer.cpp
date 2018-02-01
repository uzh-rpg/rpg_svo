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

#include <stdexcept>
#include <svo/pose_optimizer.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace svo {
namespace pose_optimizer {

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  // init
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  // compute the scale of the error for robust estimation
  std::vector<float> errors; errors.reserve(frame->fts_.size());
  for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1<<(*it)->level);
    errors.push_back(e.norm());
  }
  if(errors.empty())
    return;
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);

  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;
  for(size_t iter=0; iter<n_iter; iter++)
  {
    // overwrite scale
    if(iter == 5)
      scale = 0.85/frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    // compute residual
    for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
      e *= sqrt_inv_cov;
      if(iter == 0)
        chi2_vec_init.push_back(e.squaredNorm()); // just for debug
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm()/scale);
      A.noalias() += J.transpose()*J*weight;
      b.noalias() -= J.transpose()*e*weight;
      new_chi2 += e.squaredNorm()*weight;
    }

    // solve linear system
    const Vector6d dT(A.ldlt().solve(b));

    // check if error increased
    if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
    {
      if(verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old; // roll-back
      break;
    }

    // update the model
    SE3 T_new = SE3::exp(dT)*frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;
    if(verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    // stop when converged
    if(vk::norm_max(dT) <= EPS)
      break;
  }

  // Set covariance as inverse information matrix. Optimistic estimator!
  const double pixel_variance=1.0;
  frame->Cov_ = pixel_variance*(A*std::pow(frame->cam_->errorMultiplier2(),2)).inverse();

  // Remove Measurements with too large reprojection error
  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if(e.norm() > reproj_thresh_scaled)
    {
      // we don't need to delete a reference in the point since it was not created yet
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  error_init=0.0;
  error_final=0.0;
  if(!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init))*frame->cam_->errorMultiplier2();
  if(!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final))*frame->cam_->errorMultiplier2();

  estimated_scale *= frame->cam_->errorMultiplier2();
  if(verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;
  num_obs -= n_deleted_refs;
}

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FrameBundle::Ptr frames,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  // init
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old_W_B(frames->get_T_W_B());
  Matrix6d A;
  Vector6d b;
  float focal_len = frames->at(0)->cam_->errorMultiplier2();

  // compute the scale of the error for robust estimation
  std::vector<float> errors; errors.reserve(frames->numFeatures());
  for(auto i=0; i<frames->size(); i++)
  {
    FramePtr frame = frames->at(i);
    for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point == NULL)
        continue;
      Vector2d e = vk::project2d((*it)->f)
                 - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
      e *= 1.0 / (1<<(*it)->level);
      errors.push_back(e.norm());
    }
  }
  if(errors.empty())
    return;
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);

  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;
  for(size_t iter=0; iter<n_iter; iter++)
  {
    // overwrite scale
    if(iter == 5)
      scale = 0.85/focal_len;

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    // compute residual
    for(auto i=0; i<frames->size(); i++)
    {
      FramePtr frame = frames->at(i);
      for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
      {
        if((*it)->point == NULL)
          continue;
        Matrix26d J;
        Vector3d xyz_body(frames->get_T_B_W() * (*it)->point->pos_);
        Frame::jacobian_xyz2uv_imu(frame->T_cam_body_, xyz_body, J);
        Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_cam_body_ * xyz_body);
        double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
        e *= sqrt_inv_cov;
        if(iter == 0)
          chi2_vec_init.push_back(e.squaredNorm()); // just for debug
        J *= sqrt_inv_cov;
        double weight = weight_function.value(e.norm()/scale);
        A.noalias() += J.transpose()*J*weight;
        b.noalias() -= J.transpose()*e*weight;
        new_chi2 += e.squaredNorm()*weight;
      }
    }

    // solve linear system
    const Vector6d dT(A.ldlt().solve(b));

    // check if error increased
    if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
    {
      if(verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frames->set_T_W_B(T_old_W_B);
      break;
    }

    // update the model
    T_old_W_B = frames->get_T_W_B();
    frames->set_T_W_B((SE3::exp(dT)*frames->get_T_B_W()).inverse());
    chi2 = new_chi2;
    if(verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    // stop when converged
    if(vk::norm_max(dT) <= EPS)
      break;
  }

  // Set covariance as inverse information matrix. Optimistic estimator!
  const double pixel_variance=1.0;
  for(auto i=0; i<frames->size(); i++)
  {
    frames->at(i)->Cov_ = pixel_variance*(A*std::pow(frames->at(i)->cam_->errorMultiplier2(),2)).inverse();
  }  

  // Remove Measurements with too large reprojection error
  double reproj_thresh_scaled = reproj_thresh / focal_len;
  size_t n_deleted_refs = 0;
  for(auto i=0; i<frames->size(); i++)
  {
    FramePtr frame = frames->at(i);
    for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
      if((*it)->point == NULL)
        continue;
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
      double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
      e *= sqrt_inv_cov;
      chi2_vec_final.push_back(e.squaredNorm());
      if(e.norm() > reproj_thresh_scaled)
      {
        // we don't need to delete a reference in the point since it was not created yet
        (*it)->point = NULL;
        ++n_deleted_refs;
      }
    }
  }
  error_init=0.0;
  error_final=0.0;
  if(!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init))*focal_len;
  if(!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final))*focal_len;

  estimated_scale *= focal_len;
  if(verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;
  num_obs -= n_deleted_refs;
}

} // namespace pose_optimizer
} // namespace svo
