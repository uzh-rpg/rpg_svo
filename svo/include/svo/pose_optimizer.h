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

#ifndef SVO_POSE_OPTIMIZER_H_
#define SVO_POSE_OPTIMIZER_H_

#include <svo/global.h>

namespace svo {

using namespace Eigen;
using namespace Sophus;
using namespace std;

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,2,6> Matrix26d;
typedef Matrix<double,6,1> Vector6d;

class Point;

/// Motion-only bundle adjustment. Minimize the reprojection error of a single frame.
namespace pose_optimizer {

inline Matrix26d
frameJac(const SE3 & T,
         const Vector3d & xyz,
         const Vector2d & focal_length)
{
  const Vector3d & xyz_trans = T*xyz;
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;
  Matrix26d frame_jac;
  frame_jac(0,0) = -1./z *focal_length[0];
  frame_jac(0,1) = 0;
  frame_jac(0,2) = x/z_2 *focal_length[0];
  frame_jac(0,3) =  x*y/z_2 * focal_length[0];
  frame_jac(0,4) = -(1+(x*x/z_2)) *focal_length[0];
  frame_jac(0,5) = y/z *focal_length[0];

  frame_jac(1,0) = 0;
  frame_jac(1,1) = -1./z *focal_length[1];
  frame_jac(1,2) = y/z_2 *focal_length[1];
  frame_jac(1,3) = (1+y*y/z_2) *focal_length[1];
  frame_jac(1,4) = -x*y/z_2 *focal_length[1];
  frame_jac(1,5) = -x/z *focal_length[1];
  return frame_jac;
}

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs);

} // namespace pose_optimizer
} // namespace svo

#endif // SVO_POSE_OPTIMIZER_H_
