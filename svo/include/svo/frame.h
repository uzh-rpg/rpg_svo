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

#ifndef SVO_FRAME_H_
#define SVO_FRAME_H_

#include <sophus/se3.h>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <boost/noncopyable.hpp>
#include <svo/global.h>

namespace g2o {
class VertexSE3Expmap;
}
typedef g2o::VertexSE3Expmap g2oFrameSE3;

namespace svo {

class Point;
struct Feature;

typedef list<Feature*> Features;
typedef vector<cv::Mat> ImgPyr;

/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  static int                    frame_counter_;         //!< Counts the number of created frames. Used to set the unique id.
  int                           id_;                    //!< Unique id of the frame.
  double                        timestamp_;             //!< Timestamp of when the image was recorded.
  vk::AbstractCamera*           cam_;                   //!< Camera model.
  Sophus::SE3                   T_f_w_;                 //!< Transform (f)rame from (w)orld.
  Matrix<double, 6, 6>          Cov_;                   //!< Covariance.
  ImgPyr                        img_pyr_;               //!< Image Pyramid.
  Features                      fts_;                   //!< List of features in the image.
  vector<Feature*>              key_pts_;               //!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
  bool                          is_keyframe_;           //!< Was this frames selected as keyframe?
  g2oFrameSE3*                  v_kf_;                  //!< Temporary pointer to the g2o node object of the keyframe.
  int                           last_published_ts_;     //!< Timestamp of last publishing.

  Sophus::SE3 T_body_cam_;
  Sophus::SE3 T_cam_body_;

  Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp);
  ~Frame();

  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat& img);

  /// Select this frame as keyframe.
  void setKeyframe();

  /// Add a feature to the image
  void addFeature(Feature* ftr);

  /// The KeyPoints are those five features which are closest to the 4 image corners
  /// and to the center and which have a 3D point assigned. These points are used
  /// to quickly check whether two frames have overlapping field of view.
  void setKeyPoints();

  /// Check if we can select five better key-points.
  void checkKeyPoints(Feature* ftr);

  /// If a point is deleted, we must remove the corresponding key-point.
  void removeKeyPoint(Feature* ftr);

  /// Return number of point observations.
  inline size_t nObs() const { return fts_.size(); }

  /// Check if a point in (w)orld coordinate frame is visible in the image.
  bool isVisible(const Vector3d& xyz_w) const;

  /// Full resolution image stored in the frame.
  inline const cv::Mat& img() const { return img_pyr_[0]; }

  /// Was this frame selected as keyframe?
  inline bool isKeyframe() const { return is_keyframe_; }

  /// Get camera pose in imu frame.
  inline const Sophus::SE3& T_imu_cam() const { return T_body_cam_; }

  /// Get imu pose in camera frame.
  inline const Sophus::SE3& T_cam_imu() const { return T_cam_body_; }

  /// Get pose of world origin in frame coordinates.
  inline const Sophus::SE3& T_cam_world() const { return T_f_w_; }

  /// Get pose of the cam in world coordinates.
  inline Sophus::SE3 T_world_cam() const { return T_f_w_.inverse(); }

  /// Get pose of imu in world coordinates.
  inline Sophus::SE3 T_world_imu() const { return (T_imu_cam()*T_f_w_).inverse(); }

  /// Get pose of world-origin in IMU coordinates.
  inline Sophus::SE3 T_imu_world() const { return T_imu_cam()*T_f_w_; }

  /// Set camera to imu transformation.
  inline void set_T_cam_body(const Sophus::SE3& T_cam_imu)
  {
    T_cam_body_ = T_cam_imu;
    T_body_cam_ = T_cam_imu.inverse();
  }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam( T_f_w_ * xyz_w ); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }

  inline static Eigen::Matrix3d skew(const Eigen::Vector3d& v)
  {
    Eigen::Matrix3d v_sqew;
    v_sqew << 0, -v[2], v[1],
              v[2], 0, -v[0],
              -v[1], v[0], 0;
    return v_sqew;
  }

  /// Frame jacobian for projection of 3D point in (f)rame coordinate to
  /// unit plane coordinates uv (focal length = 1).
  //u_hat对李代数xi的导数，默认u_hat在normalize平面，f为1
  inline static void jacobian_xyz2uv(
      const Vector3d& xyz_in_f,
      Matrix<double,2,6>& J)
  {
    const double x = xyz_in_f[0];
    const double y = xyz_in_f[1];
    const double z_inv = 1./xyz_in_f[2];
    const double z_inv_2 = z_inv*z_inv;

    J(0,0) = -z_inv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*z_inv_2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*z_inv;             // y/z

    J(1,0) = 0.0;                 // 0
    J(1,1) = -z_inv;              // -1/z
    J(1,2) = y*z_inv_2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*z_inv;            // x/z
  }

  /// Jacobian of reprojection error (on unit plane) w.r.t. IMU pose.
  inline static void jacobian_xyz2uv_imu(
      const Sophus::SE3& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      Eigen::Matrix<double,2,6>& J)
  {
    // G_x: d_p_cam/d_xi
    Eigen::Matrix<double,3,6> G_x; // Generators times pose
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -skew(p_in_imu);
    const Eigen::Vector3d p_in_cam = T_cam_imu * p_in_imu;

    Eigen::Matrix<double,2,3> J_proj; // projection derivative
    J_proj << 1, 0, -p_in_cam[0]/p_in_cam[2],
        0, 1, -p_in_cam[1]/p_in_cam[2];

    // J=d_u_hat/d_xi = d_u_hat/d_cam * d_cam/d_imu * d_imu/d_xi
    J = (- 1.0/p_in_cam[2] * J_proj) * T_cam_imu.rotation_matrix() * G_x;
  }

  /// Jacobian of reprojection error (on image plane, has focal length) w.r.t. IMU pose.
  inline static void jacobian_xyz2img_imu(
      const Sophus::SE3& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      const Eigen::Matrix<double, 2, 3>& J_cam,
      Eigen::Matrix<double,2,6>& J)
  {
    Eigen::Matrix<double,3,6> G_x; // Generators times pose
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -skew(p_in_imu);

    // J=d_u/d_xi = d_u/d_cam * d_cam/d_imu * d_imu/d_xi
    J =  J_cam * T_cam_imu.rotation_matrix() * G_x;
  }

  /// Jacobian of using unit bearing vector for map point w.r.t IMU pose.
  inline static void jacobian_xyz2f_imu(
      const Sophus::SE3& T_cam_imu,
      const Eigen::Vector3d& p_in_imu,
      Eigen::Matrix<double, 3, 6>& J)
  {
    Eigen::Matrix<double,3,6> G_x;
    G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    G_x.block<3,3>(0,3) = -skew(p_in_imu);
    const Eigen::Vector3d p_in_cam = T_cam_imu * p_in_imu;

    Eigen::Matrix<double, 3, 3> J_normalize;
    double x2 = p_in_cam[0]*p_in_cam[0];
    double y2 = p_in_cam[1]*p_in_cam[1];
    double z2 = p_in_cam[2]*p_in_cam[2];
    double xy = p_in_cam[0]*p_in_cam[1];
    double yz = p_in_cam[1]*p_in_cam[2];
    double zx = p_in_cam[2]*p_in_cam[0];
    J_normalize << y2+z2, -xy, -zx,
        -xy, x2+z2, -yz,
        -zx, -yz, x2+y2;
    J_normalize *= 1 / std::pow(x2+y2+z2, 1.5);

    J = J_normalize * T_cam_imu.rotation_matrix() * G_x;
  }

};


/// Some helper functions for the frame object.
namespace frame_utils {

/// Creates an image pyramid of half-sampled images.
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

/// Get the average depth of the features in the image.
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min);

} // namespace frame_utils
} // namespace svo

#endif // SVO_FRAME_H_
