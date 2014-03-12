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

#ifndef DATASET_IMU_H_
#define DATASET_IMU_H_

namespace FileType {

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

using ::Eigen::Quaterniond;
using ::Eigen::Vector3d;

/// Specification of Dataset which contains: Camera Image, Gyroscope Rotation
class DatasetImu
{
public:
  DatasetImu() {}
  virtual ~DatasetImu() {}

  double stamp_;
  Quaterniond q_imu_;

  friend std::ostream& operator <<(std::ostream& out, const DatasetImu& pair);
  friend std::istream& operator >>(std::istream& in, DatasetImu& pair);
};

std::ostream& operator <<(std::ostream& out, const DatasetImu& gt)
{
  out
    << gt.stamp_ << " "
    << gt.q_imu_.x() << " "
    << gt.q_imu_.y() << " "
    << gt.q_imu_.z() << " "
    << gt.q_imu_.w() << " "
    << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, DatasetImu& gt)
{
  in >> gt.stamp_;
  double x,y,z,w;
  in >> x;
  in >> y;
  in >> z;
  in >> w;
  gt.q_imu_ = Quaterniond(w, x, y, z);
  return in;
}

} // end namespace FileType


#endif /* DATASET_IMU_H_ */
