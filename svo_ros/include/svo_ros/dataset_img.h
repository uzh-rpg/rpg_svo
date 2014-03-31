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

#ifndef DATASET_IMG_H
#define DATASET_IMG_H

namespace FileType {

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

using ::Eigen::Quaterniond;
using ::Eigen::Vector3d;

/// Specification of Dataset which contains: Camera Image, Gyroscope Rotation
class DatasetImg
{
public:
  DatasetImg() {}
  virtual ~DatasetImg() {}

  double timestamp_;
  std::string image_name_;

  friend std::ostream& operator <<(std::ostream& out, const DatasetImg& pair);
  friend std::istream& operator >>(std::istream& in, DatasetImg& pair);
};

std::ostream& operator <<(std::ostream& out, const DatasetImg& gt)
{
  out
    << gt.timestamp_ << " "
    << gt.image_name_ << " "
    << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, DatasetImg& gt)
{
  in >> gt.timestamp_;
  in >> gt.image_name_;
  return in;
}

} // end namespace FileType


#endif /* DATASET_IMG_H */
