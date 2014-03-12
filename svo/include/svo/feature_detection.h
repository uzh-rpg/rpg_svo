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

#ifndef SVO_FEATURE_DETECTION_H_
#define SVO_FEATURE_DETECTION_H_

#include <svo/global.h>
#include <svo/frame.h>

namespace svo {

/// Implementation of various feature detectors.
namespace feature_detection {

/// Temporary container used for corner detection. Features are initialized from these.
// TODO: why not use feature struct?
struct Corner
{
  int x;        //!< x-coordinate of corner in the image.
  int y;        //!< y-coordinate of corner in the image.
  int level;    //!< pyramid level of the corner.
  float score;  //!< shi-tomasi score of the corner.
  float angle;  //!< for gradient-features: dominant gradient angle.
  Corner(int x, int y, float score, int level, float angle) :
    x(x), y(y), level(level), score(score), angle(angle)
  {}
};
typedef vector<Corner> Corners;

/// All detectors should derive from this abstract class.
class AbstractDetector
{
public:
  virtual ~AbstractDetector() {};
  virtual void detect(
      const ImgPyr& img_pyr,
      const Features& fts,
      const int cell_size,
      const int n_levels,
      const double detection_threshold,
      Corners* corners) const = 0;
};
typedef boost::shared_ptr<AbstractDetector> DetectorPtr;

/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
  FastDetector() {}
  virtual ~FastDetector() {}
  virtual void detect(
        const ImgPyr& img_pyr,
        const Features& fts,
        const int cell_size,
        const int n_levels,
        const double detection_threshold,
        Corners* corners) const;
};

} // namespace feature_detection
} // namespace svo

#endif // SVO_FEATURE_DETECTION_H_
