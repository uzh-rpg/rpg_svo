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

#ifndef SVO_REPROJECTION_H_
#define SVO_REPROJECTION_H_

#include <svo/global.h>
#include <svo/matcher.h>

namespace vk {
  class AbstractCamera;
}

namespace svo {

class Map;
class Point;
class MapPointCandidates;

/// Project points from the map into the image and find the corresponding
/// feature (corner). We don't search a match for every point but only for one
/// point per cell. Thereby, we achieve a homogeneously distributed set of
/// matched features and at the same time we can save processing time by not
/// projecting all points.
namespace reprojection {

/// A candidate is a point that projects into the image plane and for which we
/// will search a maching feature in the image.
struct Candidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Point* pt;       //!< 3D point.
  Vector2d px;     //!< projected 2D pixel location.
  Candidate(Point* pt, Vector2d& px)
   : pt(pt),
     px(px)
  {}
};

typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;
typedef std::vector<Cell*> CandidateGrid;

/// The grid stores a set of candidate matches. For every grid cell we try to find one match.
struct Grid
{
  CandidateGrid cells;
  vector<int> cell_order;
  int cell_size;
  int grid_n_cols;
  int grid_n_rows;
  Grid();
  void initialize(vk::AbstractCamera* cam);
  ~Grid();
};

/// Reset the grid
void resetGrid(Grid& grid);

/// Project points from the map into the image. First finds keyframes with
/// overlapping field of view and projects only those map-points.
void reprojectMap(
    Map& map,
    FramePtr frame,
    Grid& grid,
    MapPointCandidates& candidate_points,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs,
    size_t& n_matches,
    size_t& n_trials);

bool reprojectCell(
    Cell& cell,
    FramePtr frame,
    Map& map,
    MapPointCandidates& candidate_points,
    size_t& n_trials);

bool reprojectPoint(
    Grid& grid,
    FramePtr frame,
    Point* point);

} // namespace reprojection
} // namespace svo

#endif // SVO_REPROJECTION_H_
