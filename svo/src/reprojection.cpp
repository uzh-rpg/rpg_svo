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
#include <stdexcept>
#include <svo/reprojection.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {
namespace reprojection {

Grid::Grid()
{}

void Grid::initialize(vk::AbstractCamera* cam)
{
  cell_size = Config::gridSize();
  grid_n_cols = ceil(static_cast<double>(cam->width())/cell_size);
  grid_n_rows = ceil(static_cast<double>(cam->height())/cell_size);
  cells.resize(grid_n_cols*grid_n_rows);
  std::for_each(cells.begin(), cells.end(), [&](Cell*& c){ c = new Cell; });
  cell_order.resize(cells.size());
  for(size_t i=0; i<cells.size(); ++i)
    cell_order[i] = i;
  random_shuffle(cell_order.begin(), cell_order.end()); // maybe we should do it at every iteration!
}

Grid::~Grid()
{
  std::for_each(cells.begin(), cells.end(), [&](Cell* c){ delete c; });
  cells.clear();
}

void resetGrid(Grid& grid)
{
  std::for_each(grid.cells.begin(), grid.cells.end(), [&](Cell* c){ c->clear(); });
}

void reprojectMap(
    Map& map,
    FramePtr frame,
    Grid& grid,
    MapPointCandidates& candidate_points,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs,
    size_t& n_matches,
    size_t& n_trials)
{
  resetGrid(grid);

  // Identify those Keyframes which share a common field of view.
  list< pair<FramePtr,double> > close_kfs;
  map.getCloseKeyframes(frame, close_kfs);

  // Sort KFs with overlap according to their closeness // TODO is this still necessary?
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                 boost::bind(&std::pair<FramePtr, double>::second, _2));

  // Reproject all mappoints of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  size_t n = 0, max_n = 10;
  overlap_kfs.reserve(max_n);
  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<max_n; ++it_frame, ++n)
  {
    FramePtr ref_frame = it_frame->first;
    overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

    // Try to reproject each mappoint that the other KF observes
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {
      // check if the feature has a mappoint assigned
      if((*it_ftr)->point == NULL)
        continue;

      // make sure we project a point only once
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
      if(reprojectPoint(grid, frame, (*it_ftr)->point))
        overlap_kfs.back().second++;
    }
  }

  // Now project all point candidates
  {
    boost::unique_lock<boost::mutex> lock(candidate_points.mut_);
    auto it=candidate_points.candidates_.begin();
    while(it!=candidate_points.candidates_.end())
    {
      if(!reprojectPoint(grid, frame, it->first))
      {
        it->first->n_failed_reproj_ += 3;
        if(it->first->n_failed_reproj_ > 30)
        {
          candidate_points.deleteCandidate(*it);
          it = candidate_points.candidates_.erase(it);
          continue;
        }
      }
      ++it;
    }
  }

  // Now we go through each grid cell and select one point to match.
  // At the end, we should have at maximum one reprojected point per cell.
  n_matches=0; n_trials=0;
  for(size_t i=0; i<grid.cells.size(); ++i)
  {
    // we prefer good quality points over unkown quality (more likely to match)
    // and unknown quality over candidates (position not optimized)
    if(reprojectCell(*grid.cells.at(grid.cell_order[i]), frame, map, candidate_points, n_trials))
      ++n_matches;
    if(n_matches > (size_t) Config::maxFts())
      break;
  }
}

bool pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

bool reprojectCell(
    Cell& cell,
    FramePtr frame,
    Map& map,
    MapPointCandidates& candidate_points,
    size_t& n_trials)
{
  cell.sort(boost::bind(&pointQualityComparator, _1, _2));
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {
    ++n_trials;

    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }

    int feature_level;
    bool found_match = matcher::findMatchDirect(*it->pt, *frame, it->px, feature_level);
    if(!found_match)
    {
      it->pt->n_failed_reproj_++;
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map.safeDeletePoint(it->pt);
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        candidate_points.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }
    it->pt->n_succeeded_reproj_++;
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

    Feature* new_feature = new Feature(frame.get(), it->px, feature_level);
    frame->addFeature(new_feature);
    new_feature->point = it->pt;
    // note, we add a reference to the point only if the frame is selected as a keyframe

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
    it = cell.erase(it);

    // Maximum one point per cell.
    return true;
  }
  return false;
}

bool reprojectPoint(
    Grid& grid,
    FramePtr frame,
    Point* point)
{
  Vector2d px(frame->w2c(point->pos_));
  if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
  {
    const int k = static_cast<int>(px[1]/grid.cell_size)*grid.grid_n_cols + static_cast<int>(px[0]/grid.cell_size);
    grid.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  return false;
}

} // namespace reprojection
} // namespace svo
