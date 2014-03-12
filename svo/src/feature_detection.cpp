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

#include <svo/feature_detection.h>
#include <svo/feature.h>
#include <fast/fast.h>
#include <vikit/vision.h>

namespace svo {
namespace feature_detection {

void FastDetector::detect(
    const ImgPyr& img_pyr,
    const Features& existing_fts,
    const int cell_size,
    const int n_levels,
    const double detection_threshold,
    Corners* corners) const
{
  // first, find which parts of the image already have a matched feature
  const int grid_n_cols = ceil(static_cast<double>(img_pyr[0].cols)/cell_size);
  const int grid_n_rows = ceil(static_cast<double>(img_pyr[0].rows)/cell_size);
  vector<bool> occupancy(grid_n_cols*grid_n_rows, false);
  for(Features::const_iterator it=existing_fts.begin(); it!=existing_fts.end(); ++it)
  {
    if((*it)->point == false)
      continue;
    const int k = static_cast<int>((*it)->px[1]/cell_size)*grid_n_cols + static_cast<int>((*it)->px[0]/cell_size);
    occupancy.at(k) = true;
  }

  // now, detect features and perform nonmax suppression
  corners->resize(grid_n_cols*grid_n_rows, Corner(0,0,detection_threshold,0,0.0f));
  for(int L=0; L<n_levels; ++L)
  {
    const int scale = (1<<L);
    vector<fast::fast_xy> fast_corners;
#if __SSE2__
      fast::fast_corner_detect_10_sse2((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#elif HAVE_FAST_NEON
      fast::fast_corner_detect_9_neon((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
      fast::fast_corner_detect_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    for(vector<int>::iterator it=nm_corners.begin(); it!=nm_corners.end(); ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      const int k = static_cast<int>((xy.y*scale)/cell_size)*grid_n_cols + static_cast<int>((xy.x*scale)/cell_size);
      if(occupancy[k])
        continue;
      const float score = vk::shiTomasiScore(img_pyr[L], xy.x, xy.y);
      if(score > corners->at(k).score)
        corners->at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
    }
  }
}

} // namespace feature_detection
} // namespace svo

