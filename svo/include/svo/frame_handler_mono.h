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

#ifndef SVO_FRAME_HANDLER_H_
#define SVO_FRAME_HANDLER_H_

#include <vikit/abstract_camera.h>
#include <svo/frame_handler_base.h>
#include <svo/reprojection.h>
#include <svo/initialization.h>

namespace svo {

/// Monocular Visual Odometry Pipeline as described in the SVO paper.
class FrameHandlerMono : public FrameHandlerBase
{
public:
  FrameHandlerMono(vk::AbstractCamera* cam);
  ~FrameHandlerMono();

  /// Provide an image.
  void addImage(const cv::Mat& img, double timestamp);

  /// Get the last frame that has been processed.
  FramePtr lastFrame() { return last_frame_; }

  /// Get the set of spatially closest keyframes of the last frame.
  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  /// Return the feature track to visualize the KLT tracking during initialization.
  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

protected:
  vk::AbstractCamera* cam_;                     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).
  reprojection::Grid reprojection_grid_;        //!< Grid used for reprojection to make sure we have evenly distributed features.
  FramePtr new_frame_;                          //!< Current frame.
  FramePtr last_frame_;                         //!< Last frame, not necessarily a keyframe.
  set<FramePtr> core_kfs_;                      //!< Keyframes in the closer neighbourhood.
  vector< pair<FramePtr,size_t> > overlap_kfs_; //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
  initialization::KltHomographyInit klt_homography_init_; //!< Used to estimate pose of the first two keyframes by estimating a homography.

  /// Processes the first frame and sets it as a keyframe.
  UpdateResult processFirstFrame();

  /// Processes all frames after the first frame until a keyframe is selected.
  UpdateResult processSecondFrame();

  /// Processes all frames after the first two keyframes.
  UpdateResult processFrame();

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll();

  void setCoreKfs(size_t n_closest);

  bool needNewKf(double scene_depth_mean);
};

} // namespace svo

#endif // SVO_FRAME_HANDLER_H_
