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

#ifndef SVO_FRAME_HANDLER_BASE_H_
#define SVO_FRAME_HANDLER_BASE_H_

#include <queue>
#include <vikit/timer.h>
#include <vikit/ringbuffer.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <svo/global.h>
#include <svo/map.h>

namespace vk {
  class AbstractCamera;
  class PerformanceMonitor;
}

namespace svo {

class Point;
class Matcher;
class DepthFilter;

/// Base class for various VO pipelines. Manages the map and the state machine.
class FrameHandlerBase : boost::noncopyable
{
public:
  enum Stage { PAUSED, FIRST_FRAME, SECOND_FRAME, DEFAULT_FRAME };
  enum TrackingQuality { INSUFFICIENT, BAD, GOOD };
  enum UpdateResult { NO_KEYFRAME, IS_KEYFRAME, FAILURE };

  FrameHandlerBase();

  virtual ~FrameHandlerBase();

  /// Set initial pose of the camera.
  void setInitialPose(const SE3& T_frame_from_world) { T_f_w_init_ = T_frame_from_world; }

  /// Get the current map.
  const Map& map() const { return map_; }

  /// Will reset the map as soon as the current frame is finished processing.
  void reset() { set_reset_ = true; }

  /// Start processing.
  void start() { set_start_ = true; }

  /// Get the current stage of the algorithm.
  Stage stage() const { return stage_; }

  /// Get tracking quality.
  TrackingQuality trackingQuality() const { return tracking_quality_; }

  /// Get the processing time of the previous iteration.
  double lastProcessingTime() const { return timer_.getTime(); }

  /// Get the number of feature observations of the last frame.
  size_t lastNumObservations() const { return num_obs_last_; }

  /// Access the depth filter.
  DepthFilter* depthFilter() const { return depth_filter_; }

protected:
  Stage stage_;                 //!< Current stage of the algorithm.
  bool set_reset_;              //!< Flag that the user can set. Will reset the system before the next iteration.
  bool set_start_;              //!< Flag the user can set to start the system when the next image is received.
  Map map_;                     //!< Map of keyframes created by the slam system.
  SE3 T_f_w_init_;              //!< Initial pose of the camera.
  vk::Timer timer_;             //!< Stopwatch to measure time to process frame.
  vk::RingBuffer<double> acc_frame_timings_;    //!< Total processing time of the last 10 frames, used to give some user feedback on the performance.
  vk::RingBuffer<size_t> acc_num_obs_;          //!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.
  size_t num_obs_last_;                         //!< Number of observations in the previous frame.
  TrackingQuality tracking_quality_;            //!< An estimate of the tracking quality based on the number of tracked features.
  DepthFilter* depth_filter_;                   //!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.

  /// Before a frame is processed, this function is called.
  bool startFrameProcessingCommon(const double timestamp);

  /// When a frame is finished processing, this function is called.
  int finishFrameProcessingCommon(
      const size_t update_id,
      const UpdateResult dropout,
      const size_t num_observations);

  /// Reset the map and frame handler to start from scratch.
  void resetCommon();

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll() { resetCommon(); }

  /// Set the tracking quality based on the number of tracked features.
  void setTrackingQuality(const size_t num_observations);

  /// Optimize some of the observed 3D points.
  void optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter);
};

} // namespace nslam

#endif // SVO_FRAME_HANDLER_BASE_H_
