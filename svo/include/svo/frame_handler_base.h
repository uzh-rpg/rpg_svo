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

namespace vk
{
class AbstractCamera;
class PerformanceMonitor;
}

namespace svo
{
class Point;
class Matcher;
class DepthFilter;

/// Base class for various VO pipelines. Manages the map and the state machine.
class FrameHandlerBase : boost::noncopyable
{
public:
  enum Stage {
    STAGE_PAUSED,
    STAGE_FIRST_FRAME,
    STAGE_SECOND_FRAME,
    STAGE_DEFAULT_FRAME,
    STAGE_RELOCALIZING
  };
  enum TrackingQuality {
    TRACKING_INSUFFICIENT,
    TRACKING_BAD,
    TRACKING_GOOD
  };
  enum UpdateResult {
    RESULT_NO_KEYFRAME,
    RESULT_IS_KEYFRAME,
    RESULT_FAILURE
  };

  FrameHandlerBase();

  virtual ~FrameHandlerBase();

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

protected:
  Stage stage_;                 //!< Current stage of the algorithm.
  bool set_reset_;              //!< Flag that the user can set. Will reset the system before the next iteration.
  bool set_start_;              //!< Flag the user can set to start the system when the next image is received.
  Map map_;                     //!< Map of keyframes created by the slam system.
  vk::Timer timer_;             //!< Stopwatch to measure time to process frame.
  vk::RingBuffer<double> acc_frame_timings_;    //!< Total processing time of the last 10 frames, used to give some user feedback on the performance.
  vk::RingBuffer<size_t> acc_num_obs_;          //!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.
  size_t num_obs_last_;                         //!< Number of observations in the previous frame.
  TrackingQuality tracking_quality_;            //!< An estimate of the tracking quality based on the number of tracked features.

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
  virtual void setTrackingQuality(const size_t num_observations);

  /// Optimize some of the observed 3D points.
  virtual void optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter);
};

} // namespace nslam

#endif // SVO_FRAME_HANDLER_BASE_H_
