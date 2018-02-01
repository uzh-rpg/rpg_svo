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

#include <svo/config.h>
#include <svo/frame_handler_stereo.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <svo/depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

#define DBG_REPRJ_ERR(INFO) /*{ printf("%s: ", INFO); \
        for(int i=0;i<new_frames_->size();i++) printf("%f ", calcReprojectError(new_frames_->at(i))); \
        printf("\n");}*/

FrameHandlerStereo::FrameHandlerStereo(vk::AbstractCamera* cam) :
  FrameHandlerBase(),
  cam_(cam),
  reprojector_(cam_, map_),
  depth_filter_(NULL)
{
  initialize();
}

void FrameHandlerStereo::initialize()
{
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->startThread();
}

FrameHandlerStereo::~FrameHandlerStereo()
{
  delete depth_filter_;
}

void FrameHandlerStereo::addImage(const cv::Mat& img_left, const cv::Mat& img_right, const double timestamp)
{
  if(!startFrameProcessingCommon(timestamp))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();
  overlap_kfs_.clear();

  Matrix3d R_cam_body;
  R_cam_body<<0,1,0,0,0,1,1,0,0;
  Vector3d t_cam_body;
  t_cam_body<<0,0,0;
  Vector3d t_cam_body_right;
  t_cam_body_right<<-0.11944,0,0;
#if 1
  // create new frame
  SVO_START_TIMER("pyramid_creation");
  new_frames_.reset(new FrameBundle(std::vector<FramePtr>({
                      FramePtr(new Frame(cam_, img_left.clone(), timestamp)),
                      FramePtr(new Frame(cam_, img_right.clone(), timestamp))
                    })));
  new_frames_->at(0)->set_T_cam_body(SE3(R_cam_body, t_cam_body));
  new_frames_->at(1)->set_T_cam_body(SE3(R_cam_body, t_cam_body_right));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstFrame();

  // set last frame
  last_frames_ = new_frames_;
  new_frames_.reset();
#else
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_FIRST_FRAME)
  {
    new_frames_.reset(new FrameBundle(std::vector<FramePtr>({
                      FramePtr(new Frame(cam_, img_left.clone(), timestamp)),
                      FramePtr(new Frame(cam_, img_right.clone(), timestamp))
                    })));
    new_frames_->at(0)->set_T_cam_body(SE3(R_cam_body, t_cam_body));
    new_frames_->at(1)->set_T_cam_body(SE3(R_cam_body, t_cam_body_right));
    res = processFirstFrame();
    last_frames_.reset(new FrameBundle(std::vector<FramePtr>(1,new_frames_->at(0))));
    new_frames_.reset();
  }
  else if(stage_ == STAGE_DEFAULT_FRAME)
  {
    new_frames_.reset(new FrameBundle(std::vector<FramePtr>(1,FramePtr(new Frame(cam_, img_left.clone(), timestamp)))));
    new_frames_->at(0)->set_T_cam_body(SE3(R_cam_body, t_cam_body));
    res = processFrame();
    last_frames_ = new_frames_;
    new_frames_.reset();
  }
#endif
  history_frames_.push_back(last_frames_);
  if(history_frames_.size()>10)
    history_frames_.pop_front();
  // finish processing
  finishFrameProcessingCommon(last_frames_->getBundleId(), res, last_frames_->numFeatures());
}

FrameHandlerStereo::UpdateResult FrameHandlerStereo::processFirstFrame()
{
  // new_frames_->set_T_W_B(new_frames_->at(0)->T_cam_imu());
  new_frames_->set_T_W_B(SE3(Matrix3d::Identity(), Vector3d::Zero()));
  
  initialization::InitResult res = initialization::initFrameStereo(new_frames_->at(0),new_frames_->at(1));
  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;

  new_frames_->setKeyframe();
  for(size_t i=0; i<new_frames_->size(); i++)
  {
    FramePtr f = new_frames_->at(i);
    if(i==0)
    {
      double depth_mean, depth_min;
      frame_utils::getSceneDepth(*f, depth_mean, depth_min);
      depth_filter_->addKeyframe(f, depth_mean, 0.5*depth_min);
    }
    map_.addKeyframe(f);
  }
  stage_ = STAGE_DEFAULT_FRAME;
  return RESULT_IS_KEYFRAME;
}

float calcReprojectError(const FramePtr& frame)
{
  float sum = 0;
  int n = 0;
  for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point == NULL)
        continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    sum += e.norm();
    n++;
  }
  if(n == 0) return 0;
  return sum/(float)n*frame->cam_->errorMultiplier2();
}

FrameHandlerBase::UpdateResult FrameHandlerStereo::processFrame()
{
  // Set initial pose TODO use prior
  new_frames_->set_T_W_B(last_frames_->get_T_W_B());
  // a. sparse image align
  // 当前帧与上一帧直接法粗匹配，利用上一帧的带深度的特征点patch
  SVO_START_TIMER("sparse_img_align");
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(last_frames_, new_frames_);
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // b. map reprojection & feature alignment
  // 将相邻关键帧中所有的点投影到当前图像，并利用与参考帧的像素误差调整特征点的位置
  SVO_START_TIMER("reproject");
  size_t repr_n_new_references = 0;
  size_t repr_n_mps = 0;
  for(size_t i=0; i<new_frames_->size(); i++)
  {
    vector< pair<FramePtr,size_t> > overlap_kfs;
    reprojector_.reprojectMap(new_frames_->at(i), overlap_kfs);
    if(i==0)
      overlap_kfs_ = overlap_kfs;
    else
      overlap_kfs_.insert(overlap_kfs_.end(),overlap_kfs.begin(),overlap_kfs.end());
    repr_n_new_references += reprojector_.n_matches_;
    repr_n_mps += reprojector_.n_trials_;
  }
  SVO_STOP_TIMER("reproject");
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  if(1) //the later frame may delete some candidates which the prev frame used
  {
    std::set<Point*> trash_pts;
    std::for_each(map_.point_candidates_.trash_points_.begin(), map_.point_candidates_.trash_points_.end(), [&](Point* pt){
      trash_pts.insert(pt);
    });
    std::for_each(map_.trash_points_.begin(), map_.trash_points_.end(), [&](Point* pt){
      trash_pts.insert(pt);
    });
    std::for_each(new_frames_->begin(), new_frames_->end(), [trash_pts](FramePtr f){
      std::for_each(f->fts_.begin(), f->fts_.end(), [trash_pts](Feature* ftr){
        if(trash_pts.count(ftr->point))
        {
          ftr->point = NULL;
        }
      });
    });
  }
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
    new_frames_->set_T_W_B(last_frames_->get_T_W_B()); // reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;
  }

  // c. pose and point optimizaiton respectively
  // 根据投影的特征点，优化当前帧的pose(优化的重投影误差)，然后再根据pose优化特征点，分开优化是为了加速
  // pose optimization
  SVO_START_TIMER("pose_optimizer");
  size_t sfba_n_edges_final;
  double sfba_thresh, sfba_error_init, sfba_error_final;
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter()*2, false,
      new_frames_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frames_, Config::structureOptimMaxPts()*2, Config::structureOptimNumIter()*2);
  SVO_STOP_TIMER("point_optimizer");

  // d. select keyframe
  // 判断当前帧是否为关键帧，如果不是关键帧则退出，如果是，则增加关键帧，并提取新的特征点加到深度滤波器中
  // 关键帧选取的逻辑非常naive：直接用的距离判断，距离阈值跟帧平均深度正相关
  for(auto it = new_frames_->begin(); it != new_frames_->end(); it++)
  { core_kfs_.insert(*it); }
  setTrackingQuality(sfba_n_edges_final);
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frames_->set_T_W_B(last_frames_->get_T_W_B());
    return RESULT_FAILURE;
  }
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frames_->at(0), depth_mean, depth_min);
  SVO_DEBUG_STREAM("Frame Depth:\t depth_mean = "<<depth_mean<<"\t depth_min = "<<depth_min);
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    for(size_t i=0; i<new_frames_->size(); i++)
      depth_filter_->addFrame(new_frames_->at(i));
    return RESULT_NO_KEYFRAME;
  }

  // e. 新增关键帧
  new_frames_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  for(size_t i=0; i<new_frames_->size(); i++)
  {
    FramePtr frame = new_frames_->at(i);
    for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
      if((*it)->point != NULL)
        (*it)->point->addFrameRef(*it);
    map_.point_candidates_.addCandidatePointToFrame(frame);
  }
  // init new depth-filters
  {
    std::vector<FramePtr> update_frames;
    for(auto & fb: history_frames_)
    {
      update_frames.insert(update_frames.begin(),fb->frames_.begin(),fb->frames_.end());
    }
    for(size_t i=1; i<new_frames_->size(); i++)
    {
      update_frames.push_back(new_frames_->at(i));
    }
    depth_filter_->addKeyframe(new_frames_->at(0), depth_mean, 0.5*depth_min, update_frames);
  }

  // if limited number of keyframes, remove the one furthest apart
  while(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frames_->imuPos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  for(size_t i=0; i<new_frames_->size(); i++)
    map_.addKeyframe(new_frames_->at(i));

  return RESULT_IS_KEYFRAME;
}

FrameHandlerStereo::UpdateResult FrameHandlerStereo::relocalizeFrame(
    const SE3& T_cur_ref,
    FramePtr ref_keyframe)
{
  return RESULT_FAILURE;
}

bool FrameHandlerStereo::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  return false;
}

void FrameHandlerStereo::resetAll()
{
  resetCommon();
  last_frames_.reset();
  new_frames_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  depth_filter_->reset();
}

bool FrameHandlerStereo::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
    Vector3d relpos = new_frames_->at(0)->w2f(it->first->pos());
    // printf("delta:(%.2f %.2f %.2f) scene depth:%.2f\n", relpos.x(),relpos.y(),relpos.z());
    if(fabs(relpos.x()) < Config::kfSelectMinDist() &&
       fabs(relpos.y()) < Config::kfSelectMinDist() &&
       fabs(relpos.z()) < Config::kfSelectMinDist())
      return false;
  }
  return true;
}

void FrameHandlerStereo::setCoreKfs(size_t n_closest)
{
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

} // namespace svo
