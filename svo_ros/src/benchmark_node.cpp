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

#include <vector>
#include <string>
#include <iostream>
#include <sophus/se3.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <vikit/file_reader.h>
#include <vikit/params_helper.h>
#include <vikit/camera_loader.h>
#include <vikit/abstract_camera.h>
#include <vikit/blender_utils.h>
#include <vikit/sample.h>
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/point.h>
#include <svo/frame_handler_mono.h>
#include <svo/feature_detection.h>
#include <svo_ros/visualizer.h>
#include <svo_ros/dataset_img.h>

namespace svo {

class BenchmarkNode
{
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  int frame_count_;
  std::ofstream trace_est_pose_;
  std::ofstream trace_trans_error_;
  std::ofstream trace_rot_error_;
  std::ofstream trace_depth_error_;
  vk::AbstractCamera* cam_;
  double img_noise_sigma_;

public:
  BenchmarkNode(ros::NodeHandle& nh);
  ~BenchmarkNode();
  void tracePose(const SE3& T_w_f, const double timestamp);
  void tracePoseError(const SE3& T_f_gt, const double timestamp);
  void traceDepthError(const FramePtr& frame, const cv::Mat& depthmap);
  void addNoiseToImage(cv::Mat& img, double sigma);
  void runBenchmark(const std::string& dataset_dir);
  void runBlenderBenchmark(const std::string& dataset_dir);
};

BenchmarkNode::BenchmarkNode(ros::NodeHandle& nh) :
    vo_(NULL),
    frame_count_(0),
    img_noise_sigma_(vk::getParam<double>("svo/dataset_noise_sigma", 0.0))
{
  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // create pose tracefile
  std::string trace_est_name(Config::traceDir() + "/traj_estimate.txt");
  trace_est_pose_.open(trace_est_name.c_str());
  if(trace_est_pose_.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");

  // Initialize VO
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::tracePose(const SE3& T_w_f, const double timestamp)
{
  Quaterniond q(T_w_f.unit_quaternion());
  Vector3d p(T_w_f.translation());
  trace_est_pose_.precision(15);
  trace_est_pose_.setf(std::ios::fixed, std::ios::floatfield );
  trace_est_pose_ << timestamp << " ";
  trace_est_pose_.precision(6);
  trace_est_pose_ << p.x() << " " << p.y() << " " << p.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

void BenchmarkNode::tracePoseError(const SE3& T_f_gt, const double timestamp)
{
  Vector3d et(T_f_gt.translation()); // translation error
  trace_trans_error_.precision(15);
  trace_trans_error_.setf(std::ios::fixed, std::ios::floatfield );
  trace_trans_error_ << timestamp << " ";
  trace_trans_error_.precision(6);
  trace_trans_error_ << et.x() << " " << et.y() << " " << et.z() << " " << std::endl;
  Vector3d er(vk::dcm2rpy(T_f_gt.rotation_matrix())); // rotation error in roll-pitch-yaw
  trace_rot_error_.precision(15);
  trace_rot_error_.setf(std::ios::fixed, std::ios::floatfield );
  trace_rot_error_ << timestamp << " ";
  trace_rot_error_.precision(6);
  trace_rot_error_ << er.x() << " " << er.y() << " " << er.z() << " " << std::endl;
}

void BenchmarkNode::traceDepthError(const FramePtr& frame, const cv::Mat& depthmap)
{
  trace_depth_error_.precision(6);
  std::for_each(frame->fts_.begin(), frame->fts_.end(), [&](Feature* ftr){
    if(ftr->point != NULL)
    {
      double depth_estimated = (ftr->point->pos_-frame->pos()).norm();
      double depth_true = depthmap.at<float>((int) ftr->px[1], (int) ftr->px[0]);
      trace_depth_error_ << frame->id_ << " "
                         << depth_estimated-depth_true << std::endl;
    }
  });
}

void BenchmarkNode::runBenchmark(const std::string& dataset_dir)
{
  // create image reader and load dataset
  std::string filename_benchmark(dataset_dir + "/images.txt");
  vk::FileReader<FileType::DatasetImg> dataset_reader(filename_benchmark);
  dataset_reader.skipComments();
  if(!dataset_reader.next()) {
    SVO_ERROR_STREAM("Failed opening dataset: "<<filename_benchmark);
    return;
  }
  std::vector<FileType::DatasetImg> dataset;
  dataset_reader.readAllEntries(dataset);

  // process dataset
  int first_frame = vk::getParam<int>("svo/dataset_first_frame", 0);
  for(auto it = dataset.begin(); it != dataset.end() && ros::ok(); ++it, ++frame_count_)
  {
    if(frame_count_ < first_frame)
      continue;
    std::string img_filename(dataset_dir + "/" + it->image_name_);
    cv::Mat img(cv::imread(img_filename, 0));
    if(img.empty()) {
      SVO_ERROR_STREAM("Reading image "<<img_filename<<" failed.");
      return;
    }
    vo_->addImage(img, it->timestamp_);
    visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, it->timestamp_);
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
    if(vo_->stage() == svo::FrameHandlerMono::STAGE_DEFAULT_FRAME)
      tracePose(vo_->lastFrame()->T_f_w_.inverse(), it->timestamp_);
  }
}

void BenchmarkNode::addNoiseToImage(cv::Mat& img, double sigma)
{
  uint8_t* p = (uint8_t*) img.data;
  uint8_t* p_end = img.ptr<uint8_t>(img.rows, img.cols);
  while(p != p_end)
  {
    int val = *p + vk::Sample::gaussian(sigma) + 0.5;
    *p = std::max(std::min(val, 255), 0);
    ++p;
  }
}

void BenchmarkNode::runBlenderBenchmark(const std::string& dataset_dir)
{
  // create image reader and load dataset
  std::string filename_benchmark(dataset_dir + "/trajectory.txt");
  vk::FileReader<vk::blender_utils::file_format::ImageNameAndPose> dataset_reader(filename_benchmark);
  dataset_reader.skipComments();
  if(!dataset_reader.next()) {
    SVO_ERROR_STREAM("Failed opening dataset: "<<filename_benchmark);
    return;
  }
  std::vector<vk::blender_utils::file_format::ImageNameAndPose> dataset;
  dataset_reader.readAllEntries(dataset);

  // create tracefiles
  trace_trans_error_.open(Config::traceDir() + "/translation_error.txt");
  trace_rot_error_.open(Config::traceDir() + "/orientation_error.txt");
  trace_depth_error_.open(Config::traceDir() + "/depth_error.txt");
  if(trace_trans_error_.fail() || trace_rot_error_.fail() || trace_depth_error_.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");

  // process dataset
  for(auto it = dataset.begin(); it != dataset.end() && ros::ok(); ++it, ++frame_count_)
  {
    // Read image, ground-truth depth-map and ground-truth pose
    std::string img_filename(dataset_dir + "/img/" + it->image_name_ + "_0.png");
    cv::Mat img(cv::imread(img_filename, 0));
    if(img.empty()) {
      SVO_ERROR_STREAM("Reading image "<<img_filename<<" failed.");
      return;
    }
    if(img_noise_sigma_ > 0)
      addNoiseToImage(img, img_noise_sigma_);
    cv::Mat depthmap;
    vk::blender_utils::loadBlenderDepthmap(
        dataset_dir+"/depth/"+it->image_name_+"_0.depth", *cam_, depthmap);
    Sophus::SE3 T_w_gt(it->q_, it->t_);

    // Set reference frame with depth
    if(frame_count_ == 0)
    {
      // set reference frame at ground-truth pose
      FramePtr frame_ref(new Frame(cam_, img, it->timestamp_));
      frame_ref->T_f_w_ = T_w_gt.inverse();

      // extract features, generate features with 3D points
      svo::feature_detection::FastDetector detector(
          cam_->width(), cam_->height(), svo::Config::gridSize(), svo::Config::nPyrLevels());
      detector.detect(frame_ref.get(), frame_ref->img_pyr_, svo::Config::triangMinCornerScore(), frame_ref->fts_);
      std::for_each(frame_ref->fts_.begin(), frame_ref->fts_.end(), [&](Feature* ftr) {
        Eigen::Vector3d pt_pos_cur = ftr->f*depthmap.at<float>(ftr->px[1], ftr->px[0]);
        Eigen::Vector3d pt_pos_world = frame_ref->T_f_w_.inverse()*pt_pos_cur;
        svo::Point* point = new svo::Point(pt_pos_world, ftr);
        ftr->point = point;
      });
      SVO_INFO_STREAM("Added "<<frame_ref->nObs()<<" 3d pts to the reference frame.");
      vo_->setFirstFrame(frame_ref);
      SVO_INFO_STREAM("Set reference frame.");
    }
    else
    {
      SVO_DEBUG_STREAM("Processing image "<<it->image_name_<<".");
      vo_->addImage(img, it->timestamp_);
      visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, it->timestamp_);
      visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
    }

    if(vo_->stage() != svo::FrameHandlerMono::STAGE_DEFAULT_FRAME)
    {
      SVO_ERROR_STREAM("SVO failed before entire dataset could be processed.");
      break;
    }

    // Compute pose error and trace to file
    Sophus::SE3 T_f_gt(vo_->lastFrame()->T_f_w_*T_w_gt);
    tracePoseError(T_f_gt, it->timestamp_);
    tracePose(vo_->lastFrame()->T_f_w_.inverse(), it->timestamp_);
    traceDepthError(vo_->lastFrame(), depthmap);
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  svo::BenchmarkNode benchmark(nh);
  std::string benchmark_dir(vk::getParam<std::string>("svo/dataset_directory"));
  if(vk::getParam<bool>("svo/dataset_is_blender", false))
    benchmark.runBlenderBenchmark(benchmark_dir);
  else
    benchmark.runBenchmark(benchmark_dir);
  printf("BenchmarkNode finished.\n");
  return 0;
}
