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
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/map.h>
#include <svo/frame_handler_mono.h>
#include <svo_ros/visualizer.h>
#include <svo_ros/dataset_img.h>

namespace svo {

class BenchmarkNode
{
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  int frame_count_;
  std::ofstream trace_est_pose_;
  vk::AbstractCamera* cam_;

public:
  BenchmarkNode(ros::NodeHandle& nh);
  ~BenchmarkNode();
  void runCamImuBenchmark();
  void processUserActions();
};

BenchmarkNode::BenchmarkNode(ros::NodeHandle& nh) :
    vo_(NULL),
    frame_count_(0)
{
  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Initialize VO
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runCamImuBenchmark()
{
  // create image reader and load dataset
  std::string benchmark_dir(vk::getParam<std::string>("svo/dataset_directory"));
  std::string filename_benchmark(benchmark_dir + "/images.txt");
  vk::FileReader<FileType::DatasetImg> dataset_reader(filename_benchmark);
  dataset_reader.skipComments();
  if(!dataset_reader.next())
    std::runtime_error("Failed to open images file");
  std::vector<FileType::DatasetImg> dataset;
  dataset_reader.readAllEntries(dataset);

  // create pose tracefile
  std::string trace_est_name(Config::traceDir() + "/traj_estimate.txt");
  trace_est_pose_.open(trace_est_name.c_str());
  if(trace_est_pose_.fail())
    throw std::runtime_error("ERROR:frame_count_ could not create tracefile. probably the specified folder does not exist.");

  // process dataset
  for(vk::vector<FileType::DatasetImg>::iterator it = dataset.begin();
      it != dataset.end(); ++it, ++frame_count_)
  {
    // Read image
    std::string img_filename(benchmark_dir + "/" + it->imgname_);
    cv::Mat img(cv::imread(img_filename, 0));
    if(frame_count_ == 0)
      SVO_INFO_STREAM("Reading image "<<img_filename);
    if(img.empty())
      throw std::runtime_error("Image could not be loaded.");

    // Add image to VO
    vo_->addImage(img, it->stamp_);

    // Visualize
    visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, it->stamp_);

    // write pose to tracefile
    if(vo_->stage() == svo::FrameHandlerMono::DEFAULT_FRAME)
    {
      Quaterniond q(vo_->lastFrame()->T_f_w_.inverse().unit_quaternion());
      Vector3d p(vo_->lastFrame()->T_f_w_.inverse().translation());
      trace_est_pose_.precision(15);
      trace_est_pose_.setf(std::ios::fixed, std::ios::floatfield );
      trace_est_pose_ << it->stamp_<< " ";
      trace_est_pose_.precision(6);
      trace_est_pose_ << p.x() << " " << p.y() << " " << p.z() << " "
                      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    if (!ros::ok())
      break;
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  {
    svo::BenchmarkNode benchmark(nh);
    benchmark.runCamImuBenchmark();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}
