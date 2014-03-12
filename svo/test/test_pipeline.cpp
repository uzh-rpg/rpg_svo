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
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>

namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  Config::traceName() = "benchmark";
  Config::traceDir() =  std::string(TEST_TRACE_DIR)+"/..";
  cam_ = new vk::PinholeCamera(752, 480, 217.083701215, 217.083701215, 376, 240);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
  for(int img_id = 1; img_id < 310; ++img_id)
  {
    // load image
	std::stringstream ss;
    ss << std::string(TEST_DATA_DIR) << "/flying_room_1_rig_1/img/frame_" << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
    if(img_id == 1)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
    		      << "#Features: " << vo_->lastNumObservations() << " \t"
    		      << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

