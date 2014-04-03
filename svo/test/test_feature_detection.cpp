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

#include <string.h>
#include <svo/global.h>
#include <svo/config.h>
#include <svo/feature_detection.h>
#include <svo/depth_filter.h>
#include <vikit/timer.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include "test_utils.h"

namespace {

using namespace Eigen;
using namespace std;

void testCornerDetector()
{
  std::string img_name(svo::test_utils::getDatasetDir() + "/flying_room_1_rig_1/img/frame_000001_0.png");
  printf("Loading image '%s'\n", img_name.c_str());
  cv::Mat img(cv::imread(img_name, 0));
  assert(img.type() == CV_8UC1 && !img.empty());

  svo::ImgPyr img_pyr;
  svo::frame_utils::createImgPyramid(
      img, max(svo::Config::nPyrLevels(), svo::Config::kltMaxLevel()+1), img_pyr);
  svo::Features fts;

  // Corner detection
  vk::Timer t;
  svo::feature_detection::Corners corners;
  svo::feature_detection::FastDetector fast_detector;
  for(int i=0; i<100; ++i)
  {
    fast_detector.detect(img_pyr, fts, svo::Config::gridSize(), svo::Config::nPyrLevels(),
                         svo::Config::triangMinCornerScore(), &corners);
  }
  printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 8.06878ms, 416)\n", t.stop()*10, corners.size());

  if(false)
  {
    cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
    cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
    for(svo::feature_detection::Corners::iterator it=corners.begin(); it!=corners.end(); ++it)
    {
      // we must check again if the score is above the threshold because the Corners
      // vector is initialized with dummy corners
      if(it->score > svo::Config::triangMinCornerScore())
      {
        cv::circle(img_rgb, cv::Point2f(it->x, it->y), 4*(it->level+1), cv::Scalar(0,255,0), 1);
      }
    }
    cv::imshow("ref_img", img_rgb);
    cv::waitKey(0);
  }
}

} // namespace


int main(int argc, char **argv)
{
  testCornerDetector();
  return 0;
}
