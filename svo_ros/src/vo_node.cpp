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
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <vs_common/vs_common.h>

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerStereo* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  VoNode();
  ~VoNode();
  void processImgLR(const cv::Mat& imgl, const cv::Mat& imgr, double timestamp);
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  vo_ = new svo::FrameHandlerStereo(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::processImgLR(const cv::Mat& imgl, const cv::Mat& imgr, double timestamp)
{
  processUserActions();
  vo_->addImage(imgl, imgr, timestamp);
  visualizer_.publishMinimal(imgl, vo_->lastFrame(), *vo_, timestamp);

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerStereo::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

#define PLAY_VIDEO 1

#if PLAY_VIDEO
#include "dataset.h"
int video_demo(svo::VoNode* vo_node)
{
  cv::VideoCapture cap(video_file);
  if(!cap.isOpened())
  {
      printf("[ERROR] read source video failed. file '%s' not exists.\n", video_file.c_str());
      return 0;
  }
  int n_img = cap.get(CV_CAP_PROP_FRAME_COUNT);
  printf("load %d images from video %s\n", n_img, video_file.c_str());
  std::ifstream fin_imgts(video_ts_file);
  if(!fin_imgts.is_open())
  {
      printf("[ERROR] stereo_video_play: cannot open file %s\n", video_ts_file.c_str());
      return -1;
  }
  usleep(3000000);

  std::vector<float>  frame_costs;
  cv::Mat image;
  double tframe;
  for(int i=0; i<n_img; i++)
  {
      cap.read(image);
      fin_imgts>>tframe;
      // if(i<100) continue;

      printf("process frame %d\n", i);
      if(image.channels()==3)
          cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);

      int r = image.rows/2;
      cv::Mat imgl = image.rowRange(0,r);
      cv::Mat imgr = image.rowRange(r,r*2);

      tic("process");
      vo_node->processImgLR(imgl, imgr, tframe);
      float cost_ms = toc("process");
      frame_costs.push_back(cost_ms);

      cv::Mat img_draw;
      cv::cvtColor(image, img_draw, cv::COLOR_GRAY2BGR);
      char text[30] = {0};
      sprintf(text, "%6.1f fps", 1000.0f/vecMean(subvec(frame_costs, std::max(0,(int)frame_costs.size()-30))));
      cv::putText(img_draw, text, cv::Point(image.cols-90, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);
      cv::imshow("image",img_draw);
      char key = cv::waitKey(100);
      if(key==27) {break;}
  }
  return 0;
}
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
  svo::VoNode vo_node;

  // subscribe to cam msgs
#if PLAY_VIDEO
  std::thread th_play_video(video_demo, &vo_node);
#else
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);
#endif

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}

