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
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <image_geometry/pinhole_camera_model.h>

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
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
  void imgCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &info);
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
  if(vk::getParam<bool>("svo/accept_console_user_input", false))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  //if(!vk::camera_loader::loadFromRosNs("svo", cam_))
  //  throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  //vo_ = new svo::FrameHandlerMono(cam_);
  //vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg,
                   const sensor_msgs::CameraInfoConstPtr& info)
{
  ROS_INFO("image callback!!");
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  //  configure camera calibration
  if (!cam_) {
    //  assume these parameters will not change throughout the odometry
    image_geometry::PinholeCameraModel mdl; 
    if (!mdl.fromCameraInfo(info)) {
      ROS_ERROR("Failed to initialize camera info");
      return;
    }
    const cv::Matx33d K = mdl.intrinsicMatrix();
    const cv::Mat_<double> dist = mdl.distortionCoeffs();
    
    cam_ = new vk::PinholeCamera(img.cols, img.rows, 
                                 K(0,0), K(1,1),  //  fx,fy
                                 K(0,2), K(1,2),  //  cx,cy
                                 dist(0), dist(1), dist(2), dist(3), dist(4));
    ROS_INFO("Initialized camera model");
    
    //  now we can initialize the VO
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();
  }
  
  ROS_INFO("Processing frame!");
  
  processUserActions();
  vo_->addImage(img, msg->header.stamp.toSec());
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
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

typedef message_filters::sync_policies::ExactTime<
  sensor_msgs::Image,
  sensor_msgs::CameraInfo> TimeSyncPolicy;

typedef message_filters::Synchronizer<TimeSyncPolicy> Synchronizer;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node;

  // subscribe to cam msgs
  image_transport::ImageTransport it(pnh);
  image_transport::SubscriberFilter sub_image;
  sub_image.subscribe(it, "image", 5);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info;
  sub_info.subscribe(pnh, "camera_info", 5);
  std::shared_ptr<Synchronizer> sync;
  sync.reset( new Synchronizer(100, sub_image, sub_info) );
  sync->registerCallback( boost::bind(&svo::VoNode::imgCb, &vo_node, _1, _2) );
  
//  image_transport::Subscriber it_sub = it.subscribe("image", 5, 
//                                                    &svo::VoNode::imgCb, 
//                                                    &vo_node);

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
