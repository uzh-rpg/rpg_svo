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
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

#include <geometry_msgs/TransformStamped.h>//#####################################3
#include <std_msgs/Bool.h>//#####################
#include <math.h>

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
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
  void resetCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin);//############################
  
  ros::Subscriber sub_quality_;
  void qualityCallback(const geometry_msgs::Vector3& msgin); // receive message from active slam
  
  ros::Publisher pub_usereset_;
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
  visualizer_.camera_facing_ = vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
						    vk::getParam<double>("svo/init_ry", 0.0),
						    vk::getParam<double>("svo/init_rz", 0.0))); // #############log the camera setup (downward facing)
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      visualizer_.camera_facing_,
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));


  // Init VO and start
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
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
      pub_usereset_.publish(1);
      //vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

// reset call back here!!####
void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

//______________________________###############################################
// Filter reset callback function
//void VoNode::resetCallback(const std_msgs::Bool::ConstPtr& msgin)
void VoNode::resetCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin)
{
  visualizer_.T_world_from_vision_ = Sophus::SE3(
	vk::quat2dcm(Vector4d((double)msgin->transform.rotation.w,
			      (double)msgin->transform.rotation.x,
			      (double)msgin->transform.rotation.y,
                              (double)msgin->transform.rotation.z)) * visualizer_.camera_facing_,
	Eigen::Vector3d((double)msgin->transform.translation.x,
                        (double)msgin->transform.translation.y,
                        (double)msgin->transform.translation.z));
  vo_->start();
  printf("SVO user input: START\n");
}

//__________________________________#####################
// active slam quality message
void VoNode::qualityCallback(const geometry_msgs::Vector3& msgin)
{
  if (msgin.x > 0.001) return;
  visualizer_.quality_reading_ = msgin.x*700.0;
  std::cout << "The scaled quality is" << visualizer_.quality_reading_ << std::endl;
}

} // namespace svo



//_________________________
// MIAN
//_____________________________
int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 1, &svo::VoNode::imgCb, &vo_node);

  // subscribe to remote input#######################################################
  //vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);
  vo_node.sub_remote_key_ = nh.subscribe("Allreset", 10, &svo::VoNode::resetCallback, &vo_node);
  vo_node.pub_usereset_   = nh.advertise<std_msgs::Bool>("svo/usereset", 10);;
  vo_node.sub_quality_    = nh.subscribe("aslam/quality", 100, &svo::VoNode::qualityCallback, &vo_node);
  
  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}
