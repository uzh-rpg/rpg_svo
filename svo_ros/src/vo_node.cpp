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
#include <svo/frame.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
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

#define ROS_CONVERSIONS
#include <vo_test/error_calculator.hpp>

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
  svo::FramePtr first_frame_;
  svo::FramePtr second_frame_;
  vo::PoseT<Eigen::Matrix3d> first_odom_, second_odom_;
  double rel_scale_;
  vo::test::ErrorCalculator error_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg, 
             const sensor_msgs::CameraInfoConstPtr &info,
             const nav_msgs::OdometryConstPtr& odom);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
  
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false), rel_scale_(-1)
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
                   const sensor_msgs::CameraInfoConstPtr& info,
                   const nav_msgs::OdometryConstPtr& odom)
{
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
  
  processUserActions();
  const FrameHandlerBase::AddImageResult res = 
      vo_->addImage(img, msg->header.stamp.toSec());
  if (res == FrameHandlerBase::RESULT_ADDED_FIRST) {
    first_frame_ = vo_->lastFrame();
    first_odom_ = vo::PoseT<Eigen::Matrix3d>(odom->pose.pose);
  }
  else if (res == FrameHandlerBase::RESULT_ADDED_SECOND) {
    //  done initializing
    second_frame_ = vo_->lastFrame();
    
    //  add second vicon odom frame
    second_odom_ = vo::PoseT<Eigen::Matrix3d>(odom->pose.pose);
    const double vicon_scale = (second_odom_.t() - first_odom_.t()).norm();
    
    //  calculate the relative scale factor
    const Vector3d p0 = first_frame_->pos();
    const Vector3d p1 = second_frame_->pos();
    const double vision_scale = (p1 - p0).norm();
    
    rel_scale_ = vicon_scale / vision_scale;
    ROS_INFO("Added second frame. Relative scale: %f", rel_scale_);
    
    ROS_INFO_STREAM("SVO position 0: " << p0);
    ROS_INFO_STREAM("SVO position 1: " << p1);
    
    ROS_INFO_STREAM("Vicon position 0: " << first_odom_.t());
    ROS_INFO_STREAM("Vicon position 1: " << second_odom_.t());
  } else {
    if (rel_scale_ > 0) {
      //  add to error calculator
      Sophus::SE3 cur = vo_->lastFrame()->T_f_w_.inverse();
      cur = second_frame_->T_f_w_ * cur;
      cur.translation() *= rel_scale_;
      
      ROS_INFO_STREAM("Translation: " << cur.translation());
      
      vo::PoseT<Eigen::Matrix3d> odom_pose(odom->pose.pose);
      odom_pose = second_odom_.inverse().compose(odom_pose);
      
      error_.addFrame(vo::Pose(cur.matrix().template cast<float>()), 
                      vo::Pose(odom_pose));
      
      //  output some error metrics
      auto rms_x = error_.positionStats(0).rms();
      auto rms_y = error_.positionStats(1).rms();
      auto rms_z = error_.positionStats(2).rms();
      auto rms_rx = error_.rotationStats(0).rms() * 180.0/M_PI;
      auto rms_ry = error_.rotationStats(1).rms() * 180.0/M_PI;
      auto rms_rz = error_.rotationStats(2).rms() * 180.0/M_PI;
      auto len = error_.pathLength();
      
      ROS_ERROR_THROTTLE_NAMED(0.5,"path_len", "Path length: %f", len);
      ROS_ERROR_THROTTLE_NAMED(0.5,"position_error_rms",
                                 "RMS position error x/y/z: %f/%f/%f (perc: %f/%f/%f)",
                                  rms_x, rms_y, rms_z, 
                                  rms_x /  len * 100, rms_y / len * 100, rms_z / len * 100);
      ROS_ERROR_THROTTLE_NAMED(0.5,"rotation_error_rms",
                                 "RMS rotation error (deg) x/y/z: %f/%f/%f",
                                  rms_rx, rms_ry, rms_rz);
    }
  }
  
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

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image,
  sensor_msgs::CameraInfo,
  nav_msgs::Odometry> TimeSyncPolicy;

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
  message_filters::Subscriber<nav_msgs::Odometry> sub_vicon;
  sub_vicon.subscribe(pnh, "vicon_odom", 5);
  
  std::shared_ptr<Synchronizer> sync;
  sync.reset( new Synchronizer(300, sub_image, sub_info, sub_vicon) );
  sync->registerCallback( boost::bind(&svo::VoNode::imgCb, &vo_node, 
                                      _1, _2, _3) );
  
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
