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

#include <map>
#include <set>
#include <ros/ros.h>
#include <vikit/user_input_thread.h>
#include <vikit/ringbuffer.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <svo_msgs/Info.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vikit/output_helper.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

class RemoteHandler
{
public:
  ros::NodeHandle& nh_;
  vk::UserInputThread input_thread_;

  ros::Publisher pub_key_;
  ros::Publisher pub_frames_;
  tf::TransformBroadcaster br_;

  vk::RingBuffer<double> acc_timing_;
  vk::RingBuffer<int> acc_obs_;
  typedef std::map<size_t, std::pair<Eigen::Vector3d, Eigen::Matrix3d> > pose_map_t;
  pose_map_t poses_;
  std::set<int> published_kfs_;
  std::set<int> current_kfs_;

  RemoteHandler(ros::NodeHandle& nh);
  ~RemoteHandler();
  void handleUserInput();
  void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg);
  void infoCb(const svo_msgs::InfoConstPtr msg);
};

RemoteHandler::RemoteHandler(ros::NodeHandle& nh) :
    nh_(nh),
    pub_key_(nh_.advertise<std_msgs::String> ("/svo/remote_key", 10)),
    pub_frames_(nh_.advertise<visualization_msgs::Marker>("/svo/vis/frame", 100)),
    acc_timing_(50),
    acc_obs_(50)
{}

RemoteHandler::~RemoteHandler()
{}

void RemoteHandler::handleUserInput()
{
  char key = input_thread_.getInput();
  std_msgs::StringPtr msg(new std_msgs::String);
  switch(key)
  {
    case 'q': msg->data = 'q'; pub_key_.publish(msg); break; // quit
    case 'r': msg->data = 'r'; pub_key_.publish(msg); break; // reset
    case 'p': msg->data = 'p'; pub_key_.publish(msg); break; // pause
    case 's': msg->data = 's'; pub_key_.publish(msg); break; // start
    default: ;
  }
}

void RemoteHandler::poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
{
  Eigen::Vector3d p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix3d R_world_from_frame(q.inverse());
  Eigen::Vector3d p_frame_in_world(-R_world_from_frame*p);

  poses_.insert(std::pair<size_t, std::pair<Eigen::Vector3d, Eigen::Matrix3d> >(
        msg->header.seq,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>(p_frame_in_world, R_world_from_frame)));

  // publish trajectory
  vk::output_helper::publishPointMarker(pub_frames_, p_frame_in_world, "trajectory_points",
                         msg->header.stamp, msg->header.seq, 0, 0.006, Eigen::Vector3d(0.,0.,0.5));

  // publish camera pose tf
  tf::StampedTransform transform_msg;
  transform_msg.frame_id_ = "cam";
  transform_msg.child_frame_id_ = "world";
  transform_msg.stamp_ = msg->header.stamp;
  transform_msg.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
  transform_msg.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  br_.sendTransform(transform_msg);

  // publish keyframes
  std::set<int> deleted_kfs;
  std::set_difference(published_kfs_.begin(), published_kfs_.end(), current_kfs_.begin(), current_kfs_.end(), std::inserter(deleted_kfs, deleted_kfs.end()));
  for(std::set<int>::iterator it=deleted_kfs.begin(); it!=deleted_kfs.end(); ++it)
  {
    pose_map_t::iterator kf = poses_.find(*it);
    if(kf != poses_.end())
      vk::output_helper::publishFrameMarker(pub_frames_, kf->second.second, kf->second.first, "kfs", ros::Time::now(), (*it)*10, 2, 0.015);
  }

  std::set<int> unpublished_kfs;
  std::set_difference(current_kfs_.begin(), current_kfs_.end(), published_kfs_.begin(), published_kfs_.end(), std::inserter(unpublished_kfs, unpublished_kfs.end()));
  for(std::set<int>::iterator it=unpublished_kfs.begin(); it!=unpublished_kfs.end(); ++it)
  {
    pose_map_t::iterator kf = poses_.find(*it);
    if(kf != poses_.end())
      vk::output_helper::publishFrameMarker(pub_frames_, kf->second.second, kf->second.first, "kfs", ros::Time::now(), (*it)*10, 0, 0.015);
  }
  published_kfs_.swap(current_kfs_);


  // publish hexacopter
  vk::output_helper::publishHexacopterMarker(pub_frames_, "cam", "hexacopter", msg->header.stamp, 1, 0, 0.3, Eigen::Vector3d(0.,0.,1.));
}

void RemoteHandler::infoCb(const svo_msgs::InfoConstPtr msg)
{
  std::string kfmsg("");
  current_kfs_.clear();
  for(std::vector<int32_t>::const_iterator it=msg->keyframes.begin(); it!=msg->keyframes.end(); ++it) {
    current_kfs_.insert(static_cast<int>(*it));
    if(*it == static_cast<int32_t>(msg->header.seq)) {
      kfmsg = "Keyframe!";
      break;
    }
  }
  acc_timing_.push_back(msg->processing_time);
  acc_obs_.push_back(msg->num_matches);

  printf("[%i] t=%.2fhz (mean=%.2f) \t n_obs=%i (mean=%i) \t %s\n",
         msg->header.seq, 1./msg->processing_time, 1./acc_timing_.getMean(),
         msg->num_matches, acc_obs_.getMean(),  kfmsg.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo_remote");
  ros::NodeHandle nh;
  RemoteHandler node(nh);

  printf("Listening to User-Input...\n");

  ros::Subscriber sub_pose = nh.subscribe("/svo/pose", 10, &RemoteHandler::poseCb, &node);
  ros::Subscriber sub_info = nh.subscribe("/svo/info", 10, &RemoteHandler::infoCb, &node);
  ros::Rate r(100);
  while (ros::ok())
  {
    node.handleUserInput();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
