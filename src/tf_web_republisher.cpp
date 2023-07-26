/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include <sstream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_web_republisher/RepublishTFs.h>
#include <tf2_web_republisher/TFArray.h>

#include "tf_pair.h"

class TFRepublisher
{
protected:
  typedef tf2_web_republisher::RepublishTFs::Request Request;
  typedef tf2_web_republisher::RepublishTFs::Response Response;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceServer tf_republish_service_;
  ros::Timer checks_timer_;

  // struct for Service client info
  struct ClientRequestInfo
  {
    std::vector<TFPair> tf_subscriptions_;
    unsigned int client_ID_;
    ros::Publisher pub_;
    ros::Duration cycle_time_;
    ros::Duration unsub_timeout_;
    ros::Time last_publish_;
    ros::Time last_valid_subscribers_;
  };

  std::list<ClientRequestInfo> active_requests_;
  boost::mutex requests_mutex_;

  // tf2 buffer and transformer
#if ROS_VERSION_MINOR < 10
  tf2::Buffer tf_buffer_;
  tf2::TransformListener tf_listener_;
#else
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
#endif
  boost::mutex tf_buffer_mutex_;

  unsigned int client_ID_count_;

public:

  TFRepublisher(const std::string& name) :
    nh_(),
    priv_nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    client_ID_count_(0)
  {
    tf_republish_service_ = nh_.advertiseService("republish_tfs",
                                                 &TFRepublisher::requestCB,
                                                 this);
    checks_timer_ = nh_.createTimer(ros::Duration(0.01), &TFRepublisher::performChecks, this);
  }

  ~TFRepublisher() {}


  const std::string cleanTfFrame( const std::string frame_id ) const
  {
    if ( frame_id[0] == '/' )
    {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  /**
   * Set up the contents of \p tf_subscriptions_ in
   * a ClientInfo struct
   */
  void setSubscriptions(ClientRequestInfo &info,
                        const std::vector<std::string>& source_frames,
                        const std::string& target_frame_,
                        float angular_thres,
                        float trans_thres) const
  {
    std::size_t request_size_ = source_frames.size();
    info.tf_subscriptions_.resize(request_size_);

    for (std::size_t i=0; i<request_size_; ++i )
    {
      TFPair& tf_pair = info.tf_subscriptions_[i];

      std::string source_frame = cleanTfFrame(source_frames[i]);
      std::string target_frame = cleanTfFrame(target_frame_);

      tf_pair.setSourceFrame(source_frame);
      tf_pair.setTargetFrame(target_frame);
      tf_pair.setAngularThres(angular_thres);
      tf_pair.setTransThres(trans_thres);
    }
  }

  bool requestCB(Request& req, Response& res)
  {
    ROS_DEBUG("RepublishTF service request received");
    // generate request_info struct
    ClientRequestInfo request_info;

    request_info.client_ID_ = client_ID_count_;
    std::stringstream topicname;
    topicname << "tf_repub_" << client_ID_count_++;

    request_info.pub_ = priv_nh_.advertise<tf2_web_republisher::TFArray>(topicname.str(), 10, true);

    // add the tf_subscriptions to the ClientGoalInfo object
    setSubscriptions(request_info,
                     req.source_frames,
                     req.target_frame,
                     req.angular_thres,
                     req.trans_thres);

    request_info.unsub_timeout_ = req.timeout;
    request_info.cycle_time_ = ros::Duration(1.0 / req.rate);
    request_info.last_valid_subscribers_ = ros::Time::now();
    request_info.last_publish_ = ros::Time::now();

    {
      boost::mutex::scoped_lock l(requests_mutex_);
      // add new request to list of active requests
      active_requests_.push_back(request_info);
    }

    res.topic_name = request_info.pub_.getTopic();
    ROS_INFO_STREAM("Publishing requested TFs on topic " << res.topic_name);
    return true;
  }

  void updateSubscriptions(std::vector<TFPair>& tf_subscriptions,
                           std::vector<geometry_msgs::TransformStamped>& transforms)
  {
    // iterate over tf_subscription vector
    std::vector<TFPair>::iterator it ;
    std::vector<TFPair>::const_iterator end = tf_subscriptions.end();

    for (it=tf_subscriptions.begin(); it!=end; ++it)
    {
      geometry_msgs::TransformStamped transform;

      try
      {
        // protecting tf_buffer
        boost::mutex::scoped_lock lock (tf_buffer_mutex_);

        // lookup transformation for tf_pair
        transform = tf_buffer_.lookupTransform(it->getTargetFrame(),
                                               it->getSourceFrame(),
                                               ros::Time(0));

        // If the transform broke earlier, but worked now (we didn't get
        // booted into the catch block), tell the user all is well again
        if (!it->is_okay)
        {
          it->is_okay = true;
          ROS_INFO_STREAM("Transform from "
                          << it->getSourceFrame()
                          << " to "
                          << it->getTargetFrame()
                          << " is working again at time "
                          << transform.header.stamp.toSec());
        }
        // update tf_pair with transformtion
        it->updateTransform(transform);
      }
      catch (tf2::TransformException ex)
      {
        // Only log an error if the transform was okay before
        if (it->is_okay)
        {
          it->is_okay = false;
          ROS_ERROR("%s", ex.what());
        }
      }

      // check angular and translational thresholds
      if (it->updateNeeded())
      {
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = it->getTargetFrame();
        transform.child_frame_id = it->getSourceFrame();

        // notify tf_subscription that a network transmission has been triggered
        it->transmissionTriggered();

        // add transform to the array
        transforms.push_back(transform);
      }
    }
  }

  void performChecks(const ros::TimerEvent& event){
    // check all active requests for subscriber expiration and publishing
    {
      boost::mutex::scoped_lock l(requests_mutex_);
      std::list<ClientRequestInfo>::iterator it = active_requests_.begin();
      while(it != active_requests_.end()) {

        if (it->pub_.getNumSubscribers() > 0)
        {
          it->last_valid_subscribers_ = ros::Time::now();
        }

        if ((ros::Time::now() - it->last_valid_subscribers_) >= it->unsub_timeout_){
          // Stop the publisher from the list and remove the request from the list
          ROS_INFO_STREAM("No subscribers on tf topic for request "
                     << it->client_ID_
                     << " for " << it->unsub_timeout_.toSec()
                     << " seconds. Unadvertising topic:"
                     << it->pub_.getTopic());

          it->pub_.shutdown();

          // search for ClientRequestInfo struct and remove it from active_requests_ list
          it = active_requests_.erase(it);
        } else {
          if ((ros::Time::now() - it->last_publish_) >= it->cycle_time_){
            tf2_web_republisher::TFArray array_msg;
            updateSubscriptions(it->tf_subscriptions_, array_msg.transforms);

            if (array_msg.transforms.size() > 0) {
              // publish TFs
              it->pub_.publish(array_msg);
              ROS_DEBUG("Request %d: TFs published:", it->client_ID_);
            } else {
              ROS_DEBUG("Request %d: No TF frame update needed:", it->client_ID_);
            }

            it->last_publish_ = ros::Time::now();
          }

          it++;
        }
      }
    }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf2_web_republisher");

  TFRepublisher tf2_web_republisher(ros::this_node::getName());
  ros::spin();

  return 0;
}
