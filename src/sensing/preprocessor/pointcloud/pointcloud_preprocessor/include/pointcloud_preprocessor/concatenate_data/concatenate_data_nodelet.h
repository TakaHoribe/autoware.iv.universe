#pragma once

#include <map>
#include <deque>
#include <mutex>

// ROS includes
#include <tf/transform_listener.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>

namespace pointcloud_preprocessor
{

  /** \brief @b PointCloudConcatenateFieldsSynchronizer is a special form of data
    * synchronizer: it listens for a set of input PointCloud messages on the same topic,
    * checks their timestamps, and concatenates their fields together into a single
    * PointCloud output message.
    * \author Radu Bogdan Rusu
    */
  class PointCloudConcatenateDataSynchronizerNodelet: public nodelet_topic_tools::NodeletLazy
  {
    public:
      typedef sensor_msgs::PointCloud2 PointCloud2;
      typedef PointCloud2::Ptr PointCloud2Ptr;
      typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

      /** \brief Empty constructor. */
      PointCloudConcatenateDataSynchronizerNodelet () : maximum_queue_size_ (3) , timeout_sec_(0.03){};

      /** \brief Empty constructor.
        * \param queue_size the maximum queue size
        */
      PointCloudConcatenateDataSynchronizerNodelet (int queue_size) : maximum_queue_size_(queue_size), timeout_sec_(0.03) {};

      /** \brief Empty destructor. */
      virtual ~PointCloudConcatenateDataSynchronizerNodelet () {};

      void onInit ();
      void subscribe ();
      void unsubscribe ();

    private:
      /** \brief The output PointCloud publisher. */
      ros::Publisher pub_output_;

      ros::Publisher pub_concat_num_;
      ros::Publisher pub_not_subscribed_topic_name_;

      /** \brief The maximum number of messages that we can store in the queue. */
      int maximum_queue_size_;

      double timeout_sec_;

      /** \brief A vector of subscriber. */
      std::vector<boost::shared_ptr<ros::Subscriber> > filters_;

      ros::Subscriber sub_twist_;

      ros::Timer timer_;

      /** \brief Output TF frame the concatenated points should be transformed to. */
      std::string output_frame_;

      /** \brief Input point cloud topics. */
      XmlRpc::XmlRpcValue input_topics_;

      /** \brief TF listener object. */
      tf::TransformListener tf_;


      std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_ptr_queue_;

      std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_;
      std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_tmp_;
      std::mutex mutex_;

      void transformPointCloud (const PointCloud2::ConstPtr &in, PointCloud2::Ptr &out);
      void combineClouds (const PointCloud2::ConstPtr &in1, const PointCloud2::ConstPtr &in2, PointCloud2::Ptr &out);
      void publish();

      void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &input_ptr, const std::string &topic_name);
      void twist_callback (const geometry_msgs::TwistStamped::ConstPtr &input);
      void timer_callback(const ros::TimerEvent&);
  };
}
