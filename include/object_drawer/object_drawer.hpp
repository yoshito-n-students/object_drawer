#ifndef OBEJCT_DRAWER_OBJECT_DRAWER_HPP
#define OBJECT_DRAWER_OBJECT_DRAWER_HPP

#include <stdexcept>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

namespace object_drawer {

class ObjectDrawer : public nodelet::Nodelet {
private:
  typedef message_filters::TimeSynchronizer< sensor_msgs::Image, object_detection_msgs::Objects >
      SyncSubscriber;

public:
  ObjectDrawer() {}
  virtual ~ObjectDrawer() {}

  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load params
    const int queue_size(pnh.param("queue_size", 10));
    line_tickness_ = pnh.param("line_tickness", 3);
    text_tickness_ = pnh.param("text_tickness", 2);
    font_scale_ = pnh.param("font_scale", 0.8);
    // TODO: line & text colors as parameters

    // annotated image publisher
    image_transport::ImageTransport it(nh);
    image_publisher_ = it.advertise("image_out", 1, true);

    // detection result subscribers
    image_subscriber_.subscribe(it, "image_raw", 1);
    object_subscriber_.subscribe(nh, "objects_in", 1);

    // callback on synchronized results
    sync_subscriber_.reset(new SyncSubscriber(queue_size));
    sync_subscriber_->connectInput(image_subscriber_, object_subscriber_);
    sync_subscriber_->registerCallback(&ObjectDrawer::drawObjectsOnImage, this);
  }

private:
  void drawObjectsOnImage(const sensor_msgs::ImageConstPtr &image_msg,
                          const object_detection_msgs::ObjectsConstPtr &object_msg) {
    try {
      // process the received message on demand
      if (image_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // received message to opencv image
      // (desired encoding of opencv's drawering functions is bgr8)
      const cv_bridge::CvImagePtr image(cv_bridge::toCvCopy(image_msg, "bgr8"));
      if (!image) {
        NODELET_ERROR("Image conversion error");
        return;
      }
      if (image->image.empty()) {
        NODELET_ERROR("Empty image message");
        return;
      }

      // validate objects
      // TODO: allow size mismatch between names, contours, and probabilities
      if (object_msg->names.size() != object_msg->contours.size()) {
        NODELET_ERROR("Invalid objects");
        return;
      }

      // dark the original image to emphasize objects
      image->image /= 2;

      // extract contours from message
      std::vector< std::vector< cv::Point > > contours;
      BOOST_FOREACH (const object_detection_msgs::Points &points_msg, object_msg->contours) {
        std::vector< cv::Point > points;
        BOOST_FOREACH (const object_detection_msgs::Point &point_msg, points_msg.points) {
          points.push_back(cv::Point(point_msg.x, point_msg.y));
        }
        contours.push_back(points);
      }

      // draw contours in red
      cv::polylines(image->image, contours,
                    true /* is_closed (e.g. draw line from last to first point) */,
                    CV_RGB(255, 0, 0), line_tickness_);

      // draw names in white at the center of corresponding contours
      for (std::size_t i = 0; i < object_msg->names.size(); ++i) {
        const cv::Rect rect(cv::boundingRect(contours[i]));
        const cv::Size text_size(cv::getTextSize(object_msg->names[i], cv::FONT_HERSHEY_SIMPLEX,
                                                 font_scale_, text_tickness_,
                                                 NULL /* baseline (won't use) */));
        cv::putText(image->image, object_msg->names[i],
                    cv::Point(rect.x + (rect.width - text_size.width) / 2,
                              rect.y + (rect.height + text_size.height) / 2),
                    cv::FONT_HERSHEY_SIMPLEX, font_scale_, CV_RGB(255, 255, 255), text_tickness_);
      }

      // TODO: draw probabilities

      // publish annotated image
      image_publisher_.publish(image->toImageMsg());

    } catch (const std::exception &error) {
      NODELET_ERROR_STREAM(error.what());
    }
  }

private:
  int line_tickness_, text_tickness_;
  double font_scale_;

  image_transport::SubscriberFilter image_subscriber_;
  message_filters::Subscriber< object_detection_msgs::Objects > object_subscriber_;
  boost::scoped_ptr< SyncSubscriber > sync_subscriber_;

  image_transport::Publisher image_publisher_;
};

} // namespace object_drawer

#endif