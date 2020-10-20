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
#include <object_detection_msgs/cv_conversions.hpp>
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
    base_color_ = colorParam(pnh, "base_color", CV_RGB(0, 0, 0));
    image_transparency_ = pnh.param("image_transparency", 0.5);
    line_thickness_ = pnh.param("line_thickness", 3);
    line_color_ = colorParam(pnh, "line_color", CV_RGB(255, 0, 0));
    line_transparency_ = pnh.param("line_transparency", 0.);
    text_thickness_ = pnh.param("text_thickness", 2);
    text_color_ = colorParam(pnh, "text_color", CV_RGB(255, 255, 255));
    text_transparency_ = pnh.param("text_transparency", 0.);
    font_scale_ = pnh.param("font_scale", 0.8);

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
      const cv_bridge::CvImagePtr image_in(cv_bridge::toCvCopy(image_msg, "bgr8"));
      if (!image_in) {
        NODELET_ERROR("Image conversion error");
        return;
      }
      if (image_in->image.empty()) {
        NODELET_ERROR("Empty image message");
        return;
      }

      // create output image filled with the base color
      cv_bridge::CvImage image_out(
          image_in->header, image_in->encoding,
          cv::Mat(image_in->image.size(), image_in->image.type(), base_color_));

      // overlay the subscribed image on the output image
      overlay(image_out.image, image_in->image, image_transparency_,
              cv::Mat::ones(image_in->image.size(), CV_8UC1));

      // convert the subscribed contours to OpenCV's type
      const std::vector< std::vector< cv::Point > > contours(
          object_detection_msgs::toCvContours(object_msg->contours));

      // overlay the contours on the output image
      cv::Mat contours_mask(cv::Mat::zeros(image_in->image.size(), CV_8UC1));
      BOOST_FOREACH (const std::vector< cv::Point > &contour, contours) {
        if (contour.size() >= 2) {
          cv::polylines(contours_mask, std::vector< std::vector< cv::Point > >(1, contour),
                        true /* is_closed (e.g. draw line from last to first point) */, 1,
                        line_thickness_);
        }
      }
      overlay(image_out.image, cv::Mat(image_in->image.size(), image_in->image.type(), line_color_),
              line_transparency_, contours_mask);

      // overlay the subscribed name texts on the output image
      const std::size_t n_texts(std::min(contours.size(), object_msg->names.size()));
      cv::Mat text_mask(cv::Mat::zeros(image_in->image.size(), CV_8UC1));
      for (std::size_t i = 0; i < n_texts; ++i) {
        const std::string &text(object_msg->names[i]); // TODO: add probability to text
        const std::vector< cv::Point > &contour(contours[i]);
        if (!text.empty() && !contour.empty()) {
          const cv::Rect rect(cv::boundingRect(contour));
          const cv::Size text_size(cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale_,
                                                   text_thickness_,
                                                   NULL /* baseline (won't use) */));
          cv::putText(text_mask, text,
                      cv::Point(rect.x + (rect.width - text_size.width) / 2,
                                rect.y + (rect.height + text_size.height) / 2),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale_, 1, text_thickness_);
        }
      }
      overlay(image_out.image, cv::Mat(image_in->image.size(), image_in->image.type(), text_color_),
              text_transparency_, text_mask);

      // publish annotated image
      image_publisher_.publish(image_out.toImageMsg());

    } catch (const std::exception &error) {
      NODELET_ERROR_STREAM(error.what());
    }
  }

  // utility function to load RGB color parameter
  // (this function cannot be static member function due to NODELET_XXX macros)
  cv::Scalar colorParam(ros::NodeHandle &nh, const std::string &name,
                        const cv::Scalar &default_val) {
    std::vector< int > val;
    if (!nh.getParam(name, val)) {
      return default_val;
    }

    if (val.size() < 3) {
      NODELET_ERROR_STREAM("Element size of " << nh.resolveName(name)
                                              << " is less than 3. Will use the default color.");
      return default_val;
    }

    if (val.size() > 3) {
      NODELET_WARN_STREAM("Element size of "
                          << nh.resolveName(name)
                          << " is greater than 3. Only 1st to 3rd elements will be used.");
    }

    return CV_RGB(val[0], val[1], val[2]);
  }

  // utility function to overlay images
  static void overlay(cv::Mat &image, const cv::Mat &layer_image, const double layer_transparency,
                      const cv::Mat &layer_mask) {
    cv::Mat full_image;
    // full_image = transparency*image + (1-transparency)*layer + 0
    cv::addWeighted(image, layer_transparency, layer_image, 1. - layer_transparency, 0.,
                    full_image);
    full_image.copyTo(image, layer_mask);
  }

private:
  cv::Scalar base_color_, line_color_, text_color_;
  double image_transparency_, line_transparency_, text_transparency_;
  int line_thickness_, text_thickness_;
  double font_scale_;

  image_transport::SubscriberFilter image_subscriber_;
  message_filters::Subscriber< object_detection_msgs::Objects > object_subscriber_;
  boost::scoped_ptr< SyncSubscriber > sync_subscriber_;

  image_transport::Publisher image_publisher_;
};

} // namespace object_drawer

#endif