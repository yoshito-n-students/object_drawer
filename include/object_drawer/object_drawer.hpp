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
    pixel_scale_ = pnh.param("pixel_scale", 0.5);
    line_tickness_ = pnh.param("line_tickness", 3);
    line_color_ = colorParam(pnh, "line_color", CV_RGB(255, 0, 0));
    text_tickness_ = pnh.param("text_tickness", 2);
    text_color_ = colorParam(pnh, "text_color", CV_RGB(255, 255, 255));
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

      // scale the original image pixels (usually darken to emphasize objects)
      image->image *= pixel_scale_;

      // draw each object
      const std::size_t n_objects(
          std::max(std::max(object_msg->names.size(), object_msg->probabilities.size()),
                   object_msg->contours.size()));
      for (std::size_t i = 0; i < n_objects; ++i) {
        // extract an object
        const std::string name(i < object_msg->names.size() ? object_msg->names[i] : std::string());
        const double probability(i < object_msg->probabilities.size() ? object_msg->probabilities[i]
                                                                      : -1.);
        const std::vector< cv::Point > contour(
            i < object_msg->contours.size()
                ? object_detection_msgs::toCvPoints(object_msg->contours[i])
                : std::vector< cv::Point >());

        // draw contour
        if (contour.size() >= 2) {
          cv::polylines(image->image, std::vector< std::vector< cv::Point > >(1, contour),
                        true /* is_closed (e.g. draw line from last to first point) */, line_color_,
                        line_tickness_);
        }

        // draw the name at the center of corresponding contour
        if (!name.empty()) {
          const cv::Rect rect(cv::boundingRect(contour));
          const cv::Size text_size(cv::getTextSize(name, cv::FONT_HERSHEY_SIMPLEX, font_scale_,
                                                   text_tickness_,
                                                   NULL /* baseline (won't use) */));
          cv::putText(image->image, name,
                      cv::Point(rect.x + (rect.width - text_size.width) / 2,
                                rect.y + (rect.height + text_size.height) / 2),
                      cv::FONT_HERSHEY_SIMPLEX, font_scale_, text_color_, text_tickness_);
        }

        // draw probability
        if (probability >= 0. && probability <= 1.) {
          // TODO: draw probabilities
        }
      }

      // publish annotated image
      image_publisher_.publish(image->toImageMsg());

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

private:
  double pixel_scale_;
  int line_tickness_, text_tickness_;
  cv::Scalar line_color_, text_color_;
  double font_scale_;

  image_transport::SubscriberFilter image_subscriber_;
  message_filters::Subscriber< object_detection_msgs::Objects > object_subscriber_;
  boost::scoped_ptr< SyncSubscriber > sync_subscriber_;

  image_transport::Publisher image_publisher_;
};

} // namespace object_drawer

#endif