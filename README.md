# object_drawer
A ROS nodelet drawing object detection results on image

## Dependencies
object_detection_msgs
* https://github.com/yoshito-n-students/object_detection_msgs

## Subscribed Topics
image_raw (sensor_msgs/Image)
* base image to be annotated

objects_in (object_detection_msgs/Objects)
* detected labels on subscribed images
* timestamp must match that of a subscribed image

## Published Topics
image_out (sensor_msgs/Image)
* annotated image showing contours and names of detected labels
* subtopics supported by image_transport are also published

## Parameters
~queue_size (int, default: 10)
* queue size of a synchronizer for subscribed images and labels

~line_tickness (int, default: 3)
* tickness of detected labels' contours in published images

~text_tickness (int, defalut: 2)
* tickness of detected labels' names in published images

~font_scale (double, default: 0.8)
* font size of detected labels' names in published images

~image_transport (string, default: "raw")
* transport type of the subscribed image topic