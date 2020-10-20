# object_drawer
A ROS nodelet drawing object detection results on image

## Dependencies
object_detection_msgs
* https://github.com/yoshito-n-students/object_detection_msgs

## Subscribed Topics
image_raw (sensor_msgs/Image)
* base image to be annotated

objects_in (object_detection_msgs/Objects)
* detected objects on subscribed images
* timestamp must match that of a subscribed image

## Published Topics
image_out (sensor_msgs/Image)
* annotated image showing contours and names of detected objects
* consisits of 4 layers; single-colored image (bottom), subscribed image, objects' contours, and texts (top)
* subtopics supported by image_transport are also published

## Parameters
~queue_size (int, default: 10)
* queue size of a synchronizer for subscribed images and objects

~base_color (int[3], default: [ 0, 0, 0 ])
* color of the lowest layer of a published image

~image_transparency (double, default: 0.5)
* transparency of subscribed image layers in published images

~line_thickness (int, default: 3)
* thickness of subscribed objects' contours in published images

~line_color (int[3], default: [ 255, 0, 0 ])
* color of subscribed objects' contours in published images

~line_transparency (double, default: 0.0)
* transparency of subscribed objects' contours in published images

~text_thickness (int, defalut: 2)
* thickness of subscribed objects' names in published images

~text_color (int[3], default: [ 255, 255, 255 ])
* color of subscribed objects' names in published images

~text_transparency (double, default: 0.0)
* transparency of subscribed objects' names in published images

~font_scale (double, default: 0.8)
* font size of subscribed objects' names in published images

~image_transport (string, default: "raw")
* transport type of the subscribed image topic

## Examples
see [launch/test.launch](launch/test.launch)