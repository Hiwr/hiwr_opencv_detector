hiwr_opencv_detector
===============================================
 
The hiwr\_opencv\_detector ROS package aims to detect object or faces. It uses a video stream as an input, and a parameter file for each object to detect.
It gives as a result a ROI (Region Of Interest), where the object is detected in the frame.


Contributing
----------------------

Contributions via pull request are welcome and may be included under the
same license as below.

Copyright
----------------------

hiwr\_opencv\_detector, except where otherwise noted, is released under the
[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0.html).
See the LICENSE file located in the root directory.

Build
----------------------
It requires hyve_msg to work properly

Execution
----------------------
Make sure that your project is compiled and sourced.

To start hiwr\_opencv\_detector, do the following (assuming you
have a working ROS core running):

Launch using roslaunch:

    $ roslaunch hiwr_opencv_detector facetracking.launch 

Launch from another launchfile:

    <include file="$(find hiwr_opencv_detector)/facetracking.launch" />
 
Node
----------------------

### Subscribed Topics

* `/camera_name/output_video`
 * The video stream topic

### Published Topics

* `/uvc_cam_node/roi`
  * Get the last ROI message (geometry_msgs/RegionOfInterest)

### Parameters

* `haarfile` (String)
 * File to load for object or faces detection