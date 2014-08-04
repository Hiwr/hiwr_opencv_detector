/*********************************************************************
*
*
* Copyright 2014 Worldline
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
***********************************************************************/

// Common
//#include "UVCCamConfig.h"
#include <list>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdio>
#include <signal.h>
#include <image_transport/image_transport.h>

// Thread
#include <thread>

// Msgs
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <rospack/rospack.h>
#include <cv_bridge/cv_bridge.h>

// Sensors
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

// Driver Base
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

// Camera

// Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/features2d/features2d.hpp"

// Nodelet
#include <nodelet/nodelet.h>


#include <pluginlib/class_list_macros.h>


//Uncomment to enable display
#define DEBUG_DISPLAY

using namespace cv;

namespace hiwr_opencv_detector
{

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

class HiwrOpencvDetectorNodelet : public nodelet::Nodelet{

protected:

    // pointer on the image
    cv_bridge::CvImagePtr im_ptr_;

    Mat frame_;
    bool configUpdated_;


private:
    // Node handler
    ros::NodeHandle private_nh_;
    ros::NodeHandle priv_NH_;              // private node handle
    ros::NodeHandle camera_nh_;           // camera name space handle
    sensor_msgs::Image image_;

    /** image transport interfaces */
    IplImage  *image_ipl_ ;

    // suscriber for the camera
    std::string video_stream_name_;
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport * it_;
    ros::Publisher pub_;

    std::thread loop_thread_;

    bool im_processed_;
    bool im_subscribed_;
    bool im_available_;

    bool spining_state_;
    bool publishing_state_;
    bool processing_state_;

    unsigned char *final_;

    image_transport::Publisher image_publisher_;
    image_transport::Publisher image_pub_;

    // Threads
    std::thread spin_thread_;
    std::thread loop_grab_image_thread_;

    //Subscriber(s)
    ros::Subscriber sub_;

    int nb_skipping_frames_;
    int skipping_id_;

    CascadeClassifier cascade_;
    vector<Rect> faces_;

    Rect detect_box_;
    Rect track_box_;

    // Frames
    Mat keypoints_;
    Mat prev_frame_;

    int gf_max_corners_;
    double gf_quality_level_ ;
    int gf_min_distance_ ;
    int gf_block_size_ ;
    bool gf_use_harris_detector_ ;

    int frame_index_;

    int add_keypoint_distance_ ;
    int add_keypoints_interval_ ;


    int drop_keypoints_interval_;
    int abs_min_keypoints_ ;
    double  std_err_xy_ ;
    double  pct_err_z_;
    int max_mse_ ;

    int min_keypoints_ ;

    double expand_roi_init_ ;
    double expand_roi_ ;

    std::deque<Rect> face_buffer_;
    int face_buffer_limit_;

public:
    HiwrOpencvDetectorNodelet();

    virtual void onInit();
    void callback(const sensor_msgs::ImageConstPtr& msg);
    void loop();

    bool configure( );
    void processTrackingFrame(Mat frame );
    void processFrame(Mat frame );
    Rect trackKeypoints(Mat img1, Mat img0);
    void addKeypoints( Mat input_image,Rect track_box);
    int distanceToCluster(Point2f test_point, Mat cluster);
    int dropKeypoints(int min_keypoints, double outlier_threshold,double mse_threshold);
    Rect detectInitialFaces(Mat& frame);
    Rect getWindow(Rect r);
    Rect filterFaces(Mat& frame);
    double dist(Rect current, int width, int height );
    void drawResult(Mat& frame);
    void getKeypoints(Mat input_image, Rect detect_box);
};

PLUGINLIB_DECLARE_CLASS(hiwr_opencv_detector, HiwrOpencvDetectorNodelet, hiwr_opencv_detector::HiwrOpencvDetectorNodelet, nodelet::Nodelet);
}

