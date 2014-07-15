
#include "UVCCamConfig.h"
#include "opencv2/features2d/features2d.hpp"

#include <list>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

#include <cstdio>
#include <thread>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <rospack/rospack.h>

#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>

#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>


#include <signal.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>


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

    std::thread spin_thread_;
    std::thread loop_grab_image_thread_;

    ros::Subscriber sub_;

    int nb_skipping_frames_;
    int skipping_id_;

    CascadeClassifier cascade_;
    vector<Rect> faces_;

    Rect detect_box_;
    Rect track_box_;

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
    void processTiredFrame(Mat frame );
    void processFrame(Mat frame );
    Rect trackKeypoints(Mat img1, Mat img0);
    void getKeypoints(Mat input_image, Rect detect_box);
    void addKeypoints( Mat input_image,Rect track_box);
    int distanceToCluster(Point2f test_point, Mat cluster);
    int dropKeypoints(int min_keypoints, double outlier_threshold,double mse_threshold);
    Rect detectInitialFaces(Mat& frame);
    Rect getWindow(Rect r);
    Rect filterFaces(Mat& frame);
    double dist(Rect current, int width, int height );
    void drawResult(Mat& frame);

};

PLUGINLIB_DECLARE_CLASS(hiwr_opencv_detector, HiwrOpencvDetectorNodelet, hiwr_opencv_detector::HiwrOpencvDetectorNodelet, nodelet::Nodelet);
}

