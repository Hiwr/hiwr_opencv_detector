
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

class Hiwr_opencv_detector_nodelet : public nodelet::Nodelet{

protected:

    // pointer on the image
    cv_bridge::CvImagePtr im_ptr;

    Mat frame;
    bool configUpdated;


private:
    ros::NodeHandle private_nh;
    ros::NodeHandle privNH_;              // private node handle
    ros::NodeHandle camera_nh_;           // camera name space handle
    sensor_msgs::Image image_;

    /** image transport interfaces */
    IplImage  *imageIpl  ;

    // suscriber for the camera
    std::string video_stream_name;
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport * it_;
    ros::Publisher pub;


    std::thread loop_thread;

    bool im_processed;
    bool im_subscribed;
    bool im_available;

    bool spiningState;
    bool publishingState;
    bool processingState;

    unsigned char *final ;

    image_transport::Publisher imagePublisher;
    image_transport::Publisher image_pub_;

    std::thread spin_thread;
    std::thread loop_grab_image_thread;

    ros::Subscriber sub;

    int nb_skipping_frames;
    int skipping_id;

    CascadeClassifier cascade;
    vector<Rect> faces;

    Rect detect_box;
    Rect track_box;

    Mat keypoints;
    Mat prev_frame ;

    int gf_maxCorners;


    double gf_qualityLevel ;
    int gf_minDistance ;
    int gf_blockSize ;
    bool gf_useHarrisDetector ;
    int frame_index;

    int add_keypoint_distance ;
    int add_keypoints_interval ;


    int drop_keypoints_interval;
    int abs_min_keypoints ;
    double  std_err_xy ;
    double  pct_err_z ;
    int max_mse ;

    int min_keypoints ;

    double expand_roi_init ;
    double expand_roi ;

    std::deque<Rect> face_buffer;
    int face_buffer_limit;

public:

   Hiwr_opencv_detector_nodelet();


    virtual void onInit();
    void callback(const sensor_msgs::ImageConstPtr& msg);
    void loop();

    bool configure( );
    void process_tracking_frame(Mat frame );
    void process_tired_frame(Mat frame );
    void process_frame(Mat frame );
    Rect track_keypoints(Mat img1, Mat img0);
    void get_keypoints(Mat input_image, Rect detect_box);
    void add_keypoints( Mat input_image,Rect track_box);
    int distance_to_cluster(Point2f test_point, Mat cluster);
    int drop_keypoints(int min_keypoints, double outlier_threshold,double mse_threshold);
    Rect detectInitialFaces(Mat& frame);
    Rect getWindow(Rect r);
    Rect filterFaces(Mat& frame);
    double dist(Rect current, int width, int height );
    void drawResult(Mat& frame);

};

PLUGINLIB_DECLARE_CLASS(hiwr_opencv_detector, Hiwr_opencv_detector_nodelet, hiwr_opencv_detector::Hiwr_opencv_detector_nodelet, nodelet::Nodelet);
}

