#ifndef UVC_CAM_NODE_H
#define UVC_CAM_NODE_H

#include <opencv/cv.h>
//#include <opencv/cxcore.h>
#include <opencv/highgui.h>
//#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include "UVCCamConfig.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "std_msgs/String.h"

#include <hyve_msg/SetState.h>
#include <hyve_msg/GetState.h>
#include <hyve_msg/TouchEvent.h>

#include <list>

using namespace cv;

namespace facetracking_nodelet
{

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

class HyveFaceTracker : public nodelet::Nodelet{




protected:

    // pointer on the image
    cv_bridge::CvImagePtr im_ptr;

    Mat frame;
    //sensor_msgs::CameraInfo cam_info_;
    /** dynamic parameter configuration */
    //typedef hyve_camera_common::UVCCamConfig Config;
    //Config config_;
    //void reconfig(Uvc_cam_node::Config&, uint32_t);
    //void locked_reconfig(Uvc_cam_node::Config &, uint32_t );
    bool configUpdated;



    //volatile int config_width;
    //volatile int config_height;

private:
   // Driver::state_t state_;               // current driver state


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



    std::thread loop_thread;

    bool im_processed;
    bool im_subscribed;
    bool im_available;

    bool spiningState;
    bool publishingState;
    bool processingState;

    unsigned char *final ;

   // void closeCamera();
   // bool openCamera(Uvc_cam_node::Config&);
   // void deal_memory();
   // bool copy_read();
   // bool read();

    image_transport::Publisher imagePublisher;
    image_transport::Publisher image_pub_;
   // void publish_frame(cv::Mat);

    std::thread spin_thread;
    std::thread loop_grab_image_thread;

    ros::Subscriber sub;

    int nb_skipping_frames;
    int skipping_id;

    CascadeClassifier cascade;
    vector<Rect> faces;
    //vector<Rect> windows;

    Rect detect_box;
    Rect track_box;
    //vector<Point2f>
    Mat keypoints;
    Mat prev_frame ;

    int gf_maxCorners;
    // = 200 ;

    double gf_qualityLevel ; // = rospy.get_param("~gf_qualityLevel", 0.02)
    int gf_minDistance ; // = rospy.get_param("~gf_minDistance", 7)
    int gf_blockSize ; // = rospy.get_param("~go_blockSize", 10)
    bool gf_useHarrisDetector ; //= rospy.get_param("~gf_useHarrisDetector", true);
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

    // publisher for the output
    //ros::Publisher pub;

   HyveFaceTracker();


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

   /* Uvc_cam_node():
        privNH_("~"),
        camera_nh_("camera"),
        cinfo_(camera_nh_)//,
      //  it_(camera_nh_)
    {
        state_ = Driver::CLOSED;
        calibration_matches_ = true;
        device_ = "Microsoft® LifeCam Cinema(TM)";
        camera_name_ = "camera";
    };*/

    //~Uvc_cam_node() {
    //    if(state_ != Driver::CLOSED) {
    //        closeCamera();
    //    }
        //End processing thread
        /*spin_thread.terminate();
        loop_grab_image_thread.terminate();*/
   // }
   /* void spin();
    std::unique_lock<std::mutex> getLock();

    virtual void configure( ){} ;
    virtual void process_frame(cv::Mat frame ){} ;
    void change_size(int width, int height);

    void setSpiningState(bool pSpinningState);
    bool getSpiningState();

    void setProcessingState(bool pProcessingState);
    bool getProcessingState();

    void setPublishingState(bool pPublishingState);
    bool getPublishingState();

    void configurePublishing(ros::NodeHandle);*/


};

PLUGINLIB_DECLARE_CLASS(facetracking_nodelet, HyveFaceTracker, facetracking_nodelet::HyveFaceTracker, nodelet::Nodelet);
}

#endif
