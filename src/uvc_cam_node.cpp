/*
 * Copyright (c) 2009, Morgan Quigley, Clemens Eppner, Tully Foote
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Modified Apr 6, 2010 by Adam Leeper - changed to use "image_transport"

/* Much of this code is either heavily inspired by or taken directly from the
 * camera1394 ROS driver by Jack O'Quin
 */
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

#include <signal.h>
#include <cstdio>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <ros/ros.h>
#include <ros/time.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <tf/transform_listener.h>
//#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <ros/console.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rospack/rospack.h>
#include <opencv2/video/tracking.hpp>

#include <pluginlib/class_list_macros.h>
//#include <nodelet/nodelet.h>

#include "uvc_cam_node.h"
/*
#include <devel/include/hyve_msg/SetState.h>
#include <devel/include/hyve_msg/GetState.h>
#include <devel/include/hyve_msg/TouchEvent.h>
*/

using namespace cv;

namespace facetracking_nodelet{

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

enum currentState {TRACKING,TIRED,OFF};

int tiredRatio;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    fprintf(stderr, "Segmentation fault, stopping uvc camera driver.\n");
    ROS_ERROR("Segmentation fault, stopping uvc camera driver.");
    ros::shutdown();                      // stop the main loop
}

ros::Publisher pub;
//ros::Publisher pubTiredValue;
//ros::Publisher pubTiredState;


HyveFaceTracker::HyveFaceTracker() {
    NODELET_DEBUG("[Facetracking Nodelet] Constructor");
}


bool HyveFaceTracker::configure( ){
    std::string defaultString ;

    printf("ros published \n");
    rospack::Rospack pack ;
    std::vector<std::string> search_path;
    pack.getSearchPathFromEnv(search_path);
    pack.crawl(search_path, false);
    pack.find("facetracking", defaultString) ;

    std::string file;

    private_nh.getParam("haarfile" , file);
    // ros::param::get("haarfile" , file);
    NODELET_DEBUG("[Facetracking Nodelet][configure] loading file is: %s/  bob  %s", defaultString.c_str(), file.c_str());
    if (!cascade.load(defaultString+ '/'+ file)){
        //   if (!cascade.load(defaultString+ '/'+ "lbpcascade_frontalface.xml")){
        NODELET_ERROR("[Facetracking Nodelet][configure] cascade load failed!");
        return false;
    }

    std::string gf_maxCorners_string;
    ros::param::get("~gf_maxCorners", gf_maxCorners_string);
    //	std::cout << "will stoid ";
    //	std::cout << gf_maxCorners_string;
    //gf_maxCorners = std::stoi(gf_maxCorners_string);

    ros::param::get("~gf_qualityLevel", gf_qualityLevel);
    ros::param::get("~gf_minDistance", gf_minDistance);
    ros::param::get("~gf_blockSize", gf_blockSize);
    ros::param::get("~gf_useHarrisDetector", gf_useHarrisDetector);
    ros::param::get("~drop_keypoints_interval", drop_keypoints_interval);

    ros::param::get("~abs_min_keypoints", abs_min_keypoints);
    ros::param::get("~std_err_xy", std_err_xy) ;
    ros::param::get("~pct_err_z", pct_err_z) ;
    ros::param::get("~max_mse", max_mse);

    ros::param::get("~add_keypoint_distance", add_keypoint_distance);
    ros::param::get("~add_keypoints_interval", add_keypoints_interval);

    ros::param::get("~min_keypoints", min_keypoints);

    ros::param::param<int>("~tired_value", tiredRatio, 30);


    ros::param::get("~expand_roi_init", expand_roi_init);
    expand_roi = expand_roi_init;


    // = rospy.get_param("~gf_qualityLevel", 0.02)
    gf_qualityLevel = 0.02;
    gf_minDistance  = 2 ;
    gf_blockSize = 5 ;
    gf_useHarrisDetector = true; //= rospy.get_param("~gf_useHarrisDetector", true);
    frame_index = 0;
    drop_keypoints_interval = 2;

    add_keypoint_distance = 2 ;
    add_keypoints_interval = 1 ;
    min_keypoints = 20;

    abs_min_keypoints = 6;
    std_err_xy = 2.5;
    pct_err_z =  0.42;
    max_mse = 10000;
    expand_roi_init = 1.02;
    expand_roi = expand_roi_init;

    nb_skipping_frames = 0;
    skipping_id = 0;

    face_buffer_limit = 5;

    NODELET_DEBUG("[Facetracking Nodelet][configure] spined %f %d %d \n", gf_qualityLevel ,  gf_minDistance ,  gf_maxCorners);
    return true;
}

void HyveFaceTracker::onInit(){
    NODELET_DEBUG("[Facetracking Nodelet][onInit] beginning");

    ros::NodeHandle& public_nh = getNodeHandle();
    private_nh = getMTPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(public_nh);

    if(!configure()) {
        NODELET_ERROR("[Facetracking Nodelet][onInit] configure failed!");
    }

    if(!private_nh.getParam("video_stream", video_stream_name)){
        NODELET_ERROR("[Facetracking Nodelet][onInit] Problem recovering the video stream");
        return;
    }


    frame.rows = 0;

    pub = public_nh.advertise<sensor_msgs::RegionOfInterest>("/uvc_cam_node/roi", 1);
    // recovery of the picture
    image_sub_ = it_->subscribe(video_stream_name.c_str(), 1,&HyveFaceTracker::callback, this);
    im_processed = false;
    im_available = false;

    NODELET_DEBUG("[Facetracking Nodelet][onInit] call thread loop");


    loop_thread = std::thread(&HyveFaceTracker::loop , this);

    NODELET_DEBUG("[Facetracking Nodelet][onInit] end");
}

// processing the image here
void HyveFaceTracker::callback(const sensor_msgs::ImageConstPtr& msg){
    NODELET_DEBUG("[Facetracking nodelet] callback : beginning");

    // recovery of the image
    try{
        im_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        // NODELET_DEBUG("[Facetracking Nodelet] callback : frame type : %d", im_ptr->type());

        // facetracking
        //if(im_processed){
        NODELET_DEBUG("[Facetracking Nodelet] callback : im_ptr ok");

        if(!im_available){
            // collecting the infos for frame
            frame = im_ptr->image;

            NODELET_DEBUG("[Facetracking Nodelet] callback : frame type : %d", frame.type());

            // cvtColor(frame, frame, CV_8UC1);

            NODELET_DEBUG("[Facetracking Nodelet] callback : frame type 2 : %d", frame.type());


            im_available = true;
        }

    }
    catch(cv_bridge::Exception& e){
        NODELET_ERROR("\nError callback : recovery of the image");
        NODELET_DEBUG("\nError callback : recovery of the image");
        return;
    }

    NODELET_DEBUG("[Facetracking nodelet] callback : end");

}

// loop to check if the image needs to be subscribed
void HyveFaceTracker::loop(){
    NODELET_DEBUG("Beginning loop");
    //cv::namedWindow("Francis", CV_WINDOW_AUTOSIZE);
    while(ros::ok()){
        // image_sub_ = it_->subscribe(video_stream_name.c_str(), 1,&HyveFaceTracker::callback, this);
        if((im_ptr!=NULL) && im_available){
            NODELET_DEBUG("[Facetracking Nodelet] loop : display");
            // cvtColor(frame, frame, );
            process_tracking_frame(frame);
            //cv::imshow("Francis", frame);
            //  NODELET_DEBUG("[Facetracking Nodelet] loop : display end");
            //cv::waitKey(20);
            im_available = false;
        }else usleep(100);
    }

    NODELET_DEBUG("End loop");
}

void HyveFaceTracker::process_tracking_frame(Mat frame ) {
    if (detect_box.width ==0 &&  detect_box.height==0)
    {
        NODELET_DEBUG("[Facetracking Nodelet] Processing detection");
        // if(skipping_id > nb_skipping_frames){
        //   skipping_id = 0;
        Rect box = detectInitialFaces(frame);




        if(box.width ==0 && box.height== 0){
            //Avoid undefined behaviors
            if(face_buffer.size() > 0)
                face_buffer.pop_front();
            return;
        }


        //Display mean face
        Rect mean;
        int  face_size = face_buffer.size();
        for(int i = 0; i < face_size; i++){
            mean.x+= face_buffer.at(i).x;
            mean.y += face_buffer.at(i).y;
            mean.width += face_buffer.at(i).width;
            mean.height += face_buffer.at(i).height;
        }
        if(face_size >0 ){
            mean.x /= face_size;
            mean.y /= face_size;
            mean.width /= face_size;
            mean.height /= face_size;
            rectangle(frame, mean, Scalar(255),2);
            sensor_msgs::RegionOfInterest msg;
            msg.x_offset =mean.x;
            msg.y_offset =mean.y;
            msg.width =mean.width;
            msg.height =mean.height;
            pub.publish(msg);

        }

        if(face_buffer.size() < face_buffer_limit){
            face_buffer.push_back(box);
            NODELET_INFO("PUSH");
        }
        else{
            NODELET_INFO("POP");
            face_buffer.pop_front();
            face_buffer.push_back(box);



        }


        /*detect_box =*/
        return;
        NODELET_DEBUG("[Facetracking Nodelet] End detection");
        //std::cout  << "detect box is" << detect_box << std::endl;
        // needed to reset the trackbox

        track_box = detect_box;
        keypoints.release();
        //we want to calc getpoint on this image, this the haar is long, next frame maybe to different
        // only if we get points ?
        get_keypoints( frame , detect_box);

        if(keypoints.size().height<5){
            NODELET_DEBUG("[Facetracking Nodelet] tracking : less than 5 keypoints");    // -> OK
            detect_box.width =0;
            detect_box.height =0;
            keypoints.release(); //empty the list
        }
        //  }
        // skipping_id++;
    }
    else
    {
        //Display keypoints
        for(int i =0 ; i<keypoints.size().height ; i++ ){
            Point2f point = keypoints.at<Point2f>(i);
            circle(frame, point, 2, Scalar(0),1);
        }

        NODELET_DEBUG("Processing tracking");
        //Step 2: If we aren't yet tracking keypoints, get them now
        if( keypoints.size().height==0 ) {
            /*Get point to track*/
            track_box = detect_box;
            get_keypoints( frame , track_box);
        }

        if(keypoints.size().height>5){
            // Step 3: If we have keypoints, track them using optical flow
            track_box = track_keypoints(frame.clone(), prev_frame);

            //Debug : draw them
            rectangle(frame, Point(track_box.x, track_box.y), Point(track_box.x+track_box.width, track_box.y+track_box.height), Scalar(0),2);

            //# Step 4: Drop keypoints that are too far from the main cluster
            if (frame_index % drop_keypoints_interval == 0 and keypoints.size().height>0){
                frame_index=0;
                int score = drop_keypoints( abs_min_keypoints, std_err_xy, max_mse);
                if (score == -1){
                    detect_box.width = 0;
                    detect_box.height = 0;
                }
            }
            // Step 5: Add keypoints if the number is getting too low
            if( frame_index % add_keypoints_interval == 0 and keypoints.size().height < min_keypoints){
                expand_roi = expand_roi_init * expand_roi ;
                add_keypoints( frame, track_box);
            } else {
                frame_index += 1 ;
                expand_roi = expand_roi_init;
            }

        }else{
            //
            printf("we got only %d keypont <F4> need to haarify \n", keypoints.size().height );
            detect_box.width =0;
            detect_box.height =0;
            keypoints.release(); //empty the list
        }

    }

    prev_frame =frame.clone();
    //prev_frame =frame.clone();
    NODELET_DEBUG("[Facetracking Nodelet] procees tracking : before drawing");
    //drawResult(frame);
    //cv::flip(frame,frame,1);
    //imshow("toto", frame);
    //cvWaitKey(1) ;

    // publish only if good ?
    if( track_box.x > 0 && track_box.y > 0  && track_box.width >0 && track_box.height > 0 ) {
        sensor_msgs::RegionOfInterest msg;
        msg.x_offset =track_box.x;
        msg.y_offset =track_box.y;
        msg.width =track_box.width;
        msg.height =track_box.height;
        pub.publish(msg);
        frame_index ++;
    }
    //publish();

}

void HyveFaceTracker::process_tired_frame(Mat frame ) {
    NODELET_DEBUG("Processing tired frame");
    Mat out(Size(1,1) , CV_8UC1) ;
    cv::resize(frame , out ,  out.size() );

    int current = out.data[0];
    std_msgs::String msgStr;
    std_msgs::UInt8 msgUInt8;
    msgUInt8.data = current;
    if(current< tiredRatio){
        msgStr.data = "tired";
    }else{
        msgStr.data = "dailight";
    }
    //pubTiredValue.publish( msgUInt8);
    //pubTiredState.publish( msgStr);
}

// useless : changes the currentState, which is not used anymore
void HyveFaceTracker::process_frame(Mat frame ) {
    NODELET_DEBUG("process frame");
    //if(currentState == TIRED ) {
    //	return process_tired_frame(frame);
    //}else
    //if(currentState == TRACKING){
    return process_tracking_frame(frame);
    // }
}

Rect HyveFaceTracker::track_keypoints(Mat img1, Mat img0){
    /*   # Reshape the current keypoints into a numpy array required
# by calcOpticalFlowPyrLK()
         */
    Mat p0 = keypoints;
    Mat p0r;
    Mat p1;
    Mat status;
    Mat err;
    // # Calculate the optical flow from the previous frame to the current frame

    //std::cout << "will cal optical " << keypoints << std::endl;

    calcOpticalFlowPyrLK(img0, img1, p0, p1 , status, err);

    //# Do the reverse calculation: from the current frame to the previous frame
    // try:

    calcOpticalFlowPyrLK(img1, img0, p1, p0r, status, err);
    Mat mdiff;
    absdiff( p0 , p0r, mdiff);

    Mat reshaped = mdiff.reshape( mdiff.size().width );


    Mat A = reshaped(Range::all() , Range(0,1));
    Mat B = reshaped(Range::all() , Range(1,2));

    Mat c =  max( A , B );
    Mat new_keypoints;
    for(int jj= 0 ; jj< c.size().height ; jj++){
        if(c.at<float>(jj) < 1.0) {
            new_keypoints.push_back( p1(Range(jj,jj+1) , Range::all()) );
            //                    printf("keep point %f %f \n" , c.at<float>(jj)  , 1.0);
        }else{
            //                  printf("dont keep point %f %f \n" , c.at<float>(jj)  , 1.0);
        }
    }

    keypoints = new_keypoints;
    if(keypoints.size().width == 0 ) {
        track_box.width =0;
        track_box.height =0;
    }else{


        if(keypoints.size().height >6 ){
            track_box = fitEllipse(keypoints).boundingRect();
        }else{
            printf(" we should not be there as this probably introdu a bug later %d  %d \n", keypoints.size().width , keypoints.size().height );
            track_box = boundingRect(keypoints);
        }
    }

    return track_box;
}

void HyveFaceTracker::get_keypoints(Mat input_image, Rect detect_box){

    NODELET_DEBUG("[Facetracking Nodelet] getKeypoints : beginning");
    Mat area = Mat(input_image, detect_box);
    vector<Point2f> output ;
    Mat mask ;
    Mat out ;

    cv::goodFeaturesToTrack(
                area, out, 30, 0.02, 7.0 );
    // ,noArray(), 3, true, 0.04) ;
    /*  , gf_maxCorners , gf_qualityLevel , gf_min_to_cluster
            , noArray()  , gf_blockSize , gf_useHarrisDetector, 0.04 );*/
    //mask = self.mask, **self.gf_params)
    if ( out.cols > 0){
        keypoints = out;
    }

    Mat new_keypoints;
    for(int i =0 ; i<keypoints.size().height ; i++ ){
        Point2f point = keypoints.at<Point2f>(i);
        point.x  +=  detect_box.x ;
        point.y  +=  detect_box.y ;
        new_keypoints.push_back(point);
        //Display points
        circle(input_image, point, 2, Scalar(0),1);
    }
    keypoints = new_keypoints;

    NODELET_DEBUG("[Facetracking Nodelet] getKeypoints : end");

}

void HyveFaceTracker::add_keypoints( Mat input_image,Rect track_box){

    Mat mask = Mat( input_image.size().height ,  input_image.size().width , CV_8UC1, Scalar::all(0));
    //TODO maybe we should deal with ange
    int x = track_box.x ; // + detect_box.x ;
    int y = track_box.y ;//+ detect_box.y;
    int h = track_box.height;
    int w = track_box.width;

    // # Expand the track box to look for new keypoints
    int  w_new = expand_roi * w;
    int  h_new = expand_roi * h;

    Point2f pt1 ;
    pt1.x =  x - w_new/2   + w/2;
    pt1.y =  y - h_new / 2 + h/2 ;

    Point2f pt2 ;
    pt2.x =  x + w_new / 2 + w/2 ;
    pt2.y = y  + h_new / 2 + w/2 ;

    rectangle(input_image, pt1, pt2, Scalar(0),2);

    RotatedRect mask_box ;
    mask_box.center = Point( x +w/2 , y  + h/2);
    //      mask_box.center = Point( 100 , 100);
    mask_box.size = Size(w_new , h_new );
    //   mask_box.size = Size(55 , 55);
    //std::cout << "mask_box " << std::endl;
    mask_box.angle=0;

    //# Create a filled white ellipse within the track_box to define the ROI
    ellipse(mask, mask_box, Scalar(255), CV_FILLED ,8);

    for(int i =0; i< keypoints.size().height ; i++){
        Point2f p = keypoints.at<Point2f>(i);
        circle(mask, p , 5, 0, -1);
    }

    Mat new_keypoints;

    goodFeaturesToTrack(input_image , new_keypoints, 30, 0.02, 7.0 ,  mask);
    //# Append new keypoints to the current list if they are not
    //# too far from the current cluster
    if(new_keypoints.size().height >  0  ){

        for(int i = 0 ; i<new_keypoints.size().height ; i++ ){
            Point2f point =  new_keypoints.at<Point2f>(i);
            int distance = distance_to_cluster(point , keypoints);
            if (distance > add_keypoint_distance){
                keypoints.push_back(point);
            }
        }
    }
    //TODO remove duplicate keypoints    self.keypoints = list(set(self.keypoints))
    /*
           for x, y in np.float32(new_keypoints).reshape(-1, 2):
           distance = self.distance_to_cluster((x,y), self.keypoints)
           if distance > self.add_keypoint_distance:
           self.keypoints.append((x,y))
# Briefly display a blue disc where the new point is added
if self.show_add_drop:
cv2.circle(self.marker_image, (x, y), 3, (255, 255, 0, 0), cv.CV_FILLED, 2, 0)
         */

}

int HyveFaceTracker::distance_to_cluster(Point2f test_point, Mat cluster){
    int min_distance = 10000;
    for(int i = 0; i< cluster.size().height; i++){
        Point2f point = cluster.at<Point2f>(i);
        if(point == test_point)
            continue;
        int distance = abs(test_point.x - point.x)  + abs(test_point.y - point.y);
        if(distance < min_distance)
            min_distance = distance ;
    }
    return min_distance;
}

int HyveFaceTracker::drop_keypoints(int min_keypoints, double outlier_threshold,double mse_threshold){
    int sum_x		= 0;
    int sum_y		= 0;
    double sse	= 0;
    Mat keypoints_xy ;
    int n_xy = keypoints.size().height;

    //  # If there are no keypoints left to track, start over
    if( n_xy == 0){
        return -1;
    }

    // # Compute the COG (center of gravity) of the cluster
    for(int i=0; i< n_xy ; i++){
        Point2f point = keypoints.at<Point2f>(i);
        sum_x += point.x;
        sum_y += point.y;
    }
    double mean_x = sum_x / n_xy;
    double mean_y = sum_y / n_xy;


    //# Compute the x-y MSE (mean squared error) of the cluster in the camera plane
    for(int i=0; i< n_xy ; i++){
        Point2f point = keypoints.at<Point2f>(i);
        sse = sse + (point.x- mean_x) * (point.x - mean_x) + (point.y - mean_y) * (point.y - mean_y);
    }
    // # Get the average over the number of feature points
    double mse_xy = sse / n_xy;

    // # The MSE must be > 0 for any sensible feature cluster
    if (mse_xy == 0 or mse_xy > mse_threshold){
        return -1;
    }

    // # Throw away the outliers based on the x-y variance
    int max_err = 0;
    for(int i=0; i< n_xy ; i++){
        Point2f point = keypoints.at<Point2f>(i);
        double std_err = ((point.x - mean_x) * (point.x - mean_x) + (point.y - mean_y) * (point.y - mean_y)) / mse_xy;
        if (std_err > max_err)
            max_err = std_err;
        if (std_err <= outlier_threshold ){
            keypoints_xy.push_back(point);
        };

    } ;

    keypoints = keypoints_xy;

    //# Consider a cluster bad if we have fewer than min_keypoints left
    if (keypoints.size().height < min_keypoints)
        return -1;
    else
        return 1;

}


/**
     * Detect faces using OpenCV's built-in module          -> OK
     */
Rect HyveFaceTracker::detectInitialFaces(Mat& frame)
{
    double min_face_size=20;
    double max_face_size=200;

    Mat gray;
    //Mat gray = frame;
    NODELET_DEBUG("[Facetracking Nodelet] detectInitialFaces : frame rows : %d", frame.rows);
    //frame.copyTo(gray);
    NODELET_DEBUG("[Facetracking Nodelet] detectInitialFaces : gray rows : %d", gray.rows);
    // cvtColor(frame, gray ,CV_16U); // -> marche pas

    equalizeHist( frame, frame);

    //Rect bob; // -> marche pas
    //faces[0] = bob;

    //cascade.detectMultiScale(gray, faces, 1.1, 3 ,0 , Size(30,30), Size(1000,1000) );
    //cascade.detectMultiScale(frame, faces, 1.1, 3 );//,0 , Size(3,3), Size(10,10) );
    cascade.detectMultiScale( frame, faces, 1.2, 2,  0|CV_HAAR_SCALE_IMAGE, Size(25, 25), Size(120,120));

    // Draw circles on the detected faces
    for( int i = 0; i < faces.size(); i++ )
    {
        min_face_size = faces[0].width*0.8;
        max_face_size = faces[0].width*1.2;
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    NODELET_DEBUG("[Facetracking Nodelet] detectInitialFaces : number of Rects detected : %d", faces.size());
    return filterFaces(frame);

}

Rect HyveFaceTracker::getWindow(Rect r){
    Point pt(r.x + r.width/2, r.y + r.height/2);

    int dim = max( r.height , r.width) * 1.5;

    Rect win;
    win.width  = dim;
    win.height = dim;
    win.x      = pt.x - win.width/2;
    win.y      = pt.y - win.height/2;

    return win;
}

Rect HyveFaceTracker::filterFaces(Mat& frame){
    Rect currentBest ;
    if(faces.size()==0){
        return currentBest;
    }
    currentBest=faces[0];
    double currentDist = dist(currentBest, frame.cols, frame.rows);//cam_info_.width, cam_info_.height);

    Rect current;
    for (unsigned int i = 1; i < faces.size(); i++){
        current = faces[i];
        double _dist = dist(current, frame.rows, frame.cols);//cam_info_.width, cam_info_.height);
        if(_dist<currentDist){
            currentDist = _dist;
            currentBest = current;
        }
    }

    return currentBest;

}

double HyveFaceTracker::dist(Rect current, int width, int height ){
    return  sqrt( pow(  (current.x + current.width/2) - width/2 , 2 ) + pow( (current.y + current.height/2)-height/2 , 2 ) ) ;
}

void HyveFaceTracker::drawResult(Mat& frame)
{
    if(detect_box.x == 0 && detect_box.y == 0){
        NODELET_DEBUG("[Face tracking Nodelet] drawResult : no box detected");
        return;
    }
    Rect r = track_box;

    rectangle(
                frame,
                Point(r.x /*+ detect_box.x */ , r.y /*+ detect_box.y*/ ),
                Point(r.x + r.width /*+ detect_box.x */, r.y + r.height /*+detect_box.y */),
                CV_RGB(255,0,0),
                -1
                );

    r = detect_box;
    rectangle(
                frame,
                Point(r.x  , r.y ),
                Point(r.x + r.width, r.y + r.height),
                CV_RGB(255,255,0),
                3
                );

    for(int i =0; i< keypoints.size().height ; i++){
        Point2f p = keypoints.at<Point2f>(i);

        circle(frame, p , 3, CV_RGB(255,255,0));
    }

}

//};

//HyveFaceTracker* cam;

/*void updateSpiningState(){
    if(currentState == OFF ) {
        cam->setProcessingState(false);
    }else{
        cam->setProcessingState(true);
    }
}*/

/*bool SetState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){

    printf("# facetracking setState service call %d\n" , req.state);
    res.state = req.state;
    if(req.state ==true){
        currentState = TRACKING;
    }else if(currentState == TRACKING){
        currentState = OFF;
    }
    //updateSpiningState();
    return true;

}
bool GetState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    if(currentState == TRACKING ) {
        res.state = true;
    }else{
        res.state =false;
    }
    printf("# facetracking getState service call %d\n" , res.state);
    return true;
}*/


/*bool SetTiredState( hyve_msg::SetState::Request &req ,hyve_msg::SetState::Response  &res  ){
    printf("# facetracking setTiredState service call %d\n" , req.state);
    res.state = req.state;
    if(req.state ==true){
        currentState = TIRED;
    }else if(currentState == TIRED){
        currentState = OFF;
    }
    //updateSpiningState();
    return true;

}
bool GetTiredState( hyve_msg::GetState::Request &req ,hyve_msg::GetState::Response  &res  ){
    if(currentState == TIRED ) {
        res.state = true;
    }else{
        res.state =false;

    }
    printf("# facetracking getTiredState service call %d\n" , res.state);
    return true;
}*/


/*int main(int argc, char **argv)
{

    ros::init(argc, argv, "HyveFaceTracker");


    ros::NodeHandle n("~");

    pub = n.advertise<sensor_msgs::RegionOfInterest>("roi", 1000);

    pubTiredState = n.advertise<std_msgs::String>("Tired/state", 1000);
    pubTiredValue = n.advertise<std_msgs::UInt8>("Tired/value", 1000);

    //cam = new HyveFaceTracker();
    ros::ServiceServer serviceSetter = n.advertiseService("setTrackingState", SetState);
    ros::ServiceServer serviceGetter = n.advertiseService("getTrackingState", GetState);

    ros::ServiceServer serviceTiredSetter = n.advertiseService("setTiredState", SetTiredState);
    ros::ServiceServer serviceTiredGetter = n.advertiseService("getTiredState", GetTiredState);



    //cam->configure();
    //cam->configurePublishing(n);
    //cam->spin();
    //printf("return form spinnded \n");
    return 0;
}*/



PLUGINLIB_DECLARE_CLASS(facetracking_nodelet, HyveFaceTracker, facetracking_nodelet::HyveFaceTracker, nodelet::Nodelet);
}







