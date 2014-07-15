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

#include "hiwr_opencv_detector.h"

using namespace cv;

namespace hiwr_opencv_detector{

  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  enum currentState {TRACKING,TIRED,OFF};

  int tired_ratio_;

/** Segfault signal handler */
  void sigsegv_handler(int sig)
  {
    signal(SIGSEGV, SIG_DFL);
    fprintf(stderr, "Segmentation fault, stopping uvc camera driver.\n");
    ROS_ERROR("Segmentation fault, stopping uvc camera driver.");
    ros::shutdown();                      // stop the main loop
  }


  HiwrOpencvDetectorNodelet::HiwrOpencvDetectorNodelet() {
    NODELET_DEBUG("[hiwr_opencv_detector Nodelet] Constructor");
  }


  bool HiwrOpencvDetectorNodelet::configure( ){
    std::string default_string;

    printf("ros published \n");
    rospack::Rospack pack ;
    std::vector<std::string> search_path;
    pack.getSearchPathFromEnv(search_path);
    pack.crawl(search_path, false);
    pack.find("hiwr_opencv_detector", default_string) ;

    std::string file;

    private_nh_.getParam("haarfile" , file);
    // ros::param::get("haarfile" , file);
    NODELET_DEBUG("[hiwr_opencv_detector Nodelet][configure] loading file is: %s/  bob  %s", default_string.c_str(), file.c_str());
    if (!cascade_.load(default_string+ '/'+ file)){
        //   if (!cascade.load(defaultString+ '/'+ "lbpcascade_frontalface.xml")){
      NODELET_ERROR("[hiwr_opencv_detector Nodelet][configure] cascade load failed!");
      return false;
    }

    std::string gf_maxCorners_string;
    ros::param::get("~gf_maxCorners", gf_maxCorners_string);
    //	std::cout << "will stoid ";
    //	std::cout << gf_maxCorners_string;
    //gf_maxCorners = std::stoi(gf_maxCorners_string);

    ros::param::get("~gf_qualityLevel", gf_quality_level_);
    ros::param::get("~gf_minDistance", gf_min_distance_);
    ros::param::get("~gf_blockSize", gf_block_size_);
    ros::param::get("~gf_useHarrisDetector", gf_use_harris_detector_);
    ros::param::get("~drop_keypoints_interval", drop_keypoints_interval_);

    ros::param::get("~abs_min_keypoints", abs_min_keypoints_);
    ros::param::get("~std_err_xy", std_err_xy_) ;
    ros::param::get("~pct_err_z", pct_err_z_) ;
    ros::param::get("~max_mse", max_mse_);

    ros::param::get("~add_keypoint_distance", add_keypoint_distance_);
    ros::param::get("~add_keypoints_interval", add_keypoints_interval_);

    ros::param::get("~min_keypoints", min_keypoints_);

    ros::param::param<int>("~tired_value", tired_ratio_, 30);


    ros::param::get("~expand_roi_init", expand_roi_init_);
    expand_roi_ = expand_roi_init_;


    // = rospy.get_param("~gf_qualityLevel", 0.02)
    gf_quality_level_ = 0.02;
    gf_min_distance_  = 2 ;
    gf_block_size_ = 5 ;
    gf_use_harris_detector_ = true; //= rospy.get_param("~gf_useHarrisDetector", true);
    frame_index_ = 0;
    drop_keypoints_interval_ = 2;

    add_keypoint_distance_ = 2 ;
    add_keypoints_interval_ = 1 ;
    min_keypoints_ = 20;

    abs_min_keypoints_ = 6;
    std_err_xy_ = 2.5;
    pct_err_z_ =  0.42;
    max_mse_ = 10000;
    expand_roi_init_ = 1.02;
    expand_roi_ = expand_roi_init_;

    nb_skipping_frames_ = 0;
    skipping_id_ = 0;

    face_buffer_limit_ = 1;

    NODELET_DEBUG("[Facetracking Nodelet][configure] spined %f %d %d \n", gf_quality_level_ ,  gf_min_distance_ ,  gf_max_corners_);
    return true;
  }

  void HiwrOpencvDetectorNodelet::onInit(){
    NODELET_DEBUG("[Facetracking Nodelet][onInit] beginning");

    ros::NodeHandle& public_nh = getNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(public_nh);

    if(!configure()) {
      NODELET_ERROR("[Facetracking Nodelet][onInit] configure failed!");
    }

    if(!private_nh_.getParam("video_stream", video_stream_name_)){
      NODELET_ERROR("[Facetracking Nodelet][onInit] Problem recovering the video stream");
      return;
    }


    frame_.rows = 0;

    pub_ = public_nh.advertise<sensor_msgs::RegionOfInterest>("/hiwr_opencv_detector/roi", 1);
    // recovery of the picture
    image_sub_ = it_->subscribe(video_stream_name_.c_str(), 1,&HiwrOpencvDetectorNodelet::callback, this);
    im_processed_ = false;
    im_available_ = false;

    NODELET_DEBUG("[Facetracking Nodelet][onInit] call thread loop");


    loop_thread_ = std::thread(&HiwrOpencvDetectorNodelet::loop , this);

    NODELET_DEBUG("[Facetracking Nodelet][onInit] end");
  }

// processing the image here
  void HiwrOpencvDetectorNodelet::callback(const sensor_msgs::ImageConstPtr& msg){
    NODELET_DEBUG("[Facetracking nodelet] callback : beginning");

    // recovery of the image
    try{
      im_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        // NODELET_DEBUG("[Facetracking Nodelet] callback : frame type : %d", im_ptr->type());

        // facetracking
        //if(im_processed){
      NODELET_DEBUG("[Facetracking Nodelet] callback : im_ptr ok");

      if(!im_available_){
            // collecting the infos for frame
        frame_ = im_ptr_->image;

        NODELET_DEBUG("[Facetracking Nodelet] callback : frame type : %d", frame_.type());

            // cvtColor(frame, frame, CV_8UC1);

        NODELET_DEBUG("[Facetracking Nodelet] callback : frame type 2 : %d", frame_.type());


        im_available_ = true;
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
  void HiwrOpencvDetectorNodelet::loop(){
    NODELET_DEBUG("Beginning loop");
    //cv::namedWindow("Francis", CV_WINDOW_AUTOSIZE);
    while(ros::ok()){
      if((im_ptr_!=NULL) && im_available_){
        NODELET_DEBUG("[Facetracking Nodelet] loop : display");
            // cvtColor(frame, frame, );
        processTrackingFrame(frame_);
            //cv::imshow("Debug", frame);
            //  NODELET_DEBUG("[Facetracking Nodelet] loop : display end");
            //cv::waitKey(20);
        im_available_ = false;
      }else usleep(100);
    }

    NODELET_DEBUG("End loop");
  }

  void HiwrOpencvDetectorNodelet::processTrackingFrame(Mat frame ) {
    if (detect_box_.width ==0 &&  detect_box_.height==0)
    {
      NODELET_DEBUG("[Facetracking Nodelet] Processing detection");
        // if(skipping_id > nb_skipping_frames){
        //   skipping_id = 0;
      Rect box = detectInitialFaces(frame);




      if(box.width ==0 && box.height== 0){
            //Avoid undefined behaviors
        if(face_buffer_.size() > 0)
          face_buffer_.pop_front();
        return;
      }


        //Display mean face
      Rect mean;
      int  face_size = face_buffer_.size();
      for(int i = 0; i < face_size; i++){
        mean.x+= face_buffer_.at(i).x;
        mean.y += face_buffer_.at(i).y;
        mean.width += face_buffer_.at(i).width;
        mean.height += face_buffer_.at(i).height;
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
        pub_.publish(msg);

      }

      if(face_buffer_.size() < face_buffer_limit_){
        face_buffer_.push_back(box);
        NODELET_INFO("PUSH");
      }
      else{
        NODELET_INFO("POP");
        face_buffer_.pop_front();
        face_buffer_.push_back(box);
      }


      return;
      NODELET_DEBUG("[Facetracking Nodelet] End detection");

      track_box_ = detect_box_;
      keypoints_.release();
      getKeypoints( frame , detect_box_);

      if(keypoints_.size().height<5){
            NODELET_DEBUG("[Facetracking Nodelet] tracking : less than 5 keypoints");    // -> OK
            detect_box_.width =0;
            detect_box_.height =0;
            keypoints_.release(); //empty the list
          }

        }
        else
        {
        //Display keypoints
          for(int i =0 ; i<keypoints_.size().height ; i++ ){
            Point2f point = keypoints_.at<Point2f>(i);
            circle(frame, point, 2, Scalar(0),1);
          }

          NODELET_DEBUG("Processing tracking");
        //Step 2: If we aren't yet tracking keypoints, get them now
          if( keypoints_.size().height==0 ) {
            /*Get point to track*/
            track_box_ = detect_box_;
            getKeypoints( frame , track_box_);
          }

          if(keypoints_.size().height>5){
            // Step 3: If we have keypoints, track them using optical flow
            track_box_ = trackKeypoints(frame.clone(), prev_frame_);

            rectangle(frame, Point(track_box_.x, track_box_.y), Point(track_box_.x+track_box_.width, track_box_.y+track_box_.height), Scalar(0),2);

            //# Step 4: Drop keypoints that are too far from the main cluster
            if (frame_index_ % drop_keypoints_interval_ == 0 and keypoints_.size().height>0){
              frame_index_=0;
              int score = dropKeypoints( abs_min_keypoints_, std_err_xy_, max_mse_);
              if (score == -1){
                detect_box_.width = 0;
                detect_box_.height = 0;
              }
            }
            // Step 5: Add keypoints if the number is getting too low
            if( frame_index_ % add_keypoints_interval_ == 0 and keypoints_.size().height < min_keypoints_){
              expand_roi_ = expand_roi_init_ * expand_roi_ ;
              addKeypoints( frame, track_box_);
            } else {
              frame_index_ += 1 ;
              expand_roi_ = expand_roi_init_;
            }

          }else{
            //
            printf("we got only %d keypont <F4> need to haarify \n", keypoints_.size().height );
            detect_box_.width =0;
            detect_box_.height =0;
            keypoints_.release(); //empty the list
          }

        }

        prev_frame_ =frame.clone();
        NODELET_DEBUG("[Facetracking Nodelet] procees tracking : before drawing");

        if( track_box_.x > 0 && track_box_.y > 0  && track_box_.width >0 && track_box_.height > 0 ) {
          sensor_msgs::RegionOfInterest msg;
          msg.x_offset =track_box_.x;
          msg.y_offset =track_box_.y;
          msg.width =track_box_.width;
          msg.height =track_box_.height;
          pub_.publish(msg);
          frame_index_ ++;
        }

      }

      void HiwrOpencvDetectorNodelet::processTiredFrame(Mat frame ) {
        NODELET_DEBUG("Processing tired frame");
        Mat out(Size(1,1) , CV_8UC1) ;
        cv::resize(frame , out ,  out.size() );

        int current = out.data[0];
        std_msgs::String msg_str;
        std_msgs::UInt8 msg_UInt8;
        msg_UInt8.data = current;
        if(current< tired_ratio_){
          msg_str.data = "tired";
        }else{
          msg_str.data = "dailight";
        }
      }

      void HiwrOpencvDetectorNodelet::processFrame(Mat frame ) {
        NODELET_DEBUG("process frame");

        return processTrackingFrame(frame);
      }

      Rect HiwrOpencvDetectorNodelet::trackKeypoints(Mat img1, Mat img0){
        Mat p0 = keypoints_;
        Mat p0r;
        Mat p1;
        Mat status;
        Mat err;

        calcOpticalFlowPyrLK(img0, img1, p0, p1 , status, err);

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

        keypoints_ = new_keypoints;
        if(keypoints_.size().width == 0 ) {
          track_box_.width =0;
          track_box_.height =0;
        }else{


          if(keypoints_.size().height >6 ){
            track_box_ = fitEllipse(keypoints_).boundingRect();
          }else{
            printf(" we should not be there as this probably introdu a bug later %d  %d \n", keypoints_.size().width , keypoints_.size().height );
            track_box_ = boundingRect(keypoints_);
          }
        }

        return track_box_;
      }

      void HiwrOpencvDetectorNodelet::getKeypoints(Mat input_image, Rect detect_box){

        NODELET_DEBUG("[Facetracking Nodelet] getKeypoints : beginning");
        Mat area = Mat(input_image, detect_box);
        vector<Point2f> output ;
        Mat mask ;
        Mat out ;

        cv::goodFeaturesToTrack(
          area, out, 30, 0.02, 7.0 );

        if ( out.cols > 0){
          keypoints_ = out;
        }

        Mat new_keypoints;
        for(int i =0 ; i<keypoints_.size().height ; i++ ){
          Point2f point = keypoints_.at<Point2f>(i);
          point.x  +=  detect_box_.x ;
          point.y  +=  detect_box_.y ;
          new_keypoints.push_back(point);
        //Display points
          circle(input_image, point, 2, Scalar(0),1);
        }
        keypoints_ = new_keypoints;

        NODELET_DEBUG("[Facetracking Nodelet] getKeypoints : end");

      }

      void HiwrOpencvDetectorNodelet::addKeypoints( Mat input_image,Rect track_box){

        Mat mask = Mat( input_image.size().height ,  input_image.size().width , CV_8UC1, Scalar::all(0));
    int x = track_box.x ; // + detect_box.x ;
    int y = track_box.y ;//+ detect_box.y;
    int h = track_box.height;
    int w = track_box.width;

    // # Expand the track box to look for new keypoints
    int  w_new = expand_roi_ * w;
    int  h_new = expand_roi_ * h;

    Point2f pt1 ;
    pt1.x =  x - w_new/2   + w/2;
    pt1.y =  y - h_new / 2 + h/2 ;

    Point2f pt2 ;
    pt2.x =  x + w_new / 2 + w/2 ;
    pt2.y = y  + h_new / 2 + w/2 ;

    rectangle(input_image, pt1, pt2, Scalar(0),2);

    RotatedRect mask_box ;
    mask_box.center = Point( x +w/2 , y  + h/2);
    mask_box.size = Size(w_new , h_new );
    mask_box.angle=0;

    //# Create a filled white ellipse within the track_box to define the ROI
    ellipse(mask, mask_box, Scalar(255), CV_FILLED ,8);

    for(int i =0; i< keypoints_.size().height ; i++){
      Point2f p = keypoints_.at<Point2f>(i);
      circle(mask, p , 5, 0, -1);
    }

    Mat new_keypoints;

    goodFeaturesToTrack(input_image , new_keypoints, 30, 0.02, 7.0 ,  mask);
    //# Append new keypoints to the current list if they are not
    //# too far from the current cluster
    if(new_keypoints.size().height >  0  ){

      for(int i = 0 ; i<new_keypoints.size().height ; i++ ){
        Point2f point =  new_keypoints.at<Point2f>(i);
        int distance = distanceToCluster(point , keypoints_);
        if (distance > add_keypoint_distance_){
          keypoints_.push_back(point);
        }
      }
    }
  }

  int HiwrOpencvDetectorNodelet::distanceToCluster(Point2f test_point, Mat cluster){
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

  int HiwrOpencvDetectorNodelet::dropKeypoints(int min_keypoints, double outlier_threshold,double mse_threshold){
    int sum_x		= 0;
    int sum_y		= 0;
    double sse	= 0;
    Mat keypoints_xy ;
    int n_xy = keypoints_.size().height;

    //  # If there are no keypoints left to track, start over
    if( n_xy == 0){
      return -1;
    }

    // # Compute the COG (center of gravity) of the cluster
    for(int i=0; i< n_xy ; i++){
      Point2f point = keypoints_.at<Point2f>(i);
      sum_x += point.x;
      sum_y += point.y;
    }
    double mean_x = sum_x / n_xy;
    double mean_y = sum_y / n_xy;


    //# Compute the x-y MSE (mean squared error) of the cluster in the camera plane
    for(int i=0; i< n_xy ; i++){
      Point2f point = keypoints_.at<Point2f>(i);
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
      Point2f point = keypoints_.at<Point2f>(i);
      double std_err = ((point.x - mean_x) * (point.x - mean_x) + (point.y - mean_y) * (point.y - mean_y)) / mse_xy;
      if (std_err > max_err)
        max_err = std_err;
      if (std_err <= outlier_threshold ){
        keypoints_xy.push_back(point);
      };

    } ;

    keypoints_ = keypoints_xy;

    //# Consider a cluster bad if we have fewer than min_keypoints left
    if (keypoints_.size().height < min_keypoints)
      return -1;
    else
      return 1;

  }


/**
     * Detect faces using OpenCV's built-in module          -> OK
     */
     Rect HiwrOpencvDetectorNodelet::detectInitialFaces(Mat& frame)
     {
      double min_face_size=20;
      double max_face_size=200;

      Mat gray;
      NODELET_DEBUG("[Facetracking Nodelet] detectInitialFaces : frame rows : %d", frame.rows);
      NODELET_DEBUG("[Facetracking Nodelet] detectInitialFaces : gray rows : %d", gray.rows);

      equalizeHist( frame, frame);

      cascade_.detectMultiScale( frame, faces_, 1.2, 2,  0|CV_HAAR_SCALE_IMAGE, Size(25, 25), Size(120,120));

    // Draw circles on the detected faces
      for( int i = 0; i < faces_.size(); i++ )
      {
        min_face_size = faces_[0].width*0.8;
        max_face_size = faces_[0].width*1.2;
        Point center( faces_[i].x + faces_[i].width*0.5, faces_[i].y + faces_[i].height*0.5 );
        ellipse( frame, center, Size( faces_[i].width*0.5, faces_[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
      }

      NODELET_DEBUG("[hiwr_opencv_detector Nodelet] detectInitialFaces : number of Rects detected : %d", faces_.size());
      return filterFaces(frame);

    }

    Rect HiwrOpencvDetectorNodelet::getWindow(Rect r){
      Point pt(r.x + r.width/2, r.y + r.height/2);

      int dim = max( r.height , r.width) * 1.5;

      Rect win;
      win.width  = dim;
      win.height = dim;
      win.x      = pt.x - win.width/2;
      win.y      = pt.y - win.height/2;

      return win;
    }

    Rect HiwrOpencvDetectorNodelet::filterFaces(Mat& frame){
      Rect currentBest ;
      if(faces_.size()==0){
        return currentBest;
      }
      currentBest=faces_[0];
    double currentDist = dist(currentBest, frame.cols, frame.rows);//cam_info_.width, cam_info_.height);

Rect current;
for (unsigned int i = 1; i < faces_.size(); i++){
  current = faces_[i];
        double _dist = dist(current, frame.rows, frame.cols);//cam_info_.width, cam_info_.height);
if(_dist<currentDist){
  currentDist = _dist;
  currentBest = current;
}
}

return currentBest;

}

double HiwrOpencvDetectorNodelet::dist(Rect current, int width, int height ){
  return  sqrt( pow(  (current.x + current.width/2) - width/2 , 2 ) + pow( (current.y + current.height/2)-height/2 , 2 ) ) ;
}

void HiwrOpencvDetectorNodelet::drawResult(Mat& frame)
{
  if(detect_box_.x == 0 && detect_box_.y == 0){
    NODELET_DEBUG("[Face tracking Nodelet] drawResult : no box detected");
    return;
  }
  Rect r = track_box_;

  rectangle(
    frame,
                Point(r.x /*+ detect_box.x */ , r.y /*+ detect_box.y*/ ),
                Point(r.x + r.width /*+ detect_box.x */, r.y + r.height /*+detect_box.y */),
    CV_RGB(255,0,0),
    -1
    );

  r = detect_box_;
  rectangle(
    frame,
    Point(r.x  , r.y ),
    Point(r.x + r.width, r.y + r.height),
    CV_RGB(255,255,0),
    3
    );

  for(int i =0; i< keypoints_.size().height ; i++){
    Point2f p = keypoints_.at<Point2f>(i);

    circle(frame, p , 3, CV_RGB(255,255,0));
  }

}


PLUGINLIB_DECLARE_CLASS(hiwr_opencv_detector, HiwrOpencvDetectorNodelet, hiwr_opencv_detector::HiwrOpencvDetectorNodelet, nodelet::Nodelet);
}
