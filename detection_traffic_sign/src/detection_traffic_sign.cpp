#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "detection_traffic_sign/red.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class Image{
    private:
        ros::NodeHandle nh;

        image_transport::ImageTransport it;
        image_transport::Subscriber sub;
        image_transport::Publisher pub;

        ros::Publisher pub_stop;
        
        Mat original_img;
        Mat RoI_img;
        Mat hsv_img;
        Mat red_mask;
        Mat red_image;

        int TotalNumberOfPixels;
        int ZeroPixels;
        int NonZeroPixels;

    public:
        Image();
        ~Image();

        void msgCallback_image(const sensor_msgs::ImageConstPtr& msg);
        void CreateRoI(Mat original_img);
        void ConverterColor(Mat RoI_img);
        int CountZero(Mat red_mask);
        int redDetection(int ZeroPixels);
};

int main(int argc, char **argv){
    
    ros::init(argc, argv, "image_converter");

    Image image;

    ros::Rate rate(100);
    
    while(ros::ok()){
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

Image::Image() : it(nh){

    // Subscribe to input video feed and publish output video feed
    sub = it.subscribe("/carla/ego_vehicle/rgb_view_front/image", 1, &Image::msgCallback_image, this);
    // pub = it.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
}

Image::~Image(){
    cv::destroyWindow(OPENCV_WINDOW);
}

void Image::msgCallback_image(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
    // pub.publish(cv_ptr -> toImageMsg());
    
    original_img = cv_ptr->image;
    CreateRoI(original_img);
}

void Image::CreateRoI(Mat original_img){

    // // rectangle
    original_img.copyTo(RoI_img);
    rectangle(RoI_img, Rect(250,200,200,300), Scalar(0,0,255), 2, 8, 0);

    RoI_img = original_img(Rect(250,200,200,300));

    // // Update GUI Window
    cv::imshow(OPENCV_WINDOW, RoI_img);
    cv::waitKey(3);

    ConverterColor(RoI_img);
}

void Image::ConverterColor(Mat RoI_img){

    cvtColor(RoI_img, hsv_img, COLOR_BGR2HSV);

    Scalar lower_red = Scalar(0, 20, 100);
    Scalar upper_red = Scalar(10, 255, 255);

    inRange(hsv_img, lower_red, upper_red, red_mask);
    // bitwise_and(RoI_img, RoI_img, red_image, red_mask);

    // cv::imshow(OPENCV_WINDOW, red_mask);
    // cv::waitKey(3);

    CountZero(red_mask);
}

int Image::CountZero(Mat red_mask){

    TotalNumberOfPixels = red_mask.rows * red_mask.cols;
    // ZeroPixels = TotalNumberOfPixels - countNonZero(red_mask);
    NonZeroPixels = countNonZero(red_mask);

    std::cout << "The number of pixels that are zero is " << NonZeroPixels << std::endl;

    redDetection(NonZeroPixels);

    return 0;
}

int Image::redDetection(int NonZeroPixels){

    pub_stop = nh.advertise<detection_traffic_sign::red>("stop", 1);
    detection_traffic_sign::red msg; // 메세지 변수 선언
    
    if(NonZeroPixels > 9 && NonZeroPixels < 100){
        std::cout << "Red" << std::endl;
        msg.red = 1;
        pub_stop.publish(msg);
    }
    else{
        std::cout<< "Pass" << std::endl;
        msg.red = 0;
        pub_stop.publish(msg);
    }

    return 0;
}

