//************************************************************************************************************************************//
// Node to return depth information from Kinect RGB-D.
// From Given Obstacle Avoidance C++ file
// Edited by: Darius Tan
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>
#include <tr1/tuple>

namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "RGB Camera";
static const char dWINDOW[] = "Depth Camera";

//********************** Image Converter Class **********************//

class ImageConverter
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher depth_pub;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    ImageSubscriber rgb_image_sub_, depth_image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cv::Mat rgbImageMat, imageMat_canny;
    float big_f, small_l;

  public:
    ImageConverter(ros::NodeHandle &nh): 
        it_(nh_),
        rgb_image_sub_(it_, "/camera/rgb/image_raw", 1),
        depth_image_sub_(it_, "/camera/depth/image_raw", 1),
    
    sync (MySyncPolicy(10), rgb_image_sub_, depth_image_sub_)
    {
        depth_pub = nh_.advertise<std_msgs::Float64>("depth_info", 1);
        sync.registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &rgbmsg, 
                 const sensor_msgs::ImageConstPtr &depthmsg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        /*
         *   For displaying depth images
         */
        // depth measurement is only accurate from 0.4m < 2.26m.
        // if < 0.4, nan. if > 2.26, always 2.26

        try
        {
            cv_ptr = cv_bridge::toCvShare(depthmsg, enc::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // To get the depth at the optical centre (320,240)px
        std_msgs::Float64 msg;
        msg.data = cv_ptr->image.at<float>(320, 240);
        
        // Publisher format -->
        // data: 2.26199245453
        depth_pub.publish(msg);
        std::cout << " depth at centre " << cv_ptr->image.at<float>(320, 240) << std::endl;

        cv::imshow(dWINDOW, cv_ptr->image);
        
        /*
         *  For displaying RGB images
         */
        try
        {
            cv_ptr = cv_bridge::toCvShare(rgbmsg, enc::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow(WINDOW, cv_ptr->image);

        cv::waitKey(3);
    }
};

//********************** MAIN FUNCTION **********************//
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_info");
  ros::NodeHandle nh;
  ImageConverter ic(nh);

  ros::spin();
  return 0;
}