//************************************************************************************************************************************//
// From Given Obstacle Avoidance cpp file
// Edited by: Darius Tan
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
//#include "/home/willson/catkin_workspace/devel/include/cmd_vel_msgs/status.h"

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
#include <tr1/tuple> // if gcc > 4.7 need to use #include <tuple> // std::tuple, std::make_tuple, std::tie

// namespace patch{
//     template < typename T > std::string to_string(const T& n){
//         std::ostringstream stm;
//         stm << n;
//         return stm.str();
//     }
// }

namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "RGB Camera";
static const char dWINDOW[] = "Depth Camera";
// static const char gWINDOW[] = "Canny image";

//********************** For Edge Detection **********************//

// std::tr1::tuple<cv::Mat, std::vector<float>> computeVLines(cv::Mat imageMat_after)
// {
//     cv::Mat err, gimageMat_canny, imageMat_canny;
//     std::vector<float> Pt;
//     std::tr1::tuple<cv::Mat, std::vector<float>> cannyReturn;

//     Canny(imageMat_after, gimageMat_canny, 7, 40, 3);
//     cvtColor(gimageMat_canny, imageMat_canny, CV_GRAY2BGR);

//     cv::imshow("Canny Image", gimageMat_canny);

//     std::vector<cv::Vec2f> lines;
//     HoughLines(gimageMat_canny, lines, 1, CV_PI / 180, 100);

//     for (size_t i = 0; i < lines.size(); i++)
//     {
//         float rho = lines[i][0];
//         float theta = lines[i][1];
//         if (theta == 0)
//         {
//             double a = cos(theta), b = sin(theta);
//             double x0 = a * rho, y0 = b * rho;

//             cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
//             cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));

//             line(imageMat_canny, pt1, pt2, cv::Scalar(0, 0, 255), 2, 8);
//             Pt.push_back(pt1.x);
//         }
//     }

//     cannyReturn = std::tr1::make_tuple(imageMat_canny, Pt);
//     return (cannyReturn);
// }

//********************** Image Converter Class **********************//

class ImageConverter
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher depth_pub; //added this

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
        // cv::moveWindow(dWINDOW, 730, 20); // makes the window unable to move
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
        // cv::moveWindow(WINDOW, 20, 20); // makes the window unable to move
        /*
         *  To identify the vertical edges in the RGB image
         */
        // rgbImageMat = cv_ptr->image;

        // std::tr1::tuple<cv::Mat, std::vector<float>> cannyReturn;
        // std::vector<float> pointReturn;

        // cv::Size rgb_size_ = rgbImageMat.size();

        // if (rgb_size_.height > 0 && rgb_size_.width > 0)
        // {
        //     cannyReturn = computeVLines(rgbImageMat);
        //     std::tr1::tie(imageMat_canny, pointReturn) = cannyReturn;

        //     if (pointReturn.size() > 0)
        //     {
        //         std::vector<float> big, small;
        //         big_f = 640;
        //         small_l = 0;

        //         for (int i = 0; i < pointReturn.size(); i++)
        //         {

        //             if (pointReturn[i] > 320)
        //             {
        //                 big.push_back(pointReturn[i]);
        //             }
        //             else
        //             {
        //                 small.push_back(pointReturn[i]);
        //             }
        //         }
        //         if (big.size() > 0)
        //         {
        //             sort(big.begin(), big.end());
        //             // big_f = big[0];
        //             big_f = big[big.size() - 1];
        //         }
        //         if (small.size() > 0)
        //         {
        //             sort(small.begin(), small.end());
        //             //small_l = small[small.size()-1];
        //             small_l = small[0];
        //         }
        //     }
        //     std::cout << "Right edge " << big_f << " "
        //               << " Left edge " << small_l << std::endl;
        // }


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