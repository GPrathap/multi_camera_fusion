#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.h>
#include <boost/assign/list_of.hpp>

typedef const boost::function< void(const sensor_msgs::ImageConstPtr &)>  callback;

/*void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view"+ window_name, cv_bridge::toCvShare(msg, "bgr8")->image);
    std::cout<< "image is here" << std::endl;

    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/

class myImages {
public:
    cv::Mat myMatImage1;
    sensor_msgs::ImagePtr myMessagePtr;
    std::string window_name;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg );
};

void myImages::imageCallback(const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        this->myMatImage1 = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("view"+this->window_name, this->myMatImage1);
        cv::waitKey(10);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_listener" + std::string(argv[1]));
  ros::NodeHandle nh;
  myImages MyImagesObj;
  MyImagesObj.window_name = std::string(argv[1]);
  cv::namedWindow("view"+std::string(argv[1]));
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
    callback boundImageCallback = boost::bind(&myImages::imageCallback, &MyImagesObj, _1);
  image_transport::Subscriber sub = it.subscribe("/apollo/sensor/camera/perception/"+std::string(argv[1]), 1, boundImageCallback);
  ros::spin();
  cv::destroyWindow("view"+std::string(argv[1]));
}