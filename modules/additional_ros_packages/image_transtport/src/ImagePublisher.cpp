#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>
#include <glob.h>
#include <vector>

using std::vector;

vector<std::string> globVector(const std::string& pattern){
    glob_t glob_result;
    glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::cout<< pattern.c_str() <<std::endl;
    vector<std::string> files;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        files.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_front_left = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_front_left_side", 1);
    ros::Publisher pub_front_right = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_front_right_side", 1);
    ros::Publisher pub_left = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_left", 1);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_right", 1);
    ros::Publisher pub_left_backwards = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_left_backwards", 1);
    ros::Publisher pub_right_backwards = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_right_backwards", 1);
    ros::Publisher pub_front = nh.advertise<sensor_msgs::Image>("/apollo/sensor/camera/perception/image_front_camera", 1);


    vector<std::string> files_left = globVector(std::string(argv[1])+"/*");
    vector<std::string> files_right = globVector(std::string(argv[2])+"/*");

    std::cout<< "Number of images to be sent: " << files_left.size() << std::endl;


    ros::Rate loop_rate(10);
    unsigned int i=0;
    while (nh.ok()) {
        std::cout<< files_left.at(i) << std::endl;
        std::cout<< files_right.at(i) << std::endl;

        cv::Mat image_left = cv::imread(files_left.at(i), CV_LOAD_IMAGE_COLOR);
        cv::Mat image_right = cv::imread(files_right.at(i), CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left).toImageMsg();
        sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();

        pub_front_left.publish(msg_left);
        pub_left.publish(msg_left);
        pub_left_backwards.publish(msg_left);
        pub_front_right.publish(msg_right);
        pub_right.publish(msg_right);
        pub_right_backwards.publish(msg_right);
        pub_front.publish(msg_right);

        ros::spinOnce();
        loop_rate.sleep();
        i++;
        if(i==files_left.size()){
            i = 0;
        }
    }
}

