#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "video_player_msgs/VideoRequest.h"

class VideoPlayerNode{
public:
    VideoPlayerNode();
    ~VideoPlayerNode();
    void msgCallback(const video_player_msgs::VideoRequest::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);
    void setVideo(std::string file_name);
    void play();
    void showImage(std::string file_name);
    void stop();
    void rewind();
    
    bool playing;
    bool image_showing;
   
private:
    ros::Timer timer_;
    ros::NodeHandle nh_;

    ros::Subscriber request_sub_;

    cv::VideoCapture capture_;
    cv::Mat frame_;
    std::string file_name_;
    bool now_once_;
    int now_frame_;
    int end_frame_;
    cv::Size size_;
    std::string window_name_;

    cv::Mat	image_;

    int command_;
    std::string file_name_msg_;
    std::string image_msg_;

    void startCapturing();
    void capture();
    void show();
    void createWindow();

};

#endif