#include "video_player/video_player.h"
   
VideoPlayer::VideoPlayer(){
    ros::NodeHandle nh("~");
    timer_ = nh.createTimer(ros::Duration(0.05), &VideoPlayer::timerCallback, this);
    request_pub_ = nh.advertise<video_player_msgs::VideoRequest>("/robotvideo", 10);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publish設定したあと少し待たないとバグる

    now_once_ = true;
    file_name_ = "";
    image_name_ = "";
}

VideoPlayer::~VideoPlayer(){

}

void VideoPlayer::timerCallback(const ros::TimerEvent&){
    
}

void VideoPlayer::setVideo(std::string file_name) {
    file_name_ = file_name;
    ROS_INFO("video info");
}

void VideoPlayer::play() {
    now_once_ = false;
    command_ = 2;
    publishGoal();
    ROS_INFO("play");
}

void VideoPlayer::playOnce() {
    now_once_ = true;
    command_ = 1;
    publishGoal();
    ROS_INFO("play once");
}

void VideoPlayer::restart() {
    command_ = 3;
    publishGoal();
    ROS_INFO("restart");
}

void VideoPlayer::showImage(std::string file_name) {
    command_ = 4;
    image_name_ = file_name;
    publishGoal();
    ROS_INFO("show image");
}

void VideoPlayer::cancel() {
    command_ = 0;
    publishGoal();
    ROS_INFO("cancel");
}

/*
    # Commands
    int8 VIDEO_SET = -1
    int8 PLAY_STOP = 0 # Stop this sound from playing
    int8 PLAY_ONCE = 1 # Play the sound once
    int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs
    int8 PLAY_RESTART = 3
*/
void VideoPlayer::publishGoal() {
    if (file_name_.length() <= 0 && image_name_.length() <= 0) {
        ROS_ERROR("not set file!");
        return;
    }
    video_player_msgs::VideoRequest request;
    request.command = command_;
    request.video_file = file_name_;
    request.image_file = image_name_;
    request_pub_.publish(request);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publishしたあと少し待たないとバグる
    ROS_INFO("request Published");

}
