#ifndef Video_PLAYER_H
#define Video_PLAYER_H

#include <ros/ros.h>
#include <video_player_msgs/VideoRequest.h>

class VideoPlayer{
public:
    VideoPlayer();
    ~VideoPlayer();
    void timerCallback(const ros::TimerEvent&);
    void setVideo(std::string file_name);
    void play();
    void playOnce();
    void restart();
    void cancel();

    bool playing;
   
private:
    ros::Timer timer_;
    ros::Publisher request_pub_;

    void publishGoal();

    std::string file_name_;
    bool now_once_;
    int command_;
};

#endif
