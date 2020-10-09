#include "video_player/video_player_node.h"

#include "video_player/video_player_node.h"

VideoPlayerNode::VideoPlayerNode(){
    ros::NodeHandle nh_private("~");
    timer_ = nh_.createTimer(ros::Duration(0.05), &VideoPlayerNode::timerCallback, this);
    request_sub_ = nh_.subscribe("/robotvideo", 100, &VideoPlayerNode::msgCallback, this);
    file_name_ = "";
    now_frame_ = 0;
    playing = false;
    now_once_ = false;
    window_name_ = "test";
    size_.width = 1920;
    size_.height = 1080;

    ROS_INFO("Start Video Player!!");
}

VideoPlayerNode::~VideoPlayerNode(){

}

void VideoPlayerNode::msgCallback(const video_player_msgs::VideoRequest::ConstPtr& msg){
    /*
    # Commands
    int8 VIDEO_SET = -1
    int8 PLAY_STOP = 0 # Stop this sound from playing
    int8 PLAY_ONCE = 1 # Play the sound once
    int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs
    int8 PLAY_RESTART = 3
    */
    command_ = msg->command;
    file_name_msg_ = msg->arg;
    ROS_INFO("Get Request!");
    if (command_ == -1)
        command_ = 4;
    switch (command_) {
        case 0:
            ROS_INFO("Stop Video");
            stop();
            break;
        case 1:
            ROS_INFO("Play Once Video");
            now_once_ = true;
            rewind();
            play();
            break;
        case 2:
            ROS_INFO("Play Video");
            now_once_ = false;
            rewind();
            play();
            break;
        case 3:
            ROS_INFO("Restart Video");
            play();
            break;
        case 4:
            setVideo(file_name_msg_);
            ROS_INFO("Set Video");
            break;
        default:
            ROS_ERROR("set=-1,stop=0,noce=1,start=2,restart=3");
            break;
    }

}

void VideoPlayerNode::timerCallback(const ros::TimerEvent&){
    if (playing) {
        capture();
        show();
    }
}

void VideoPlayerNode::startCapturing(){
    capture_.open(file_name_);
    if (!capture_.isOpened())
        ROS_ERROR("Error. Falid to open videofile.");
    else
        ROS_INFO("Start capturing.");
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, size_.width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, size_.height);
}

void VideoPlayerNode::setVideo(std::string file_name) {
    if (file_name == ""){
        ROS_ERROR("Not set video!!");
        return;
    }
    if (file_name != file_name_){
        file_name_ = file_name;
        startCapturing();
        end_frame_ = (int)capture_.get(cv::CAP_PROP_FRAME_COUNT);   // 全フレーム数を取得
    }
}

void VideoPlayerNode::play() {
    if (file_name_ == ""){
        setVideo(file_name_msg_);
    }
    playing = true;
    capture_.set(cv::CAP_PROP_POS_FRAMES, now_frame_);
    // ROS_INFO("video start");
}

void VideoPlayerNode::stop() {
    if (playing){
        now_frame_ = (int)capture_.get(cv::CAP_PROP_POS_FRAMES); // フレームの位置を取得
        cv::destroyWindow(window_name_);
        cv::waitKey(1);
        playing = false;
        // ROS_INFO("video stop");
    }
}

void VideoPlayerNode::rewind() {
    now_frame_ = 0;
    capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
    // ROS_INFO("video rewind");
}

void VideoPlayerNode::capture(){
    capture_ >> frame_;
    if (frame_.empty()) {
        rewind();
        if(now_once_)
            stop();
    }
}

void VideoPlayerNode::show(){
    cv::imshow( window_name_, frame_);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_player_node");
    //ノード名の初期化

    VideoPlayerNode player;

    ros::spin();

    return 0;
}