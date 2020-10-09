#include "video_player/video_player.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_sample");
    //ノード名の初期化

    VideoPlayer player;
    // VideoPlayer宣言したあと少し待つ
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // Videoの設定
    player.setVideo("/home/suzuki-t/.ros/media/sample.mp4");
    // 再生
    player.play();
    for (int i=0; i<5; i++)
        loop_rate.sleep();

    // 停止
    player.cancel();

    // ros::spin();

    return 0;
}