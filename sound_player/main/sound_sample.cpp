#include "sound_player/sound_player.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_sample");
    //ノード名の初期化

    SoundPlayer player;
    
    player.setSound("/home/suzuki-t/.ros/media/sample.wav");
    player.play();

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    loop_rate.sleep();
    loop_rate.sleep();
    loop_rate.sleep();
    player.cancel();
    loop_rate.sleep();
    loop_rate.sleep();
    player.say("Hello");

    // ros::spin();

    return 0;
}