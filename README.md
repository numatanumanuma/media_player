# media_player

# media_player

2020/10 SZK

### sound_player

~~~
$ rosrun sound_play sndplay_node.py
# プレイヤーノード こいつにリクエストを投げる

$ rosrun sound_play test.py
テスト
~~~

現状曲を重ねがけは出来ず、上書きされてしまう。

[sound_play/SoundRequest Message](http://docs.ros.org/api/sound_play/html/msg/SoundRequest.html)

### video_player

opencvで動画を表示

巻き戻しするとバグる(10/6現在)
