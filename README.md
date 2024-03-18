# tts_ros

This repositiory integrates the Python [TTS](https://pypi.org/project/TTS/) pacakge into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common).

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone https://github.com/mgonzs13/tts_ros.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r tts_ros/requirements.txt
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```

## Demos

```shell
$ ros2 run tts_ros tts_node
```

```shell
$ ros2 run audio_common audio_player_node
```

```shell
$ ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}"
```
