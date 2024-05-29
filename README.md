# tts_ros

This repositiory integrates the Python [TTS](https://pypi.org/project/TTS/) (Text-to-Speech) pacakge into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common).

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

## Usage

To use this tool you have to run the tts_node. It has the following parameters:

- `chunk`: Size of audio chunks to be sent to the audio player.
- `frame_id`: Frame of for the tts.
- `model`: The tts model. You can check the available models with `tts --list_models`.
- `model_path`: Path to a local model file.
- `config_path`: Path to a config file.
- `vocoder_path`: Path to a vocoder model file.
- `vocoder_config_path`: Path to a config file.
- `device`: The device to run the model same as in torch.
- `speaker_wav`: The wav file to perform voice cloning.
- `speaker`: Which speaker voice to use for multi-speaker models. Check with `tts --model_name <model> --list_language_idx`.
- `stream`: Whether to stream the audio data.

### Format

```shell
$ ros2 run tts_ros tts_node --ros-args -p chunk:=4096 -p frame_id:="your-frame" -p model:="your-model" -p device:="cpu/cuda" -p speaker_wav:="/path/to/wav/file" -p stream:=False
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
