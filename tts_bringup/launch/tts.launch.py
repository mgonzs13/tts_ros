# MIT License

# Copyright (c) 2025 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tts_ros",
                executable="tts_node",
                name="tts_node",
                parameters=[
                    {
                        "chunk": LaunchConfiguration("chunk", default=4096),
                        "frame_id": LaunchConfiguration("frame_id", default=""),
                        "model": LaunchConfiguration(
                            "model", default="tts_models/en/ljspeech/vits"
                        ),
                        "model_path": LaunchConfiguration("model_path", default=""),
                        "config_path": LaunchConfiguration("config_path", default=""),
                        "vocoder_path": LaunchConfiguration("vocoder_path", default=""),
                        "vocoder_config_path": LaunchConfiguration(
                            "vocoder_config_path", default=""
                        ),
                        "device": LaunchConfiguration("device", default="cpu"),
                        "speaker_wav": LaunchConfiguration("speaker_wav", default=""),
                        "speaker": LaunchConfiguration("speaker", default=""),
                        "stream": LaunchConfiguration("stream", default=False),
                    }
                ],
                remappings=[("audio", "audio/out")],
            ),
            Node(
                package="audio_common",
                executable="audio_player_node",
                name="player_node",
                namespace="audio",
                parameters=[
                    {
                        "channels": LaunchConfiguration("channels", default=2),
                    }
                ],
                remappings=[("audio", "out")],
                condition=IfCondition(
                    PythonExpression(
                        [LaunchConfiguration("launch_audio_player", default=True)]
                    )
                ),
            ),
        ]
    )
