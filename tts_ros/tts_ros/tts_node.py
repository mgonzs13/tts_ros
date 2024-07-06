#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

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


import os
import wave
import pyaudio
import tempfile
import threading
import collections
import numpy as np
from TTS.api import TTS as TtsModel

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from audio_common_msgs.msg import AudioStamped
from audio_common_msgs.action import TTS
from audio_common.utils import data_to_msg
from audio_common.utils import get_msg_chunk


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_node")

        self.declare_parameters("", [
            ("chunk", 4096),
            ("frame_id", ""),

            ("model", "tts_models/en/ljspeech/vits"),
            ("model_path", ""),
            ("config_path", ""),
            ("vocoder_path", ""),
            ("vocoder_config_path", ""),
            ("device", "cpu"),

            ("speaker_wav", ""),
            ("speaker", ""),
            ("stream", False),
        ])

        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        model = self.get_parameter(
            "model").get_parameter_value().string_value
        model_path = self.get_parameter(
            "model_path").get_parameter_value().string_value
        config_path = self.get_parameter(
            "config_path").get_parameter_value().string_value
        vocoder_path = self.get_parameter(
            "vocoder_path").get_parameter_value().string_value
        vocoder_config_path = self.get_parameter(
            "vocoder_config_path").get_parameter_value().string_value

        self.speaker_wav = self.get_parameter(
            "speaker_wav").get_parameter_value().string_value
        self.speaker = self.get_parameter(
            "speaker").get_parameter_value().string_value
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value
        self.stream = self.get_parameter(
            "stream").get_parameter_value().bool_value

        self.tts = TtsModel(
            model_name=model,
            model_path=model_path,
            config_path=config_path,
            vocoder_path=vocoder_path,
            vocoder_config_path=vocoder_config_path
        ).to(self.device)

        if (
            not self.speaker_wav or
            not (
                os.path.exists(self.speaker_wav) and
                os.path.isfile(self.speaker_wav)
            )
        ):
            self.speaker_wav = None

        if not self.speaker or not self.tts.is_multi_speaker:
            self.speaker = None

        # goal queue
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        # publish audio data
        self._pub_rate = None
        self._pub_lock = threading.Lock()
        self.__player_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

        # action server
        self._action_server = ActionServer(
            self,
            TTS,
            "say",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().debug(f"Stream: {self.stream} {self.speaker_wav}")

        if self.stream and self.speaker_wav:
            self.get_logger().info(
                f"Streaming. Getting embeddings from {self.speaker_wav}")
            self.gpt_cond_latent, self.speaker_embedding = self.tts.synthesizer.tts_model.get_conditioning_latents(
                audio_path=[self.speaker_wav])
        else:
            self.stream = False

        self.get_logger().info("TTS node started")

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request: ServerGoalHandle) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_queue_lock:
            if self._current_goal is not None:
                self._goal_queue.append(goal_handle)
            else:
                self._current_goal = goal_handle
                self._current_goal.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> TTS.Result:

        request: TTS.Goal = goal_handle.request
        text = request.text
        language = request.language

        # check text
        if not text.strip():
            goal_handle.succeed()
            return TTS.Result()

        # Stream values, might be specific to the model?
        audio_format = pyaudio.paInt16
        channels = 1
        rate = 24000

        if not self.tts.is_multi_lingual:
            language = None

        # generate the audio from text
        self.get_logger().info("Generating Audio")

        try:
            if not self.stream:
                # create an audio file
                audio_file = tempfile.NamedTemporaryFile(mode="w+")
                self.tts.tts_to_file(
                    text,
                    speaker_wav=self.speaker_wav,
                    speaker=self.speaker,
                    language=language,
                    file_path=audio_file.name
                )

                # read audio chunks
                audio_file.seek(0)
                wf = wave.open(audio_file.name, "rb")
                audio_file.close()

                audio_format = pyaudio.get_format_from_width(wf.getsampwidth())
                channels = wf.getnchannels()
                rate = wf.getframerate()

                def read_wav_chunks():
                    while True:
                        frames = wf.readframes(self.chunk)
                        if not frames:
                            break
                        yield frames

                chunks = read_wav_chunks()

            else:
                # TODO: TTS/tts/layers/xtts/stream_generator.py:138: UserWarning: You have modified the pretrained model configuration to control generation. This is a deprecated strategy to control generation and will be removed soon, in a future version. Please use a generation configuration file (see https://huggingface.co/docs/transformers/main_classes/text_generation)
                chunks = self.tts.synthesizer.tts_model.inference_stream(
                    text,
                    speaker_wav=self.speaker_wav,
                    speaker=self.speaker,
                    language=language,
                    gpt_cond_latent=self.gpt_cond_latent,
                    speaker_embedding=self.speaker_embedding,
                    stream_chunk_size=self.chunk,
                    temperature=0.65,
                    repetition_penalty=10.0,
                    speed=1.0,
                    enable_text_splitting=True,
                )

        except Exception as e:
            self.get_logger().error(
                f"Exception '{e}' when processing text '{text}'")
            goal_handle.abort()
            self.run_next_goal()
            return TTS.Result()

        # pub audio chunks
        with self._pub_lock:
            self.run_next_goal()

            self.get_logger().info("Publishing Audio")
            frequency = rate / self.chunk

            if self._pub_rate is None:
                self._pub_rate = self.create_rate(frequency)

            for _, chunk_data in enumerate(chunks):
                for j in range(0, len(chunk_data), self.chunk):

                    if self.stream:
                        data = chunk_data[j:j+self.chunk]
                        data = data.clone().detach().cpu().numpy()
                        data = data[None, : int(data.shape[0])]
                        data = np.clip(data, -1, 1)
                        data = (data * 32767).astype(np.int16)

                    else:
                        data = chunk_data

                    audio_msg = data_to_msg(data, audio_format)
                    if audio_msg is None:
                        self.get_logger().error(
                            f"Format {audio_format} unknown")
                        self._goal_handle.abort()
                        return TTS.Result()

                    msg = AudioStamped()
                    msg.header.frame_id = self.frame_id
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.audio = audio_msg
                    msg.audio.info.channels = channels
                    msg.audio.info.chunk = get_msg_chunk(audio_msg)
                    msg.audio.info.rate = rate

                    if not goal_handle.is_active:
                        return TTS.Result()

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return TTS.Result()

                    self.__player_pub.publish(msg)
                    self._pub_rate.sleep()

                    if not self.stream:
                        break

            result = TTS.Result()
            result.text = text
            goal_handle.succeed()
            return result

    def run_next_goal(self) -> bool:
        with self._goal_queue_lock:
            try:
                self._current_goal = self._goal_queue.popleft()
                t = threading.Thread(target=self._current_goal.execute)
                t.start()
                return True

            except IndexError:
                self._current_goal = None
                return False


def main():
    rclpy.init()
    node = AudioCapturerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
