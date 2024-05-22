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
import time
import wave
import pyaudio
import tempfile
import threading
from TTS.api import TTS as TtsModel
import numpy as np

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


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_node")

        self.declare_parameters("", [
            ("chunk", 4096),
            ("frame_id", ""),

            ("model", "tts_models/en/ljspeech/vits"),
            ("device", "cpu"),

            ("speaker_wav", ""),
            ("speaker", ""),
            ("stream", False),
        ])

        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value
        self.speaker_wav = self.get_parameter(
            "speaker_wav").get_parameter_value().string_value
        self.speaker = self.get_parameter(
            "speaker").get_parameter_value().string_value
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value
        self.stream = self.get_parameter(
            "stream").get_parameter_value().bool_value

        self.tts = TtsModel(self.model).to(self.device)

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

        self.player_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

        # action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
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
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> TTS.Result:

        request: TTS.Goal = goal_handle.request

        start_time = time.time()

        text = request.text
        language = request.language

        if not self.tts.is_multi_lingual:
            language = None

        if not self.stream:
            # create audio file
            audio_file = tempfile.NamedTemporaryFile(mode="w+")
            self.tts.tts_to_file(
                text,
                speaker_wav=self.speaker_wav,
                speaker=self.speaker,
                language=language,
                file_path=audio_file.name
            )

            # pub audio
            audio_file.seek(0)
            wf = wave.open(audio_file.name, "rb")
            audio_file.close()
            audio_format = pyaudio.get_format_from_width(wf.getsampwidth())

            frequency = wf.getframerate() / self.chunk
            pub_rate = self.create_rate(frequency)

            # send audio data
            data = wf.readframes(self.chunk)

            while data:
                pub_rate.sleep()

                if not goal_handle.is_active:
                    return TTS.Result()

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return TTS.Result()

                audio_data_msg = data_to_msg(data, audio_format)
                if audio_data_msg is None:
                    self.get_logger().error(f"Format {audio_format} unknown")
                    self._goal_handle.abort()
                    return TTS.Result()

                msg = AudioStamped()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.audio.audio_data = audio_data_msg
                msg.audio.info.format = audio_format
                msg.audio.info.channels = wf.getnchannels()
                msg.audio.info.chunk = self.chunk
                msg.audio.info.rate = wf.getframerate()

                self.player_pub.publish(msg)

                data = wf.readframes(self.chunk)

        else:
            # Stream chunks
            # These values might be specific to the model?
            audio_format = pyaudio.paInt16
            # audio_format = pyaudio.paFloat32
            rate = 24000
            channels = 1
            frequency = rate / self.chunk
            pub_rate = self.create_rate(frequency)

            self.get_logger().debug(f"Streaming chunks")

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

            for i, chunk_data in enumerate(chunks):
                for j in range(0, len(chunk_data), self.chunk):
                    data = chunk_data[j:j+self.chunk]

                    self.get_logger().debug(
                        f"Streaming chunk number {i} subchunk {j}")

                    if not goal_handle.is_active:
                        return TTS.Result()

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return TTS.Result()

                    data = data.clone().detach().cpu().numpy()
                    data = data[None, : int(data.shape[0])]
                    data = np.clip(data, -1, 1)
                    data = (data * 32767).astype(np.int16)

                    audio_data_msg = data_to_msg(data.tobytes(), audio_format)
                    if audio_data_msg is None:
                        self.get_logger().error(
                            f"Format {audio_format} unknown")
                        self._goal_handle.abort()
                        return TTS.Result()

                    msg = AudioStamped()
                    msg.header.frame_id = self.frame_id
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.audio.audio_data = audio_data_msg
                    msg.audio.info.format = audio_format
                    msg.audio.info.channels = channels
                    msg.audio.info.chunk = self.chunk
                    msg.audio.info.rate = rate

                    self.player_pub.publish(msg)
                    end_time = time.time()
                    self.get_logger().debug(
                        f"Text to speech chunk {i} {end_time - start_time} seconds")
                    pub_rate.sleep()

        pub_rate.sleep()
        end_time = time.time()
        self.get_logger().debug(
            f"Text to speech took {end_time - start_time} seconds")

        goal_handle.succeed()
        return TTS.Result()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
