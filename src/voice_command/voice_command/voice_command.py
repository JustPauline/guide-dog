#!/usr/bin/env python3

import json
import threading

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yahboom_msgs.msg import PerfStamp
from vosk import KaldiRecognizer, Model


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voicecommand_node')

        self.cmd_pub = self.create_publisher(String, '/spokencommand', 10)

        # Vosk model & recognizer setup
        model_path = "/root/yahboomcar_ws/vosk-model-small-en-us-0.15"
        self.model = Model(model_path)

        commands = [
            "forward", "backward", "stop",
            "go", "left", "right", "find chair"
        ]
        grammar_json = json.dumps(commands)

        self.rec = KaldiRecognizer(self.model, 48000)
        self.rec.SetWords(True)
        self.rec.SetGrammar(grammar_json)

        # Audio capture
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            input=True,
            frames_per_buffer=8000
        )
        self.stream.start_stream()

        # Start recognition thread
        self.recognition_thread = threading.Thread(target=self.listen_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

    def listen_loop(self):
        while rclpy.ok():
            data = self.stream.read(4000, exception_on_overflow=False)

            if not self.rec.AcceptWaveform(data):
                continue

            # Received a final result
            res = json.loads(self.rec.Result())
            text = res.get('text', '').strip().lower()
            if not text:
                continue

            # Publish recognized command
            cmd_msg = String()
            cmd_msg.data = text
            self.cmd_pub.publish(cmd_msg)

            self.get_logger().info(f"Recognized and published '{text}'")

    def destroy_node(self):
        # Clean up audio
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception:
            pass
        try:
            self.audio.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
