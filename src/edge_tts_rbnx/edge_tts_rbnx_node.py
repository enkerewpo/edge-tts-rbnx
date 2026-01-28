#!/usr/bin/env python3
"""
Edge TTS ROS2 Node for Robonix
Provides text-to-speech primitive capability using Microsoft Edge TTS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import edge_tts
import asyncio
import subprocess
import tempfile
import os
import logging
from pydub import AudioSegment
import io


class EdgeTTSNode(Node):
    def __init__(self):
        super().__init__('edge_tts_rbnx')
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.logger = logging.getLogger(__name__)
        
        # Parameters
        self.declare_parameter('voice', 'zh-CN-XiaoyiNeural')
        self.declare_parameter('volume', '100%')
        self.declare_parameter('device', '')
        
        self.voice = self.get_parameter('voice').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        
        # Primitive input/output topics (must match rbnx_manifest input_schema/output_schema)
        text_topic = 'speech/tts/text'
        status_topic = 'speech/tts/status'
        self.sub_text = self.create_subscription(
            String, text_topic, self.text_callback, 10
        )
        self.pub_status = self.create_publisher(Bool, status_topic, 10)
        self.logger.info(
            f"Edge TTS Node initialized: voice={self.voice}, volume={self.volume}, "
            f"subscribed to '{text_topic}', publishing to '{status_topic}'"
        )
        if self.device:
            self.logger.info(f"Using audio device: {self.device}")
    
    def text_callback(self, msg):
        """Handle text input and synthesize speech"""
        text = msg.data.strip()
        preview = (text[:80] + '...') if len(text) > 80 else text
        self.logger.info(f"[prm] Received text on speech/tts/text, len={len(text)}, preview='{preview}'")
        if not text:
            self.logger.warning("[prm] Empty text received, publishing status=False")
            self._publish_status(False)
            return
        
        # Run async TTS in executor
        try:
            self.logger.info("[prm] Starting TTS synthesis...")
            asyncio.run(self.speak_text(text))
            self.logger.info("[prm] TTS finished successfully, publishing status=True")
            self._publish_status(True)
        except Exception as e:
            self.logger.error(f"[prm] TTS failed: {e}", exc_info=True)
            self._publish_status(False)
    
    def _publish_status(self, success: bool):
        """Publish status message"""
        msg = Bool()
        msg.data = success
        self.pub_status.publish(msg)
    
    async def speak_text(self, text: str):
        """Synthesize and play text using Edge TTS"""
        self.logger.info(f"[prm] Starting TTS: text_len={len(text)}, voice={self.voice}, volume={self.volume}")
        
        # Get audio bytes from Edge TTS
        audio_bytes = b""
        try:
            async for chunk in edge_tts.Communicate(text, self.voice).stream():
                if chunk["type"] == "audio":
                    audio_bytes += chunk["data"]
        except Exception as e:
            self.logger.error(f"Failed to synthesize audio: {e}")
            raise
        
        if not audio_bytes:
            self.logger.error("No audio data received")
            raise RuntimeError("No audio data received from TTS")
        
        self.logger.info(f"[prm] Received {len(audio_bytes)} bytes of audio data")
        
        # Convert MP3 to WAV
        try:
            audio = AudioSegment.from_file(io.BytesIO(audio_bytes), format="mp3")
            self.logger.info(f"Audio duration: {len(audio)}ms ({len(audio)/1000:.2f}s)")
        except Exception as e:
            self.logger.error(f"Failed to convert audio: {e}")
            raise
        
        # Save to temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
            tmp_path = tmp_file.name
            audio.export(tmp_path, format="wav")
        
        self.logger.debug(f"Audio saved to temporary file: {tmp_path}")
        
        try:
            # Configure PulseAudio if available
            self._configure_pulseaudio()
            
            # Play audio
            self._play_audio(tmp_path)
        finally:
            # Clean up temporary file
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
                self.logger.debug("Temporary file deleted")
    
    def _configure_pulseaudio(self):
        """Configure PulseAudio sink"""
        try:
            # Get default sink if device not specified
            if not self.device:
                result = subprocess.run(
                    ["pactl", "list", "short", "sinks"],
                    capture_output=True, text=True, timeout=5
                )
                if result.returncode == 0 and result.stdout.strip():
                    sink_id = result.stdout.strip().split('\n')[0].split('\t')[0]
                    self.device = sink_id
            
            if self.device:
                subprocess.run(
                    ["pactl", "set-sink-port", self.device, "analog-output-speaker"],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, timeout=5
                )
                subprocess.run(
                    ["pactl", "set-sink-mute", self.device, "0"],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, timeout=5
                )
                subprocess.run(
                    ["pactl", "set-sink-volume", self.device, self.volume],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, timeout=5
                )
                self.logger.info(f"[prm] PulseAudio sink {self.device} configured: volume={self.volume}")
        except Exception as e:
            self.logger.debug(f"Failed to configure PulseAudio: {e}")
    
    def _play_audio(self, audio_path: str):
        """Play audio file using paplay or aplay"""
        # Try paplay first
        try:
            result = subprocess.run(
                ["paplay", audio_path],
                capture_output=True, text=True, timeout=30
            )
            if result.returncode == 0:
                self.logger.info("[prm] Audio played successfully with paplay")
                return
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
            self.logger.debug(f"paplay failed: {e}, trying aplay...")
        
        # Fallback to aplay
        try:
            if self.device and ',' in self.device:
                # ALSA device format: card,device
                device_str = f"plughw:{self.device}"
                subprocess.run(
                    ["aplay", "-D", device_str, audio_path],
                    check=True, timeout=30,
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
                self.logger.info(f"Audio played successfully with aplay (device {device_str})")
            else:
                subprocess.run(
                    ["aplay", audio_path],
                    check=True, timeout=30,
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
                self.logger.info("Audio played successfully with aplay (default device)")
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            self.logger.error(f"All playback methods failed: {e}")
            raise


def main(args=None):
    rclpy.init(args=args)
    node = EdgeTTSNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
