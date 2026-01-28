#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Speak Skill
#
# Skill that wraps prm::speech.tts for RTDL. Subscribes to skill start,
# resolves TTS primitive topics via QueryPrimitive, publishes text to primitive
# and reports skill status when TTS completes.
"""Speak skill: TTS primitive wrapper for skill-only (RTDL) invocation."""

import json
import signal
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from robonix_sdk.srv import QueryPrimitive


class SpeakSkill(Node):
    """Implements speak skill that invokes prm::speech.tts primitive."""

    def __init__(self):
        super().__init__("speak_skill")

        self.start_topic = "/robot1/skill/speak/start"
        self.status_topic = "/robot1/skill/speak/status"

        self.tts_text_topic = None
        self.tts_status_topic = None

        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        service_qos.deadline = Duration(seconds=0)
        service_qos.lifespan = Duration(seconds=0)
        service_qos.liveliness = LivelinessPolicy.AUTOMATIC
        service_qos.liveliness_lease_duration = Duration(seconds=0)

        self.query_primitive_client = self.create_client(
            QueryPrimitive, "/rbnx/prm/query", qos_profile=service_qos
        )
        self.get_logger().info("QueryPrimitive service client created")

        self._query_primitives()

        self.start_subscriber = self.create_subscription(
            String, self.start_topic, self.start_callback, 10
        )
        self.get_logger().info(f"Subscribing to start topic: {self.start_topic}")

        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.get_logger().info(f"Publishing to status topic: {self.status_topic}")

        if self.tts_text_topic:
            self.tts_text_publisher = self.create_publisher(
                String, self.tts_text_topic, 10
            )
            self.get_logger().info(
                f"Publishing to TTS text topic: {self.tts_text_topic}"
            )
        else:
            self.tts_text_publisher = None

        if self.tts_status_topic:
            self.tts_status_received = None
            self.tts_status_subscriber = self.create_subscription(
                Bool, self.tts_status_topic, self._tts_status_callback, 10
            )
            self.get_logger().info(
                f"Subscribing to TTS status topic: {self.tts_status_topic}"
            )
        else:
            self.tts_status_subscriber = None

        self.speak_in_progress = False
        self.current_skill_id = None

        self.get_logger().info("Speak skill initialized")

    def _query_primitives(self):
        """Query robonix core for prm::speech.tts. Exit if not found."""
        max_retries = 5
        retry_delay = 2.0

        self.get_logger().info("Querying prm::speech.tts...")
        tts_found = False
        for attempt in range(max_retries):
            try:
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_primitive_client.wait_for_service(
                    timeout_sec=wait_timeout
                ):
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    break

                request = QueryPrimitive.Request()
                request.name = "prm::speech.tts"
                request.filter = "{}"

                future = self.query_primitive_client.call_async(request)
                start_time = time.time()
                timeout_sec = 3.0
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)

                if not future.done():
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    break

                response = future.result()
                if response and response.instances:
                    instance = response.instances[0]
                    input_schema = (
                        json.loads(instance.input_schema)
                        if isinstance(instance.input_schema, str)
                        else instance.input_schema
                    )
                    output_schema = (
                        json.loads(instance.output_schema)
                        if isinstance(instance.output_schema, str)
                        else instance.output_schema
                    )
                    if "text" in input_schema:
                        self.tts_text_topic = input_schema["text"]
                        self.get_logger().info(
                            f"  Found TTS text topic: {self.tts_text_topic}"
                        )
                    if "status" in output_schema:
                        self.tts_status_topic = output_schema["status"]
                        self.get_logger().info(
                            f"  Found TTS status topic: {self.tts_status_topic}"
                        )
                    if self.tts_text_topic and self.tts_status_topic:
                        tts_found = True
                        break
            except Exception as e:
                self.get_logger().error(f"Error querying prm::speech.tts: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

        if not tts_found:
            self.get_logger().error(
                "Failed to query prm::speech.tts after all retries. Exiting."
            )
            sys.exit(1)

    def _tts_status_callback(self, msg):
        """Record latest TTS primitive status (Bool)."""
        if hasattr(msg, "data"):
            self.tts_status_received = msg.data

    def start_callback(self, msg):
        """Handle skill start request."""
        try:
            data = json.loads(msg.data)
            skill_id = data.get("skill_id", "unknown")
            params = data.get("params", {})

            text = params.get("text", "")
            if isinstance(text, str):
                text = text.strip()

            self.get_logger().info(
                f"Received speak request: skill_id={skill_id}, text_len={len(text)}"
            )

            if self.speak_in_progress:
                self.get_logger().warn(
                    "Speak already in progress, ignoring request"
                )
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "Operation already in progress"},
                    errno=1,
                )
                return

            if not text:
                self.get_logger().error("text parameter is required and non-empty")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "text parameter is required and non-empty"},
                    errno=2,
                )
                return

            if not self.tts_text_publisher or not self.tts_status_topic:
                self.get_logger().error("TTS primitive not available")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "TTS primitive not available"},
                    errno=3,
                )
                return

            self.current_skill_id = skill_id
            self.speak_in_progress = True
            self.tts_status_received = None

            self._publish_status(
                skill_id, "running", {"message": "Starting TTS..."}, errno=0
            )

            import threading
            thread = threading.Thread(target=self._run_speak, args=(skill_id, text), daemon=True)
            thread.start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse start message JSON: {e}")
            self._publish_status(
                "unknown", "error", {"error": f"Invalid JSON: {e}"}, errno=4
            )
        except Exception as e:
            self.get_logger().error(f"Error in start_callback: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._publish_status("unknown", "error", {"error": str(e)}, errno=5)

    def _run_speak(self, skill_id, text):
        """Publish text to TTS primitive and wait for status, then publish skill status."""
        try:
            msg = String()
            msg.data = text
            self.tts_text_publisher.publish(msg)
            self.get_logger().info("Published text to TTS primitive")

            timeout = 60.0
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.tts_status_received is not None:
                    success = bool(self.tts_status_received)
                    if success:
                        self._publish_status(
                            skill_id,
                            "finished",
                            {"message": "Speech completed", "text_length": len(text)},
                            errno=0,
                        )
                    else:
                        self._publish_status(
                            skill_id,
                            "error",
                            {"error": "TTS synthesis or playback failed"},
                            errno=6,
                        )
                    self.speak_in_progress = False
                    return
                time.sleep(0.1)

            self._publish_status(
                skill_id,
                "error",
                {"error": "TTS status timeout"},
                errno=7,
            )
            self.speak_in_progress = False

        except Exception as e:
            self.get_logger().error(f"Error in _run_speak: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._publish_status(
                skill_id, "error", {"error": str(e)}, errno=8
            )
            self.speak_in_progress = False

    def _publish_status(self, skill_id, state, result, errno=0):
        """
        Publish skill status to status_topic.

        Format matches executor expectations:
        skill_id, state ("running"|"finished"|"error"), result, errno,
        and optionally error, message, error_message.
        """
        status_msg = {
            "skill_id": skill_id,
            "state": state,
            "result": result,
            "errno": errno,
        }

        if isinstance(result, dict):
            if "error" in result:
                status_msg["error"] = result["error"]
            if "message" in result:
                status_msg["message"] = result["message"]
            if "error_message" in result:
                status_msg["error_message"] = result["error_message"]

        if (state == "error" or errno != 0) and "error" not in status_msg:
            status_msg["error"] = (
                result.get("error", f"Skill execution failed: state={state}, errno={errno}")
                if isinstance(result, dict)
                else f"Skill execution failed: state={state}, errno={errno}"
            )

        out = String()
        out.data = json.dumps(status_msg)
        self.status_publisher.publish(out)
        self.get_logger().info(
            f"Published status: skill_id={skill_id}, state={state}, errno={errno}"
        )


def main(args=None):
    rclpy.init(args=args)
    speak_skill = SpeakSkill()

    shutdown_requested = False

    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        speak_skill.get_logger().info(
            f"Received signal {signum}, shutting down..."
        )
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(speak_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        speak_skill.get_logger().info(
            "Received KeyboardInterrupt, shutting down..."
        )
    finally:
        speak_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            speak_skill.get_logger().info("Speak skill shutdown complete")


if __name__ == "__main__":
    main()
