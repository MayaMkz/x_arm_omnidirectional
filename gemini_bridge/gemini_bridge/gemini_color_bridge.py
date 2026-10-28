#!/usr/bin/env python3

import json
import time
import uuid
from google import genai

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


MODEL = "gemini-2.5-flash"

SYSTEM_PROMPT = """
You are a robot pick-and-place color interpreter.

Your job is to convert the user's message into one robot action.

The robot can only pick cubes of these colors:

PINK
BLUE
GREEN
RED

Understand commands in English or Spanish.

Valid commands are:

PICK_COLOR
STOP
HOME
INVALID

Examples:

"pick the red cube" -> {"command": "PICK_COLOR", "color": "RED"}
"grab the red one" -> {"command": "PICK_COLOR", "color": "RED"}
"take all red cubes" -> {"command": "PICK_COLOR", "color": "RED"}
"agarra el cubo rojo" -> {"command": "PICK_COLOR", "color": "RED"}
"toma todos los rojos" -> {"command": "PICK_COLOR", "color": "RED"}
"quiero el dado rojo" -> {"command": "PICK_COLOR", "color": "RED"}

"pick the blue cube" -> {"command": "PICK_COLOR", "color": "BLUE"}
"grab blue" -> {"command": "PICK_COLOR", "color": "BLUE"}
"agarra el azul" -> {"command": "PICK_COLOR", "color": "BLUE"}
"toma los cubos azules" -> {"command": "PICK_COLOR", "color": "BLUE"}

"pick the green cube" -> {"command": "PICK_COLOR", "color": "GREEN"}
"grab green" -> {"command": "PICK_COLOR", "color": "GREEN"}
"agarra el verde" -> {"command": "PICK_COLOR", "color": "GREEN"}
"toma los dados verdes" -> {"command": "PICK_COLOR", "color": "GREEN"}

"pick the pink cube" -> {"command": "PICK_COLOR", "color": "PINK"}
"grab pink" -> {"command": "PICK_COLOR", "color": "PINK"}
"agarra el rosa" -> {"command": "PICK_COLOR", "color": "PINK"}
"agarra el rosado" -> {"command": "PICK_COLOR", "color": "PINK"}
"toma los dados rosas" -> {"command": "PICK_COLOR", "color": "PINK"}

"go home" -> {"command": "HOME", "color": null}
"home" -> {"command": "HOME", "color": null}
"regresa" -> {"command": "HOME", "color": null}
"inicio" -> {"command": "HOME", "color": null}

"stop" -> {"command": "STOP", "color": null}
"detente" -> {"command": "STOP", "color": null}
"para" -> {"command": "STOP", "color": null}

If the user mentions more than one color, return INVALID.
If the user does not mention pink, blue, green, or red, return INVALID.
If the message does not refer to robot picking or color selection, return INVALID.

Return ONLY JSON.

For color picking:
{
  "command": "PICK_COLOR",
  "color": "RED"
}

For home:
{
  "command": "HOME",
  "color": null
}

For stop:
{
  "command": "STOP",
  "color": null
}

For invalid:
{
  "command": "INVALID",
  "color": null,
  "reason": "could not detect color"
}
"""


class GeminiColorBridgeNode(Node):
    def __init__(self):
        super().__init__("gemini_color_bridge_node")

        self.client = genai.Client()

        self.sub = self.create_subscription(
            String,
            "/user_command",
            self.command_callback,
            10
        )

        self.robot_action_pub = self.create_publisher(
            String,
            "/robot_action",
            10
        )

        target_color_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.target_color_pub = self.create_publisher(
            String,
            "/target_color",
            target_color_qos
        )

        self.get_logger().info("Gemini color bridge node started.")

    def extract_json(self, text):
        text = text.strip()

        if text.startswith("```"):
            text = text.replace("```json", "")
            text = text.replace("```", "")
            text = text.strip()

        start = text.find("{")
        end = text.rfind("}")

        if start == -1 or end == -1 or end <= start:
            raise ValueError("No JSON object found in Gemini response")

        return text[start:end + 1]

    def validate_command(self, data):
        allowed_commands = {"PICK_COLOR", "STOP", "HOME", "INVALID"}
        allowed_colors = {"PINK", "BLUE", "GREEN", "RED", None}

        command = data.get("command")
        color = data.get("color")

        if command not in allowed_commands:
            return False, "Unsupported command"

        if color not in allowed_colors:
            return False, "Unsupported color"

        if command == "PICK_COLOR" and color is None:
            return False, "PICK_COLOR requires a color"

        if command in {"STOP", "HOME", "INVALID"} and color is not None:
            return False, f"{command} must have null color"

        return True, None

    def invalid_response(self, reason):
        return {
            "command": "INVALID",
            "color": None,
            "reason": reason
        }

    def command_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f"Received user_text: [{user_text}]")

        if not user_text:
            data = self.invalid_response("Empty input")
            self.publish_command(data)
            return

        prompt = f"{SYSTEM_PROMPT}\n\nUser instruction: {user_text}"

        try:
            response = self.client.models.generate_content(
                model=MODEL,
                contents=prompt
            )

            raw = response.text.strip()
            self.get_logger().info(f"Raw Gemini response: {raw}")

            json_text = self.extract_json(raw)
            data = json.loads(json_text)

            valid, reason = self.validate_command(data)

            if not valid:
                self.get_logger().warn(f"Invalid Gemini JSON: {reason}")
                data = self.invalid_response(reason)

            self.publish_command(data)

        except Exception as e:
            self.get_logger().error(f"Gemini request failed: {e}")
            data = self.invalid_response(str(e))
            self.publish_command(data)

    def publish_target_color_once(self, color):
        order_id = f"{int(time.time())}_{color}_{uuid.uuid4().hex[:6]}"

        payload = {
            "color": color,
            "order_id": order_id
        }

        color_msg = String()
        color_msg.data = json.dumps(payload)

        self.target_color_pub.publish(color_msg)

        self.get_logger().info(
            f"Published target color once: {color_msg.data}"
        )

    def publish_command(self, data):
        action_msg = String()
        action_msg.data = json.dumps(data)
        self.robot_action_pub.publish(action_msg)

        if data.get("command") == "PICK_COLOR":
            self.publish_target_color_once(data["color"])

        self.get_logger().info(f"Published robot action: {action_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GeminiColorBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
