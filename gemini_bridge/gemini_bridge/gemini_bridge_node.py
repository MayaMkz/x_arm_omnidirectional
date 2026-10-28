#!/usr/bin/env python3
import json
from google import genai

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MODEL = "gemini-2.5-flash"

SYSTEM_PROMPT = """
You are a robot movement interpreter.

Your job is to convert the user's message into one robot action.

Valid commands are:

MOVE UP
MOVE DOWN
MOVE LEFT
MOVE RIGHT
HOME
STOP
INVALID

Understand commands in English or Spanish.

Examples:

"move up" -> {"command": "MOVE", "direction": "UP"}
"go higher" -> {"command": "MOVE", "direction": "UP"}
"arriba" -> {"command": "MOVE", "direction": "UP"}
"sube" -> {"command": "MOVE", "direction": "UP"}

"move down" -> {"command": "MOVE", "direction": "DOWN"}
"abajo" -> {"command": "MOVE", "direction": "DOWN"}
"baja" -> {"command": "MOVE", "direction": "DOWN"}

"move left" -> {"command": "MOVE", "direction": "LEFT"}
"izquierda" -> {"command": "MOVE", "direction": "LEFT"}
"ve a la izquierda" -> {"command": "MOVE", "direction": "LEFT"}

"move right" -> {"command": "MOVE", "direction": "RIGHT"}
"derecha" -> {"command": "MOVE", "direction": "RIGHT"}
"ve a la derecha" -> {"command": "MOVE", "direction": "RIGHT"}

"go home" -> {"command": "HOME", "direction": null}
"home" -> {"command": "HOME", "direction": null}
"regresa" -> {"command": "HOME", "direction": null}
"inicio" -> {"command": "HOME", "direction": null}

"stop" -> {"command": "STOP", "direction": null}
"detente" -> {"command": "STOP", "direction": null}
"para" -> {"command": "STOP", "direction": null}

If the user intention is unclear, choose the closest valid action.
If the message does not refer to robot motion, return INVALID.

Return ONLY JSON.

For movement:
{
  "command": "MOVE",
  "direction": "UP"
}

For home:
{
  "command": "HOME",
  "direction": null
}

For stop:
{
  "command": "STOP",
  "direction": null
}

For invalid:
{
  "command": "INVALID",
  "direction": null,
  "reason": "could not understand"
}
"""


class GeminiBridgeNode(Node):
    def __init__(self):
        super().__init__('gemini_bridge_node')

        self.client = genai.Client()

        self.sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/robot_action',
            10
        )

        self.get_logger().info('Gemini bridge node started.')

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
        allowed_commands = {"MOVE", "STOP", "HOME", "INVALID"}
        allowed_directions = {"UP", "DOWN", "LEFT", "RIGHT", None}

        command = data.get("command")
        direction = data.get("direction")

        if command not in allowed_commands:
            return False, "Unsupported command"

        if direction not in allowed_directions:
            return False, "Unsupported direction"

        if command == "MOVE" and direction is None:
            return False, "MOVE requires a direction"

        if command in ["STOP", "HOME", "INVALID"] and direction is not None:
            return False, f"{command} must have null direction"

        return True, None

    def invalid_response(self, reason):
        return {
            "command": "INVALID",
            "direction": None,
            "distance_m": None,
            "station_id": None,
            "duration_s": None,
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

    def publish_command(self, data):
        out = String()
        out.data = json.dumps(data)
        self.pub.publish(out)
        self.get_logger().info(f"Published JSON: {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GeminiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
