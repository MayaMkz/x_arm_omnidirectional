import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import customtkinter as ctk
from datetime import datetime


class PromptDisplay(Node):
    def __init__(self):
        super().__init__('prompt_display_node')
        self.publisher_ = self.create_publisher(String, '/user_command', 10)

    def send_prompt(self, prompt):
        msg = String()
        msg.data = prompt
        self.publisher_.publish(msg)
        self.get_logger().info(f'Prompt sent: {prompt}')


class PickassoGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.window = ctk.CTk()
        self.window.title("Pickasso")
        self.window.geometry("900x650")
        self.window.configure(fg_color="#050b14")

        self.title = ctk.CTkLabel(
            self.window,
            text="Pickasso",
            font=("Arial", 72, "bold"),
            text_color="#1e90ff"
        )
        self.title.pack(pady=(40, 5))

        self.subtitle = ctk.CTkLabel(
            self.window,
            text="ROBOT COMMAND INTERFACE",
            font=("Arial", 20),
            text_color="#9bbce8"
        )
        self.subtitle.pack(pady=(0, 10))

        self.status = ctk.CTkLabel(
            self.window,
            text="● Connected to ROS 2",
            font=("Arial", 18),
            text_color="#34d399"
        )
        self.status.pack(pady=(0, 25))

        self.input_frame = ctk.CTkFrame(
            self.window,
            fg_color="#07111f",
            border_color="#1e90ff",
            border_width=2,
            corner_radius=18
        )
        self.input_frame.pack(padx=40, pady=10, fill="x")

        self.entry = ctk.CTkEntry(
            self.input_frame,
            placeholder_text="Escribe tu prompt aquí...",
            font=("Arial", 20),
            height=55,
            fg_color="#020817",
            border_color="#334155",
            text_color="white"
        )
        self.entry.pack(side="left", padx=20, pady=25, fill="x", expand=True)
        self.entry.bind("<Return>", self.send_prompt)

        self.send_button = ctk.CTkButton(
            self.input_frame,
            text="➤",
            font=("Arial", 28, "bold"),
            width=70,
            height=55,
            corner_radius=14,
            command=self.send_prompt
        )
        self.send_button.pack(side="right", padx=(0, 20), pady=25)

        self.history_frame = ctk.CTkFrame(
            self.window,
            fg_color="#07111f",
            border_color="#0f2a44",
            border_width=1,
            corner_radius=18
        )
        self.history_frame.pack(padx=40, pady=25, fill="both", expand=True)

        self.history_title = ctk.CTkLabel(
            self.history_frame,
            text="Historial de Prompts",
            font=("Arial", 22, "bold"),
            text_color="#38bdf8"
        )
        self.history_title.pack(anchor="w", padx=25, pady=(20, 10))

        self.history_box = ctk.CTkTextbox(
            self.history_frame,
            font=("Arial", 16),
            fg_color="#020817",
            text_color="#e5e7eb",
            corner_radius=12
        )
        self.history_box.pack(padx=25, pady=(0, 20), fill="both", expand=True)
        self.history_box.configure(state="disabled")

        self.bottom_bar = ctk.CTkLabel(
            self.window,
            text="Publicando en: /user_command        ● ROS 2 Humble",
            font=("Arial", 15),
            text_color="#60a5fa"
        )
        self.bottom_bar.pack(pady=(0, 15))

    def send_prompt(self, event=None):
        prompt = self.entry.get().strip()

        if prompt == "":
            return

        self.ros_node.send_prompt(prompt)

        current_time = datetime.now().strftime("%H:%M:%S")

        self.history_box.configure(state="normal")
        self.history_box.insert("end", f"› {prompt}     [{current_time}]\n")
        self.history_box.configure(state="disabled")
        self.history_box.see("end")

        self.entry.delete(0, "end")

    def run(self):
        self.window.mainloop()


def main():
    rclpy.init()
    ros_node = PromptDisplay()
    gui = PickassoGUI(ros_node)
    gui.run()
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
