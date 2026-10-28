#!/usr/bin/env python3

import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QLineEdit,
    QVBoxLayout, QHBoxLayout, QFrame, QScrollArea
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPixmap


IMAGE_PATH = "/home/jerry/x_arm/src/gemini_bridge/gemini_bridge/assets/robot_display.jpeg"


class PickassoRosNode(Node):
    def __init__(self):
        super().__init__("pickasso_gui_qt_node")
        self.publisher_ = self.create_publisher(String, "/user_command", 10)
        self.get_logger().info("Pickasso Qt GUI node started.")

    def send_prompt(self, prompt):
        msg = String()
        msg.data = prompt
        self.publisher_.publish(msg)
        self.get_logger().info(f"Prompt sent: {prompt}")


class PickassoQtGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.dot_state = True

        self.setWindowTitle("Pickasso")
        self.setMinimumSize(1150, 760)

        self.setStyleSheet("""
            QWidget {
                background-color: #05070d;
                color: #f8fafc;
                font-family: Arial;
            }

            QFrame#Sidebar {
                background-color: #0b1220;
                border-radius: 28px;
            }

            QLabel#Logo {
                font-size: 44px;
                font-weight: 900;
                color: #f8fafc;
            }

            QLabel#LogoAccent {
                font-size: 15px;
                color: #60a5fa;
                letter-spacing: 2px;
            }

            QLabel#Status {
                font-size: 16px;
                color: #22c55e;
            }

            QLabel#SideText {
                font-size: 15px;
                color: #94a3b8;
            }

            QFrame#Hero {
                background-color: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #111827,
                    stop:0.45 #0f172a,
                    stop:1 #172554
                );
                border-radius: 34px;
                border: 1px solid #1d4ed8;
            }

            QLabel#HeroTitle {
                font-size: 54px;
                font-weight: 900;
                color: #ffffff;
            }

            QLabel#HeroSubtitle {
                font-size: 20px;
                color: #bfdbfe;
            }

            QLineEdit {
                background-color: rgba(2, 6, 23, 210);
                border: 1px solid #334155;
                border-radius: 22px;
                padding: 18px;
                font-size: 20px;
                color: #ffffff;
            }

            QLineEdit:focus {
                border: 2px solid #60a5fa;
            }

            QPushButton {
                background-color: #2563eb;
                border: none;
                border-radius: 20px;
                padding: 16px;
                color: white;
                font-size: 17px;
                font-weight: bold;
            }

            QPushButton:hover {
                background-color: #3b82f6;
            }

            QPushButton:pressed {
                background-color: #1d4ed8;
            }

            QPushButton#SendButton {
                background-color: #ffffff;
                color: #0f172a;
                font-size: 28px;
                border-radius: 22px;
                min-width: 85px;
            }

            QPushButton#PinkButton {
                background-color: #db2777;
            }

            QPushButton#BlueButton {
                background-color: #2563eb;
            }

            QPushButton#GreenButton {
                background-color: #16a34a;
            }

            QPushButton#RedButton {
                background-color: #dc2626;
            }

            QFrame#GlassCard {
                background-color: rgba(15, 23, 42, 210);
                border-radius: 28px;
                border: 1px solid #1e293b;
            }

            QLabel#SectionTitle {
                font-size: 24px;
                font-weight: 800;
                color: #e0f2fe;
            }

            QLabel#BigStatus {
                font-size: 28px;
                font-weight: 900;
                color: #ffffff;
            }

            QLabel#Muted {
                font-size: 15px;
                color: #94a3b8;
            }

            QLabel#Bubble {
                background-color: #111827;
                border: 1px solid #263244;
                border-radius: 18px;
                padding: 12px;
                color: #e5e7eb;
                font-size: 15px;
            }

            QLabel#RobotImage {
                background-color: transparent;
                border-radius: 18px;
            }

            QScrollArea {
                border: none;
                background: transparent;
            }
        """)

        self.build_ui()
        self.start_status_animation()

    def build_ui(self):
        main = QHBoxLayout()
        main.setContentsMargins(28, 28, 28, 28)
        main.setSpacing(24)

        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        sidebar.setFixedWidth(280)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(28, 32, 28, 32)
        sidebar_layout.setSpacing(20)

        logo = QLabel("Pickasso")
        logo.setObjectName("Logo")

        accent = QLabel("ROBOT AI SYSTEM")
        accent.setObjectName("LogoAccent")

        self.status = QLabel("● ROS 2 Connected")
        self.status.setObjectName("Status")

        side_text = QLabel(
            "Natural language interface\n"
            "for color-based pick & place.\n\n"
            "Write a command or select\n"
            "one of the quick actions."
        )
        side_text.setObjectName("SideText")
        side_text.setWordWrap(True)

        image_label = QLabel()
        image_label.setObjectName("RobotImage")
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setFixedSize(224, 224)

        pixmap = QPixmap(IMAGE_PATH)

        if not pixmap.isNull():
            image_label.setPixmap(
                pixmap.scaled(
                    224,
                    224,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
            )
        else:
            image_label.setText("Image not found")
            image_label.setStyleSheet("color: #ef4444; font-size: 13px;")

        sidebar_layout.addWidget(logo)
        sidebar_layout.addWidget(accent)
        sidebar_layout.addSpacing(10)
        sidebar_layout.addWidget(self.status)
        sidebar_layout.addSpacing(20)
        sidebar_layout.addWidget(side_text)
        sidebar_layout.addSpacing(18)
        sidebar_layout.addWidget(image_label)
        sidebar_layout.addStretch()

        footer = QLabel("Publishing:\n/user_command")
        footer.setObjectName("SideText")
        sidebar_layout.addWidget(footer)

        sidebar.setLayout(sidebar_layout)
        main.addWidget(sidebar)

        content = QVBoxLayout()
        content.setSpacing(24)

        hero = QFrame()
        hero.setObjectName("Hero")

        hero_layout = QVBoxLayout()
        hero_layout.setContentsMargins(38, 34, 38, 34)
        hero_layout.setSpacing(18)

        hero_title = QLabel("What should Pickasso do?")
        hero_title.setObjectName("HeroTitle")

        hero_subtitle = QLabel(
            "Type naturally in English or Spanish. Gemini will interpret the color command."
        )
        hero_subtitle.setObjectName("HeroSubtitle")

        input_row = QHBoxLayout()
        input_row.setSpacing(16)

        self.entry = QLineEdit()
        self.entry.setPlaceholderText(
            "Example: Pick all red cubes / Agarra todos los cubos rojos"
        )
        self.entry.returnPressed.connect(self.send_prompt)

        self.send_button = QPushButton("➜")
        self.send_button.setObjectName("SendButton")
        self.send_button.clicked.connect(self.send_prompt)

        input_row.addWidget(self.entry)
        input_row.addWidget(self.send_button)

        hero_layout.addWidget(hero_title)
        hero_layout.addWidget(hero_subtitle)
        hero_layout.addSpacing(12)
        hero_layout.addLayout(input_row)

        hero.setLayout(hero_layout)
        content.addWidget(hero)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(22)

        quick_card = QFrame()
        quick_card.setObjectName("GlassCard")
        quick_layout = QVBoxLayout()
        quick_layout.setContentsMargins(28, 26, 28, 28)
        quick_layout.setSpacing(16)

        quick_title = QLabel("Quick Actions")
        quick_title.setObjectName("SectionTitle")

        quick_hint = QLabel("Send a clean color command instantly.")
        quick_hint.setObjectName("Muted")

        quick_layout.addWidget(quick_title)
        quick_layout.addWidget(quick_hint)

        grid1 = QHBoxLayout()
        grid1.setSpacing(12)

        self.pink_button = self.color_button("PINK", "Pick all pink cubes", "PinkButton")
        self.blue_button = self.color_button("BLUE", "Pick all blue cubes", "BlueButton")
        self.green_button = self.color_button("GREEN", "Pick all green cubes", "GreenButton")
        self.red_button = self.color_button("RED", "Pick all red cubes", "RedButton")

        grid1.addWidget(self.pink_button)
        grid1.addWidget(self.blue_button)
        grid1.addWidget(self.green_button)
        grid1.addWidget(self.red_button)

        quick_layout.addLayout(grid1)
        quick_layout.addStretch()
        quick_card.setLayout(quick_layout)

        state_card = QFrame()
        state_card.setObjectName("GlassCard")
        state_layout = QVBoxLayout()
        state_layout.setContentsMargins(28, 26, 28, 28)
        state_layout.setSpacing(16)

        state_title = QLabel("System State")
        state_title.setObjectName("SectionTitle")

        self.big_status = QLabel("Ready")
        self.big_status.setObjectName("BigStatus")

        self.last_command = QLabel("No command sent yet.")
        self.last_command.setObjectName("Muted")
        self.last_command.setWordWrap(True)

        state_layout.addWidget(state_title)
        state_layout.addWidget(self.big_status)
        state_layout.addWidget(self.last_command)
        state_layout.addStretch()
        state_card.setLayout(state_layout)

        cards_row.addWidget(quick_card, stretch=3)
        cards_row.addWidget(state_card, stretch=2)

        content.addLayout(cards_row)

        history_card = QFrame()
        history_card.setObjectName("GlassCard")
        history_layout = QVBoxLayout()
        history_layout.setContentsMargins(28, 26, 28, 28)
        history_layout.setSpacing(14)

        history_title = QLabel("Recent Commands")
        history_title.setObjectName("SectionTitle")

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)

        self.history_container = QWidget()
        self.history_layout = QVBoxLayout()
        self.history_layout.setContentsMargins(0, 0, 0, 0)
        self.history_layout.setSpacing(10)
        self.history_layout.addStretch()
        self.history_container.setLayout(self.history_layout)

        self.scroll.setWidget(self.history_container)

        history_layout.addWidget(history_title)
        history_layout.addWidget(self.scroll)

        history_card.setLayout(history_layout)

        content.addWidget(history_card, stretch=1)

        main.addLayout(content)
        self.setLayout(main)

    def color_button(self, label, command_text, object_name):
        button = QPushButton(label)
        button.setObjectName(object_name)
        button.setMinimumHeight(64)
        button.clicked.connect(lambda: self.quick_command(command_text))
        return button

    def quick_command(self, command_text):
        self.entry.setText(command_text)
        self.send_prompt()

    def send_prompt(self):
        prompt = self.entry.text().strip()

        if not prompt:
            return

        self.ros_node.send_prompt(prompt)

        now = datetime.now().strftime("%H:%M:%S")

        self.big_status.setText("Command Sent")
        self.last_command.setText(prompt)

        bubble = QLabel(f"› {prompt}    [{now}]")
        bubble.setObjectName("Bubble")
        bubble.setWordWrap(True)

        self.history_layout.insertWidget(0, bubble)
        self.entry.clear()

        QTimer.singleShot(1200, lambda: self.big_status.setText("Ready"))

    def start_status_animation(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.toggle_status_dot)
        self.timer.start(850)

    def toggle_status_dot(self):
        self.dot_state = not self.dot_state
        if self.dot_state:
            self.status.setText("● ROS 2 Connected")
        else:
            self.status.setText("○ ROS 2 Connected")


def main(args=None):
    rclpy.init(args=args)

    ros_node = PickassoRosNode()

    app = QApplication(sys.argv)
    gui = PickassoQtGUI(ros_node)
    gui.show()

    try:
        app.exec()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
