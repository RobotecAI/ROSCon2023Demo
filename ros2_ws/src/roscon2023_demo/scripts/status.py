#!/usr/bin/env python3

import sys
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from functools import partial

class ROS2StringTopicViewer(Node):
    def __init__(self):
        super().__init__('ros2_string_topic_viewer')
        self.subscribers = {}
        self.topic_values = {}

        self.qos_profile = QoSProfile(depth=10)

        # Define the topics you want to subscribe to and display
        self.topic_names = ['/otto_1/deliberation_description',
                            '/otto_2/deliberation_description',
                            '/otto_3/deliberation_description',
                            '/otto_4/deliberation_description',
                            '/otto_5/deliberation_description',
                            '/otto_6/deliberation_description',
                            '/otto_7/deliberation_description',
                            '/otto_8/deliberation_description',
                            '/ur1/status_description',
                            '/ur2/status_description',
                            '/ur3/status_description',
                            '/ur4/status_description',
                            ]  #

        for topic_name in self.topic_names:
            self.subscribers[topic_name] = self.create_subscription(
                String,
                topic_name,
                partial(self.topic_callback, incoming_topic_name=topic_name),
                self.qos_profile,
         
            )
            self.topic_values[topic_name] = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)  # Update the UI every second

    def topic_callback(self, msg, incoming_topic_name):
        self.topic_values[incoming_topic_name] = msg.data

    def update_ui(self):

        rclpy.spin_once(self)
        # Update the UI with the latest topic values
        for topic_name, value in self.topic_values.items():
            self.ui_elements[topic_name].setText(f"{topic_name}: {value}")

def main(args=None):
    global node
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    centralWidget = QWidget(mainWindow)
    layout = QVBoxLayout(centralWidget)
    mainWindow.setCentralWidget(centralWidget)

    node = ROS2StringTopicViewer()

    node.ui_elements = {}

    for topic_name in node.topic_names:
        label = QLabel(centralWidget)
        layout.addWidget(label)
        node.ui_elements[topic_name] = label

    mainWindow.show()
    sys.exit(app.exec_())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
