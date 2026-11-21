"""Explicit practice-only heartbeat. Does not implement the SSL game-controller protocol."""

import rclpy
from rclpy.node import Node
from roboturk_interfaces.msg import RefereeState


def main():
    rclpy.init()
    node = Node("practice_referee")
    publisher = node.create_publisher(RefereeState, "/roboturk/referee", 1)

    def publish():
        msg = RefereeState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.command, msg.attack_sign = "running", 1
        publisher.publish(msg)

    node.create_timer(0.1, publish)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
