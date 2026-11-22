"""Publish the effective SSL referee decision with the simulation clock stamp."""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from roboturk.ssl_referee import GROUP, PORT, RefereeSocket
from roboturk_interfaces.msg import RefereeState


class OfficialReferee(Node):
    def __init__(self, **node_options):
        super().__init__("ssl_referee", **node_options)
        address = self.declare_parameter("address", GROUP).value
        port = self.declare_parameter("port", PORT).value
        interface = self.declare_parameter("interface", "0.0.0.0").value
        self.receiver = RefereeSocket(address, port, interface)
        self.publisher = self.create_publisher(RefereeState, "/roboturk/referee", 1)
        self.create_timer(0.04, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self):
        state = self.receiver.poll()
        msg = RefereeState(
            command=state.mode.value,
            attack_sign=state.attack_sign,
            ssl_command=state.command,
            stage=state.stage,
            command_counter=state.counter,
            source=state.source,
            restart_team=state.restart_team,
            placement_team=state.placement_team,
            has_placement_target=state.placement_target is not None,
            placement_x=(state.placement_target or (0.0, 0.0))[0],
            placement_y=(state.placement_target or (0.0, 0.0))[1],
            blue_score=state.score[0],
            yellow_score=state.score[1],
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = OfficialReferee()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
