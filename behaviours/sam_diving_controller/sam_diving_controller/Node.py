#!/usr/bin/python3

import rclpy
import sys

from .SAMDiveView import SAMDiveView
from .ActionServerDiveController import DiveActionServerController
from .DiveController import DiveController
from .DivingModel import DiveControlModel
from .ConvenienceView import ConvenienceView

from rclpy.executors import MultiThreadedExecutor

def main():
    """
    Run manual setpoints
    """

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("DivingNode")

    node.declare_parameter('view_rate', 0.1)
    node.declare_parameter('model_rate', 0.1)
    node.declare_parameter('controller_rate', 0.1)
    node.declare_parameter('convenience_rate', 0.1)

    # This is not a frequency, but a period.
    # t = 10 -> callback gets called every 10 sec
    view_rate = node.get_parameter('view_rate').get_parameter_value().double_value
    model_rate = node.get_parameter('model_rate').get_parameter_value().double_value
    controller_rate = node.get_parameter('controller_rate').get_parameter_value().double_value

    convenience_view_rate = node.get_parameter('convenience_rate').get_parameter_value().double_value

    view = SAMDiveView(node)
    controller = DiveController(node, view)   # Note, this is a MVC controller, not a control theory controller
    model = DiveControlModel(node, view, controller, model_rate)  # This is where the actual PID controller lives.

    convenience_view = ConvenienceView(node, controller, model)


    node.create_timer(view_rate, view.update)
    node.create_timer(model_rate, model.update)
    node.create_timer(controller_rate, controller.update)

    node.create_timer(convenience_view_rate, convenience_view.update)

    def _loginfo(node, s):
        node.get_logger().info(s)

    _loginfo(node,"Setpoints in Topic")
    _loginfo(node,"Created MVC")

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
        #rclpy.spin(node)
        _loginfo(node, "Spinning up")
    except KeyboardInterrupt:
        pass

    _loginfo(node,"Shutting down")


def action_server():

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("DivingNode")

    node.declare_parameter('view_rate', 0.1)
    node.declare_parameter('model_rate', 0.1)
    node.declare_parameter('controller_rate', 0.1)
    node.declare_parameter('convenience_rate', 0.1)

    # This is not a frequency, but a period.
    # t = 10 -> callback gets called every 10 sec
    view_rate = node.get_parameter('view_rate').get_parameter_value().double_value
    model_rate = node.get_parameter('model_rate').get_parameter_value().double_value
    controller_rate = node.get_parameter('controller_rate').get_parameter_value().double_value

    convenience_view_rate = node.get_parameter('convenience_rate').get_parameter_value().double_value

    view = SAMDiveView(node)
    controller = DiveActionServerController(node, view)   # Note, this is a MVC controller, not a control theory controller
    model = DiveControlModel(node, view, controller, model_rate)  # This is where the actual PID controller lives.

    convenience_view = ConvenienceView(node, controller, model)


    node.create_timer(view_rate, view.update)
    node.create_timer(model_rate, model.update)
    node.create_timer(controller_rate, controller.update)

    node.create_timer(convenience_view_rate, convenience_view.update)

    def _loginfo(node, s):
        node.get_logger().info(s)

    _loginfo(node,"Action Server")
    _loginfo(node,"Created MVC")

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
        #rclpy.spin(node)
        _loginfo(node, "Spinning up")
    except KeyboardInterrupt:
        pass

    _loginfo(node,"Shutting down")


if __name__ == "__main__":
    main()
