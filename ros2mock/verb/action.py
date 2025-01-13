import importlib

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros2cli.node import NODE_NAME_PREFIX
from ros2action.api import action_name_completer, action_type_completer
from ros2mock.api import ActionResultCompleter, ActionFeedbackCompleter, edit_string_in_editor
from ros2mock.verb import VerbExtension
from rosidl_runtime_py import set_message_fields, message_to_yaml
from rclpy.action import ActionServer, GoalResponse
import yaml


class ActionVerb(VerbExtension):
    """Mock an action."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            "action_name", help="Name of the ROS action to mock to (e.g. '/add_two_ints')"
        )
        arg.completer = action_name_completer
        arg = parser.add_argument(
            "action_type",
            help="Type of the ROS action (e.g. 'example_interfaces/action/Fibonacci')",
        )
        arg.completer = action_type_completer
        arg = parser.add_argument(
            "values",
            nargs="?",
            default="{}",
            help="Values to fill the action result with in YAML format "
            + "(e.g. '{a: 1, b: 2}'), "
            + "otherwise the action result will be published with default values",
        )
        arg.completer = ActionResultCompleter(action_type_key="action_type")
        arg = parser.add_argument(
            "--feedback",
            nargs="?",
            action="append",
            help="Values to be used as feedback, can be used multiple times. Get returned in sequence",
        )
        arg.completer = ActionFeedbackCompleter(action_type_key="action_type")
        
        arg = parser.add_argument(
            "--reject", action="store_true", help="Reject the goal"
        )
        arg = parser.add_argument("--abort", action="store_true", help="Abort the goal.")

        arg = parser.add_argument(
            "--editor", action="store_true", help="If set, opens the editor on result"
        )

        arg = parser.add_argument(
            "--timeout",
            type=float,
            default=None,
            help="Timeout before result or next feedback is sent",
        )

    def main(self, *, args):
        return mocker(
            args.action_type,
            args.action_name,
            args.values,
            args.feedback,
            args.editor,
            args.reject,
            args.abort,
            args.timeout,
        )


def mocker(
    action_type: str,
    action_name: str,
    values: str,
    feedbacks: list[str],
    editor: bool,
    reject: bool,
    abort: bool,
    timeout: float | None,
):
    feedbacks = feedbacks or []
    try:
        parts = action_type.split("/")
        if len(parts) == 2:
            parts = [parts[0], "srv", parts[1]]
        package_name = parts[0]
        module = importlib.import_module(".".join(parts[:-1]))
        action_type_name = parts[-1]
        action_module = getattr(module, action_type_name)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError("The passed action type is invalid")
    try:
        action_module.Goal
        action_module.Result
        action_module.Feedback
    except AttributeError:
        raise RuntimeError("The passed type is not a action")

    values_dictionary = yaml.safe_load(values)

    rclpy.init()
    node = rclpy.create_node(
        NODE_NAME_PREFIX + "_action_mocker_%s_%s" % (package_name, action_type_name)
    )

    if timeout:
        wait_rate = node.create_rate(1.0 / timeout)

    result = action_module.Result()
    try:
        set_message_fields(result, values_dictionary)
    except Exception as e:
        return "Failed to populate field: {0}".format(e)

    def handle_goal_callback(message):
        node.get_logger().info("Received goal request")
        if reject:
            node.get_logger().info("Goal rejected")
            return GoalResponse.REJECT
        node.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def generic_callback(goal_handle):
        if editor:
            response_string = edit_string_in_editor(message_to_yaml(result))
            try:
                set_message_fields(result, yaml.safe_load(response_string))
            except Exception as e:
                return "Failed to populate field: {0}".format(e)

        feedback = action_module.Feedback()

        for f in feedbacks:
            if timeout:
                wait_rate.sleep()
            f = f or "{}"  # in case it's None
            set_message_fields(feedback, yaml.safe_load(f))
            goal_handle.publish_feedback(feedback)
            node.get_logger().info("Published Feedback")

        if abort:
            goal_handle.abort()
            return result

        if timeout:
            wait_rate.sleep()
        goal_handle.succeed()
        node.get_logger().info("Returned Result")
        return result

    server = ActionServer(node, action_module, action_name, generic_callback,
      goal_callback=handle_goal_callback)

    exec = MultiThreadedExecutor()
    rclpy.spin(node, executor=exec)

    node.destroy_node()
    rclpy.shutdown()
