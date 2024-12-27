import importlib
import time

import rclpy
from ros2cli.node import NODE_NAME_PREFIX
from ros2service.api import ServiceNameCompleter, ServiceTypeCompleter
from ros2mock.api import ServiceResponseCompleter, edit_string_in_editor
from ros2mock.verb import VerbExtension
from rosidl_runtime_py import set_message_fields, message_to_yaml
import yaml

class ServiceVerb(VerbExtension):
    """Mock a service."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to mock to (e.g. '/add_two_ints')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        arg = parser.add_argument(
            'service_type',
            help="Type of the ROS service (e.g. 'std_srvs/srv/Empty')")
        arg.completer = ServiceTypeCompleter(
            service_name_key='service_name')
        arg = parser.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the service response with in YAML format ' +
                 "(e.g. '{a: 1, b: 2}'), " +
                 'otherwise the service request will be published with default values')
        arg.completer = ServiceResponseCompleter(
            service_type_key='service_type')
        arg = parser.add_argument('--editor',
            action="store_true", help="If set, opens the editor on response")

    def main(self, *, args):
        return mocker(
            args.service_type, args.service_name, args.values, args.editor)


def mocker(service_type, service_name, values, editor):
    try:
        parts = service_type.split('/')
        if len(parts) == 2:
            parts = [parts[0], 'srv', parts[1]]
        package_name = parts[0]
        module = importlib.import_module('.'.join(parts[:-1]))
        srv_name = parts[-1]
        srv_module = getattr(module, srv_name)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError('The passed service type is invalid')
    try:
        srv_module.Request
        srv_module.Response
    except AttributeError:
        raise RuntimeError('The passed type is not a service')

    values_dictionary = yaml.safe_load(values)

    rclpy.init()
    node = rclpy.create_node(NODE_NAME_PREFIX + '_requester_%s_%s' % (package_name, srv_name))

    response = srv_module.Response()
    try:
        set_message_fields(response, values_dictionary)
    except Exception as e:
        return 'Failed to populate field: {0}'.format(e)

    def generic_callback(req, _):
        node.get_logger().debug(f"Request: {req}")
        if editor:
            response_string = edit_string_in_editor(message_to_yaml(response))
            set_message_fields(response, yaml.safe_load(response_string))
        return response

    node.create_service(srv_module, service_name, generic_callback)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

