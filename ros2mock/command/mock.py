from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class MockCommand(CommandExtension):
    """Various mocking related sub-commands."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        parser.add_argument(
            '--include-hidden-services', action='store_true',
            help='Consider hidden services as well')

        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2mock.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')

        # call the verb's main method
        return extension.main(args=args)
