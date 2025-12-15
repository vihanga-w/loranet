# Returns ping with pong, used by node for signal quality analysis

from commands_handler import CommandRegistry

def register_commands(registry: CommandRegistry):
    def ping_command(gateway, *args):
        return b"pong"

    registry.register(
        "ping",
        ping_command,
        help="Returns \"pong\""
    )