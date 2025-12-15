# Returns the current server time

import time
import math
from commands_handler import CommandRegistry

def register_commands(registry: CommandRegistry):
    def time_command(gateway, *args):
        return str(math.floor(time.time() * 1000)).encode('utf-8')

    registry.register(
        "time",
        time_command,
        help="Returns the current server time"
    )