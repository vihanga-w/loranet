from typing import Callable, Dict

class CommandRegistry:
    def __init__(self):
        self._commands: Dict[str, Dict[str, Callable[[object], bytes] | str]] = {}

    def register(self, name: str, handler: Callable[[object], bytes], help: str = ""):
        self._commands[name.lower()] = {
            "handler": handler,
            "help": help
        }
    
    def list_commands(self):
        return self._commands.keys()

    def execute(self, gateway, command: str, *args):
        cmd = command.lower()
        if cmd not in self._commands:
            raise ValueError(f"Unknown command '{command}'")

        handler = self._commands[cmd]["handler"]

        if callable(handler):
            return handler(gateway, *args)
        
        # Return the string if it's not callable
        return handler