import os
import importlib.util
from lora_gateway import LoRaGateway

def main():
    gw = LoRaGateway()

    print("Initializing gateway...")
    gw.setup()

    print("Gateway ready")
    print("UID:", gw.lora.UID)
    print("RF:", gw.lora.rf_parameters)

    print("Registering commands...")

    if not os.path.exists("commands"):
        os.mkdir("commands")

    # Get list of command files in ./commands/ and register them
    commands_dir = os.path.join(os.path.dirname(__file__), "commands")

    for filename in os.listdir(commands_dir):
        if filename.endswith(".py") and filename != "__init__.py":
            module_name = filename[:-3]
            module_path = os.path.join(commands_dir, filename)

            print(f"Loading command module: {filename}")

            spec = importlib.util.spec_from_file_location(module_name, module_path)
            
            if spec is None or spec.loader is None:
                print(f"Failed to load spec for {filename}")
                continue
            
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            if hasattr(module, "register_commands"):
                module.register_commands(gw.commands)
                print(f"Registered commands from {filename}")

    print("Available commands:", ", ".join(gw.commands.list_commands()))

    try:
        gw.start()
    except KeyboardInterrupt:
        print("Stopping gateway")
        gw.stop()

if __name__ == "__main__":
    main()