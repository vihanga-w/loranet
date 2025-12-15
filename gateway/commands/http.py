# Provides HTTP/HTTPS internet uplink for gateway

from commands_handler import CommandRegistry
import tls_client
import json

# LoRa has a small payload size so save characters where possible
METHOD_MAP = {
    "G": "GET",
    "Po": "POST",
    "Pu": "PUT",
    "Op": "OPTIONS"
}

sessions = {}

def register_commands(registry: CommandRegistry):
    # Format: <Optional["sid"=<sesh_id>]> <method: G | Po | Pu | Op> <url>
    def http_command(gateway, *argv):
        # Mutable
        args: list[str] = list(argv[0])

        # Must have at least method + url
        if len(args) < 2:
            return b"ERR: Invalid request, unacceptable argument length"
        
        # TODO: Implement a mechanism of gateways sharing sessions
        # TODO: This is important because the client may not always choose the same gw (in cases of a better route appearing)

        session = tls_client.Session(
            client_identifier="chrome_120",
        )

        sesh_id = None

        # Get session id if available
        for i, arg in enumerate(args):
            if arg.startswith("sid="):
                sesh_id = arg.split("sid=", 1)[1]
                del args[i]
                break

        # TODO: Consider making it fail-safe (fallback to creating new session)
        if sesh_id and sesh_id not in sessions:
            return b"ERR: Invalid request, session not be found"
        elif sesh_id:
            session = sessions[sesh_id]
        
        if args[0] not in METHOD_MAP:
            return f"ERR: Invalid request, invalid provided method: \"{args[0]}\"".encode()
        
        method = METHOD_MAP[args[0]]
        url = args[1]

        # Check that the url is valid
        if not url.startswith("http://") and not url.startswith("https://"):
            return b"ERR: Invalid URL"

        print(f"[http] Sending {method} request to {url}")

        response = None

        if method == "GET":
            # TODO: Handle headers
            response = session.get(
                url,
            )
        elif method == "POST":
            # TODO: Handle body + headers
            response = session.post(
                url,
            )
        elif method == "PUT":
            # TODO: Handle body + headers
            response = session.put(
                url,
            )
        elif method == "OPTIONS":
            response = session.options(
                url,
            )
        
        if not response:
            return b"ERR: Request failed, no response"
        
        res = {
            "code": response.status_code,
            "headers": dict(response.headers),
            "cookies": response.cookies.get_dict(),
            "body": response.text,
        }
        
        # JSON serialise the response

        return json.dumps(res).encode()

    registry.register(
        "http",
        http_command,
        help="Returns response from HTTP request"
    )