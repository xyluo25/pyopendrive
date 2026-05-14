from __future__ import annotations

import argparse
from pathlib import Path

from . import DEFAULT_XODR, run_server


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the pyopendrive web editor.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--xodr", default=str(DEFAULT_XODR))
    args = parser.parse_args()

    default_xodr = Path(args.xodr) if args.xodr else None
    server, url = run_server(
        host=args.host,
        port=args.port,
        open_browser=not args.no_browser,
        default_xodr=default_xodr,
    )
    print(url, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
